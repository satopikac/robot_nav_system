"""Multi-robot map manager.

Coordinates per-robot DMROS instances, watches each robot's local semantic map,
merges them into a global semantic map, and triggers conversion + reload for
the NavigationAgent.

Architecture:
  Robot_0 RGBD → DMROS_0 → output/tb3_0/semantic/semantic_map.json ─┐
  Robot_1 RGBD → DMROS_1 → output/tb3_1/semantic/semantic_map.json ─┤→ Merger → global_dmros.json → Converter → nav semantic_map.json
  Robot_2 RGBD → DMROS_2 → output/tb3_2/semantic/semantic_map.json ─┘
"""
from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

from ..config import Config
from ..logging_setup import get_logger
from .map_converter import MapConverter
from .map_merger import SemanticMapMerger
from .semantic_map import SemanticMap

log = get_logger("multi_robot_map_manager")


class MultiRobotMapManager:
    """Manages per-robot semantic maps and merges them into a global map.

    Runs a background thread that:
    1. Polls each robot's DMROS output directory for changes
    2. Loads changed per-robot semantic maps
    3. Merges all robot maps using SemanticMapMerger (deduplication)
    4. Converts the merged DMROS-format map to nav format via MapConverter
    5. Triggers SemanticMap.reload() so the agent sees updated data
    """

    def __init__(
        self,
        config: Config,
        robots: List[str],
        base_output_dir: Path,
        global_dmros_map_path: Path,
        nav_map_path: Path,
        converter: MapConverter,
        semantic_map: SemanticMap,
        merge_distance: float = 1.0,
        poll_interval: float = 3.0,
    ):
        self._config = config
        self._robots = robots
        self._base_output_dir = base_output_dir
        self._global_dmros_path = global_dmros_map_path
        self._nav_map_path = nav_map_path
        self._converter = converter
        self._semantic_map = semantic_map
        self._poll_interval = max(0.5, poll_interval)

        self._merger = SemanticMapMerger(merge_distance=merge_distance)

        # Per-robot state tracking
        self._robot_map_paths: Dict[str, Path] = {}
        self._robot_mtimes: Dict[str, float] = {}
        for robot in robots:
            robot_map = base_output_dir / robot / "semantic" / "semantic_map.json"
            self._robot_map_paths[robot] = robot_map
            self._robot_mtimes[robot] = 0.0

        # Thread control
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Statistics
        self._merge_count = 0
        self._lock = threading.Lock()

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def merge_count(self) -> int:
        return self._merge_count

    def get_robot_map_status(self) -> Dict[str, Dict]:
        """Return status of each robot's local semantic map."""
        status = {}
        for robot in self._robots:
            path = self._robot_map_paths[robot]
            exists = path.exists()
            n_objects = 0
            if exists:
                objects = self._merger.load_dmros_objects(path)
                n_objects = len(objects)
            status[robot] = {
                "path": str(path),
                "exists": exists,
                "objects": n_objects,
                "last_mtime": self._robot_mtimes.get(robot, 0.0),
            }
        return status

    def start(self) -> None:
        """Start the background merge polling thread."""
        if self._running:
            return
        self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="MultiRobotMapManager"
        )
        self._thread.start()
        log.info(
            "MultiRobotMapManager started (robots=%s, poll=%.1fs, merge_dist=%.2fm)",
            self._robots, self._poll_interval, self._merger.merge_distance,
        )

    def stop(self) -> None:
        """Stop the background thread."""
        if not self._running:
            return
        self._stop_event.set()
        self._running = False
        if self._thread:
            self._thread.join(timeout=self._poll_interval + 2.0)
            self._thread = None
        log.info("MultiRobotMapManager stopped (total merges: %d)", self._merge_count)

    def force_merge(self) -> bool:
        """Manually trigger a merge cycle. Returns True if the global map changed."""
        return self._check_and_merge(force=True)

    def _poll_loop(self) -> None:
        """Main polling loop."""
        while not self._stop_event.is_set():
            try:
                self._check_and_merge()
            except Exception:
                log.exception("MultiRobotMapManager error (will retry)")
            self._stop_event.wait(self._poll_interval)

    def _check_and_merge(self, force: bool = False) -> bool:
        """Check for changes in any robot's map and merge if needed.

        Returns True if the global semantic map was updated.
        """
        any_changed = force

        # Check each robot's map for modification
        for robot in self._robots:
            path = self._robot_map_paths[robot]
            if not path.exists():
                continue
            try:
                mtime = path.stat().st_mtime
            except OSError:
                continue

            if mtime > self._robot_mtimes[robot]:
                self._robot_mtimes[robot] = mtime
                any_changed = True
                log.debug("Robot %s map updated (mtime=%.2f)", robot, mtime)

        if not any_changed:
            return False

        # Load all robot maps
        robot_maps: Dict[str, list] = {}
        total_objects = 0
        for robot in self._robots:
            path = self._robot_map_paths[robot]
            objects = self._merger.load_dmros_objects(path)
            if objects:
                robot_maps[robot] = objects
                total_objects += len(objects)

        if not robot_maps:
            return False

        log.info(
            "Merging maps: %d robots, %d total objects",
            len(robot_maps), total_objects,
        )

        # Merge and save global DMROS-format map
        try:
            self._merger.merge_and_save(robot_maps, self._global_dmros_path)
        except Exception:
            log.exception("Map merge failed")
            return False

        # Convert global DMROS map to nav format
        try:
            self._converter.convert_fast(self._global_dmros_path, self._nav_map_path)
        except Exception:
            log.exception("Map conversion failed after merge")
            return False

        # Reload the semantic map
        changed = self._semantic_map.reload()
        if changed:
            with self._lock:
                self._merge_count += 1
            log.info(
                "Global semantic map updated: %d objects (v%d, merge #%d)",
                len(self._semantic_map.objects),
                self._semantic_map.version,
                self._merge_count,
            )

        return changed
