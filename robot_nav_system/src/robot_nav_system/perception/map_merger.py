"""Multi-robot semantic map merger.

Merges per-robot semantic maps into a single global semantic map by:
1. Collecting all objects from each robot's local semantic map
2. Deduplicating objects by spatial proximity + class name matching
3. Averaging positions for merged duplicates
4. Outputting a unified global semantic map
"""
from __future__ import annotations

import json
import math
import threading
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..logging_setup import get_logger

log = get_logger("map_merger")


class ObjectCluster:
    """A cluster of observations of the same physical object from different robots."""

    __slots__ = ("class_name", "observations", "merged_position")

    def __init__(self, class_name: str, first_obs: Dict[str, Any], source_robot: str):
        self.class_name = class_name
        self.observations: List[Tuple[str, Dict[str, Any]]] = [(source_robot, first_obs)]
        self.merged_position: Optional[Tuple[float, float, float]] = None

    def try_merge(self, obj: Dict[str, Any], source_robot: str, distance_threshold: float) -> bool:
        """Try to merge obj into this cluster. Returns True if merged."""
        obj_pos = self._extract_pos(obj)
        for _, existing in self.observations:
            existing_pos = self._extract_pos(existing)
            dist = math.hypot(obj_pos[0] - existing_pos[0], obj_pos[1] - existing_pos[1])
            if dist <= distance_threshold:
                self.observations.append((source_robot, obj))
                self.merged_position = None  # invalidate cache
                return True
        return False

    def get_merged_position(self) -> Tuple[float, float, float]:
        """Average position across all observations."""
        if self.merged_position is not None:
            return self.merged_position

        xs, ys, yaws = [], [], []
        for _, obs in self.observations:
            pos = self._extract_pos(obs)
            xs.append(pos[0])
            ys.append(pos[1])
            yaws.append(pos[2])

        n = len(xs)
        # Average yaw using circular mean
        sin_sum = sum(math.sin(y) for y in yaws)
        cos_sum = sum(math.cos(y) for y in yaws)
        avg_yaw = math.atan2(sin_sum / n, cos_sum / n)

        self.merged_position = (sum(xs) / n, sum(ys) / n, avg_yaw)
        return self.merged_position

    def get_source_robots(self) -> List[str]:
        """Return list of robots that observed this object."""
        return list(set(r for r, _ in self.observations))

    @staticmethod
    def _extract_pos(obj: Dict[str, Any]) -> Tuple[float, float, float]:
        """Extract (x, y, yaw) from either DMROS or nav format."""
        # Nav format: position.{x, y, yaw}
        pos = obj.get("position", {})
        if isinstance(pos, dict) and "x" in pos:
            return (float(pos["x"]), float(pos["y"]), float(pos.get("yaw", 0.0)))
        # DMROS format: bbox_2d.center = [x, y, ...]
        bbox = obj.get("bbox_2d", {})
        center = bbox.get("center", [0.0, 0.0])
        x = float(center[0]) if len(center) > 0 else 0.0
        y = float(center[1]) if len(center) > 1 else 0.0
        yaw = float(center[2]) if len(center) > 2 else 0.0
        return (x, y, yaw)


class SemanticMapMerger:
    """Merges multiple per-robot semantic maps into a global semantic map.

    Each robot produces its own DMROS semantic_map.json. This merger:
    1. Loads all per-robot maps
    2. Groups objects by class_name
    3. Deduplicates by spatial proximity (same class + close position = same object)
    4. Averages positions across robot observations
    5. Writes a unified global semantic map

    The geometric occupancy grid merging is handled separately by the
    `multirobot_map_merge` ROS package.
    """

    def __init__(
        self,
        merge_distance: float = 1.0,
        position_format: str = "dmros",
    ):
        """
        Args:
            merge_distance: Max distance (meters) to consider two detections as
                            the same physical object.
            position_format: "dmros" for DMROS bbox_2d format, "nav" for nav
                             position format. Determines how positions are read
                             from per-robot maps.
        """
        self.merge_distance = merge_distance
        self.position_format = position_format
        self._lock = threading.Lock()

    def merge(
        self,
        robot_maps: Dict[str, List[Dict[str, Any]]],
    ) -> List[Dict[str, Any]]:
        """Merge per-robot object lists into a deduplicated global list.

        Args:
            robot_maps: {robot_name: [list of object dicts]}

        Returns:
            List of merged object dicts in DMROS-like format (class_name, bbox_2d).
        """
        with self._lock:
            return self._merge_impl(robot_maps)

    def _merge_impl(
        self,
        robot_maps: Dict[str, List[Dict[str, Any]]],
    ) -> List[Dict[str, Any]]:
        # Group clusters by class_name
        clusters_by_class: Dict[str, List[ObjectCluster]] = defaultdict(list)

        for robot_name, objects in robot_maps.items():
            for obj in objects:
                cls = str(obj.get("class_name", "object")).strip() or "object"
                merged = False
                for cluster in clusters_by_class[cls]:
                    if cluster.try_merge(obj, robot_name, self.merge_distance):
                        merged = True
                        break
                if not merged:
                    clusters_by_class[cls].append(ObjectCluster(cls, obj, robot_name))

        # Build merged output
        merged_objects: List[Dict[str, Any]] = []
        for cls, clusters in clusters_by_class.items():
            for cluster in clusters:
                x, y, yaw = cluster.get_merged_position()
                sources = cluster.get_source_robots()

                # Collect the best extent from the observation with most data
                best_extent = [0.0, 0.0]
                for _, obs in cluster.observations:
                    bbox = obs.get("bbox_2d", {})
                    ext = bbox.get("extent", [0.0, 0.0])
                    ex = abs(float(ext[0])) if len(ext) > 0 else 0.0
                    ey = abs(float(ext[1])) if len(ext) > 1 else 0.0
                    if ex * ey > best_extent[0] * best_extent[1]:
                        best_extent = [ex, ey]

                merged_objects.append({
                    "class_name": cls,
                    "bbox_2d": {
                        "center": [round(x, 4), round(y, 4), round(yaw, 4)],
                        "extent": [round(best_extent[0], 4), round(best_extent[1], 4)],
                    },
                    "observation_count": len(cluster.observations),
                    "source_robots": sources,
                })

        log.info(
            "Merged %d total observations -> %d unique objects (from %d robots)",
            sum(len(objs) for objs in robot_maps.values()),
            len(merged_objects),
            len(robot_maps),
        )
        return merged_objects

    def merge_and_save(
        self,
        robot_maps: Dict[str, List[Dict[str, Any]]],
        output_path: Path,
    ) -> Path:
        """Merge and write global DMROS-format semantic map."""
        merged = self.merge(robot_maps)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as f:
            json.dump(
                {
                    "metadata": {
                        "total_objects": len(merged),
                        "source_robots": sorted(robot_maps.keys()),
                        "merge_distance": self.merge_distance,
                    },
                    "objects": merged,
                },
                f,
                ensure_ascii=False,
                indent=2,
            )
        return output_path

    @staticmethod
    def load_dmros_objects(path: Path) -> List[Dict[str, Any]]:
        """Load objects from a DMROS semantic_map.json file."""
        if not path.exists():
            return []
        try:
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                return data.get("objects", [])
            if isinstance(data, list):
                return data
        except (json.JSONDecodeError, OSError) as e:
            log.warning("Failed to load DMROS map %s: %s", path, e)
        return []
