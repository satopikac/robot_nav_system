"""File watcher for DMROS semantic map output.

Polls the DMROS output directory for changes to semantic_map.json.
On change, triggers MapConverter and SemanticMap.reload().
"""
from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

from ..logging_setup import get_logger
from .map_converter import MapConverter
from .semantic_map import SemanticMap

log = get_logger("map_watcher")


class MapWatcher:
    """Background thread that watches DMROS output and triggers conversion + reload."""

    def __init__(
        self,
        dmros_output_path: Path,
        nav_map_path: Path,
        converter: MapConverter,
        semantic_map: SemanticMap,
        poll_interval: float = 3.0,
    ):
        self._dmros_path = dmros_output_path
        self._nav_map_path = nav_map_path
        self._converter = converter
        self._semantic_map = semantic_map
        self._poll_interval = max(0.5, poll_interval)

        self._last_mtime: float = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        """Start the background polling thread."""
        if self._running:
            return
        self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="MapWatcher")
        self._thread.start()
        log.info("MapWatcher started (watching %s, interval=%.1fs)",
                 self._dmros_path, self._poll_interval)

    def stop(self) -> None:
        """Stop the background polling thread."""
        if not self._running:
            return
        self._stop_event.set()
        self._running = False
        if self._thread:
            self._thread.join(timeout=self._poll_interval + 1.0)
            self._thread = None
        log.info("MapWatcher stopped")

    def force_update(self) -> bool:
        """Manually trigger conversion and reload. Returns True if map changed."""
        return self._try_convert_and_reload()

    def _poll_loop(self) -> None:
        """Main polling loop running in background thread."""
        while not self._stop_event.is_set():
            try:
                self._try_convert_and_reload()
            except Exception:
                log.exception("MapWatcher error (will retry next cycle)")
            self._stop_event.wait(self._poll_interval)

    def _try_convert_and_reload(self) -> bool:
        """Check if DMROS output changed, convert, and reload.

        Returns True if the semantic map was updated.
        """
        if not self._dmros_path.exists():
            return False

        try:
            current_mtime = self._dmros_path.stat().st_mtime
        except OSError:
            return False

        if current_mtime <= self._last_mtime:
            return False  # No change

        self._last_mtime = current_mtime
        log.debug("DMROS map changed (mtime=%.2f), converting...", current_mtime)

        try:
            self._converter.convert_fast(self._dmros_path, self._nav_map_path)
        except Exception:
            log.exception("Map conversion failed, keeping previous version")
            return False

        changed = self._semantic_map.reload()
        if changed:
            log.info("Semantic map updated to v%d (%d objects)",
                     self._semantic_map.version, len(self._semantic_map.objects))
        return changed
