"""Semantic map with fuzzy object matching and hot-reload support."""
from __future__ import annotations

import json
import threading
from difflib import SequenceMatcher
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..agent.models import SemanticObject
from ..exceptions import SemanticMapError
from ..logging_setup import get_logger

log = get_logger("semantic_map")


class SemanticMap:
    """Manages the semantic map with thread-safe hot-reload capability."""

    def __init__(self, objects: List[SemanticObject], file_path: Optional[Path] = None):
        self._objects = objects
        self._id_index = {o.obj_id: o for o in objects}
        self._file_path = file_path
        self._version = 0
        self._lock = threading.Lock()

    @property
    def objects(self) -> List[SemanticObject]:
        with self._lock:
            return list(self._objects)

    @property
    def version(self) -> int:
        return self._version

    @property
    def file_path(self) -> Optional[Path]:
        return self._file_path

    @property
    def is_empty(self) -> bool:
        with self._lock:
            return len(self._objects) == 0

    @classmethod
    def from_json(cls, path: str | Path) -> "SemanticMap":
        path_obj = Path(path)
        objects = cls._load_objects(path_obj)
        return cls(objects, file_path=path_obj)

    @classmethod
    def empty(cls, file_path: Optional[str | Path] = None) -> "SemanticMap":
        """Create an empty semantic map (exploration will populate it)."""
        fp = Path(file_path) if file_path else None
        return cls([], file_path=fp)

    @staticmethod
    def _load_objects(path: Path) -> List[SemanticObject]:
        if not path.exists():
            raise SemanticMapError(f"Semantic map not found: {path}")
        try:
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except json.JSONDecodeError as e:
            raise SemanticMapError(f"Invalid semantic map JSON: {e}") from e
        except OSError as e:
            raise SemanticMapError(f"Failed to read semantic map: {e}") from e

        if not isinstance(data, dict) or "objects" not in data:
            raise SemanticMapError("Semantic map must contain top-level key 'objects'.")

        objects: List[SemanticObject] = []
        for idx, item in enumerate(data.get("objects", [])):
            try:
                position = item["position"]
                objects.append(
                    SemanticObject(
                        obj_id=str(item["id"]),
                        name=str(item["name"]),
                        aliases=[str(a) for a in item.get("aliases", [])],
                        category=str(item.get("category", "")),
                        room=str(item.get("room", "")),
                        position=(
                            float(position["x"]),
                            float(position["y"]),
                            float(position.get("yaw", 0.0)),
                        ),
                        description=str(item.get("description", "")),
                    )
                )
            except (KeyError, TypeError, ValueError) as e:
                raise SemanticMapError(f"Invalid object at index {idx}: {e}") from e

        return objects

    def reload(self) -> bool:
        """Re-read from disk. Returns True if the map changed.

        Thread-safe. On error, keeps the previous version and logs the error.
        """
        if not self._file_path or not self._file_path.exists():
            return False
        try:
            new_objects = self._load_objects(self._file_path)
        except SemanticMapError as e:
            log.warning("Semantic map reload failed, keeping previous version: %s", e)
            return False

        with self._lock:
            old_count = len(self._objects)
            self._objects = new_objects
            self._id_index = {o.obj_id: o for o in new_objects}
            self._version += 1

        if len(new_objects) != old_count:
            log.info("Semantic map reloaded: %d -> %d objects (v%d)",
                     old_count, len(new_objects), self._version)
            return True
        return True

    def as_prompt_brief(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [
                {
                    "id": obj.obj_id,
                    "name": obj.name,
                    "aliases": obj.aliases,
                    "category": obj.category,
                    "room": obj.room,
                    "description": obj.description,
                }
                for obj in self._objects
            ]

    def find_by_id(self, obj_id: str) -> Optional[SemanticObject]:
        with self._lock:
            return self._id_index.get(obj_id)

    def _string_score(self, a: str, b: str) -> float:
        if not a or not b:
            return 0.0
        a_l, b_l = a.lower(), b.lower()
        if a_l == b_l:
            return 1.0
        if a_l in b_l or b_l in a_l:
            return 0.9
        return SequenceMatcher(None, a_l, b_l).ratio()

    def match_object(
        self, query: str, threshold: float = 0.48
    ) -> Tuple[Optional[SemanticObject], float]:
        query = (query or "").strip()
        if not query:
            return None, 0.0

        with self._lock:
            objects = list(self._objects)

        best_obj: Optional[SemanticObject] = None
        best_score = 0.0
        for obj in objects:
            candidates = obj.all_keywords()
            score = max(
                (self._string_score(query, cand) for cand in candidates), default=0.0
            )
            query_tokens = [tok for tok in query.lower().split() if tok]
            if query_tokens:
                for tok in query_tokens:
                    if any(tok in c.lower() for c in candidates):
                        score = max(score, 0.72)
            if score > best_score:
                best_score = score
                best_obj = obj

        if best_score < threshold:
            return None, best_score
        return best_obj, best_score
