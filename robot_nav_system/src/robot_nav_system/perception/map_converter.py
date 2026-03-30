"""Convert DMROS semantic_map.json to unified navigation agent format.

Two modes:
- convert(): Full conversion with LLM-generated Chinese metadata
- convert_fast(): Quick conversion without LLM (uses English class names directly)
"""
from __future__ import annotations

import json
import math
import re
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..config import Config
from ..exceptions import MapConversionError
from ..logging_setup import get_logger

log = get_logger("map_converter")


def _sanitize_id_token(value: str) -> str:
    token = value.strip().lower()
    token = re.sub(r"[^a-z0-9]+", "_", token)
    token = token.strip("_")
    return token or "object"


def _shorten(text: str, max_chars: int) -> str:
    text = (text or "").strip()
    return text[:max_chars] if len(text) > max_chars else text


def _near_conflict(
    candidate: Tuple[float, float],
    placed: Sequence[Tuple[float, float]],
    robot_length: float,
    robot_width: float,
    margin: float,
) -> bool:
    radius = 0.5 * math.hypot(robot_length, robot_width)
    min_dist = 2.0 * radius + margin
    for px, py in placed:
        if math.hypot(candidate[0] - px, candidate[1] - py) < min_dist:
            return True
    return False


def _overlap_bbox(
    candidate: Tuple[float, float],
    center: Tuple[float, float],
    extent: Tuple[float, float],
    robot_length: float,
    robot_width: float,
    margin: float,
) -> bool:
    dx = abs(candidate[0] - center[0])
    dy = abs(candidate[1] - center[1])
    return (dx <= 0.5 * extent[0] + 0.5 * robot_length + margin and
            dy <= 0.5 * extent[1] + 0.5 * robot_width + margin)


def _pick_stop_pose(
    obj_x: float, obj_y: float, obj_yaw: float,
    obj_ex: float, obj_ey: float,
    placed: Sequence[Tuple[float, float]],
    robot_length: float, robot_width: float, margin: float,
) -> Tuple[float, float, float]:
    base_offset = 0.5 * max(obj_ex, obj_ey) + 0.5 * max(robot_length, robot_width) + margin
    radii = [base_offset + i * 0.2 for i in range(8)]
    angles = [
        obj_yaw + math.pi,
        obj_yaw + math.pi / 2,
        obj_yaw - math.pi / 2,
        obj_yaw,
        obj_yaw + 3 * math.pi / 4,
        obj_yaw - 3 * math.pi / 4,
        obj_yaw + math.pi / 4,
        obj_yaw - math.pi / 4,
    ]
    for r in radii:
        for a in angles:
            cx = obj_x + r * math.cos(a)
            cy = obj_y + r * math.sin(a)
            if _near_conflict((cx, cy), placed, robot_length, robot_width, margin):
                continue
            if _overlap_bbox((cx, cy), (obj_x, obj_y), (obj_ex, obj_ey),
                             robot_length, robot_width, margin):
                continue
            face_yaw = math.atan2(obj_y - cy, obj_x - cx)
            return cx, cy, face_yaw

    fx = obj_x + base_offset + 1.0
    fy = obj_y
    return fx, fy, math.atan2(obj_y - fy, obj_x - fx)


def _extract_bbox2d(obj: dict) -> Tuple[float, float, float, float, float]:
    """Extract (x, y, yaw, extent_x, extent_y) from DMROS object."""
    # DMROS format: bbox_2d.center = [x, y], bbox_2d.extent = [ex, ey]
    bbox_2d = obj.get("bbox_2d", {})
    center = bbox_2d.get("center", [0.0, 0.0])
    extent = bbox_2d.get("extent", [0.0, 0.0])

    x = float(center[0]) if len(center) > 0 else 0.0
    y = float(center[1]) if len(center) > 1 else 0.0
    yaw = float(center[2]) if len(center) > 2 else 0.0
    ex = abs(float(extent[0])) if len(extent) > 0 else 0.0
    ey = abs(float(extent[1])) if len(extent) > 1 else 0.0
    return x, y, yaw, ex, ey


class MapConverter:
    """Converts DMROS semantic map output to unified navigation agent format."""

    def __init__(self, config: Config):
        self.robot_length = float(config.get("converter.robot_length", 0.281))
        self.robot_width = float(config.get("converter.robot_width", 0.306))
        self.safety_margin = float(config.get("converter.safety_margin", 0.15))
        self.use_llm = bool(config.get("dmros.use_llm_for_metadata", False))

        # LLM settings for full conversion
        self.api_key = str(config.get("llm.api_key", "")).strip()
        self.base_url = str(config.get("llm.base_url", "")).strip()
        self.model = str(config.get("llm.model", "deepseek-chat"))

    def convert_fast(self, dmros_map_path: Path, output_path: Path) -> Path:
        """Quick conversion without LLM -- uses English class_name directly."""
        raw_objects = self._load_dmros_objects(dmros_map_path)
        if not raw_objects:
            log.warning("DMROS map is empty, skipping conversion")
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with output_path.open("w", encoding="utf-8") as f:
                json.dump({"objects": []}, f, ensure_ascii=False, indent=2)
            return output_path

        counters: Dict[str, int] = defaultdict(int)
        placed: List[Tuple[float, float]] = []
        out_objects: List[Dict[str, Any]] = []

        for obj in raw_objects:
            cls_name = str(obj.get("class_name", "object")).strip() or "object"
            counters[cls_name] += 1
            token = _sanitize_id_token(cls_name)
            obj_id = f"{token}_{counters[cls_name]:02d}"

            x, y, yaw, ex, ey = _extract_bbox2d(obj)
            px, py, pyaw = _pick_stop_pose(
                x, y, yaw, ex, ey, placed,
                self.robot_length, self.robot_width, self.safety_margin,
            )
            placed.append((px, py))

            out_objects.append({
                "id": obj_id,
                "name": _shorten(cls_name, 16),
                "aliases": [cls_name],
                "category": "other",
                "room": "",
                "position": {"x": round(px, 4), "y": round(py, 4), "yaw": round(pyaw, 4)},
                "description": cls_name,
            })

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as f:
            json.dump({"objects": out_objects}, f, ensure_ascii=False, indent=2)

        log.info("Fast-converted %d objects -> %s", len(out_objects), output_path)
        return output_path

    def convert(self, dmros_map_path: Path, output_path: Path) -> Path:
        """Full conversion with LLM-generated Chinese metadata."""
        if not self.api_key or not self.use_llm:
            return self.convert_fast(dmros_map_path, output_path)

        try:
            from openai import OpenAI
        except ImportError:
            log.warning("openai not installed, falling back to fast conversion")
            return self.convert_fast(dmros_map_path, output_path)

        raw_objects = self._load_dmros_objects(dmros_map_path)
        if not raw_objects:
            log.warning("DMROS map is empty, skipping conversion")
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with output_path.open("w", encoding="utf-8") as f:
                json.dump({"objects": []}, f, ensure_ascii=False, indent=2)
            return output_path

        class_names = sorted({str(o.get("class_name", "object")).strip() or "object"
                              for o in raw_objects})

        client = OpenAI(api_key=self.api_key, base_url=self.base_url)

        # Generate text metadata via LLM
        text_map = self._generate_metadata(class_names, client)

        # Infer room names via LLM
        room_names = self._infer_rooms(raw_objects, client)

        # Build output
        counters: Dict[str, int] = defaultdict(int)
        placed: List[Tuple[float, float]] = []
        out_objects: List[Dict[str, Any]] = []

        for idx, obj in enumerate(raw_objects):
            cls_name = str(obj.get("class_name", "object")).strip() or "object"
            counters[cls_name] += 1
            token = _sanitize_id_token(cls_name)
            obj_id = f"{token}_{counters[cls_name]:02d}"

            x, y, yaw, ex, ey = _extract_bbox2d(obj)
            px, py, pyaw = _pick_stop_pose(
                x, y, yaw, ex, ey, placed,
                self.robot_length, self.robot_width, self.safety_margin,
            )
            placed.append((px, py))

            meta = text_map.get(cls_name, {})
            out_objects.append({
                "id": obj_id,
                "name": meta.get("name", cls_name),
                "aliases": meta.get("aliases", [cls_name]),
                "category": meta.get("category", "其他"),
                "room": room_names[idx] if idx < len(room_names) else "未知区域",
                "position": {"x": round(px, 4), "y": round(py, 4), "yaw": round(pyaw, 4)},
                "description": meta.get("description", f"{cls_name}目标"),
            })

        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as f:
            json.dump({"objects": out_objects}, f, ensure_ascii=False, indent=2)

        log.info("Full-converted %d objects -> %s", len(out_objects), output_path)
        return output_path

    def _load_dmros_objects(self, path: Path) -> List[dict]:
        """Load objects from DMROS semantic_map.json."""
        if not path.exists():
            raise MapConversionError(f"DMROS map not found: {path}")
        try:
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            raise MapConversionError(f"Failed to read DMROS map: {e}") from e

        if isinstance(data, dict):
            return data.get("objects", [])
        if isinstance(data, list):
            return data
        return []

    def _generate_metadata(
        self, class_names: List[str], client: Any,
    ) -> Dict[str, dict]:
        """Generate Chinese metadata via LLM."""
        prompt = {
            "task": "为每个物体类别生成中文可读名称、别名和简短描述",
            "rules": {
                "language": "zh-CN",
                "name_max_chars": 8,
                "alias_count": 3,
                "alias_max_chars": 10,
                "description_max_chars": 24,
                "category_max_chars": 8,
                "preferred_categories": ["家具", "电器", "容器", "设备", "装饰", "工具", "站点", "其他"],
            },
            "input_class_names": class_names,
            "output_schema": {
                "items": [{"class_name": "string", "name": "string",
                           "aliases": ["string"], "category": "string", "description": "string"}]
            },
        }
        try:
            resp = client.chat.completions.create(
                model=self.model, temperature=0.2,
                messages=[
                    {"role": "system", "content": "你是机器人语义地图数据工程助手，只输出合法JSON。"},
                    {"role": "user", "content": json.dumps(prompt, ensure_ascii=False)},
                ],
            )
            content = resp.choices[0].message.content or ""
            parsed = json.loads(re.search(r"\{.*\}", content, re.S).group(0))
            mapping: Dict[str, dict] = {}
            for item in parsed.get("items", []):
                key = str(item.get("class_name", "")).strip()
                if key:
                    mapping[key] = {
                        "name": _shorten(str(item.get("name", key)), 8),
                        "aliases": [_shorten(str(a), 10) for a in item.get("aliases", [key])[:3]],
                        "category": _shorten(str(item.get("category", "其他")), 8),
                        "description": _shorten(str(item.get("description", f"{key}目标")), 24),
                    }
            # Fill missing
            for cls in class_names:
                if cls not in mapping:
                    mapping[cls] = {"name": _shorten(cls, 8), "aliases": [cls],
                                    "category": "其他", "description": f"{cls}目标"}
            return mapping
        except Exception as e:
            log.warning("LLM metadata generation failed, using defaults: %s", e)
            return {cls: {"name": _shorten(cls, 8), "aliases": [cls],
                          "category": "其他", "description": f"{cls}目标"}
                    for cls in class_names}

    def _infer_rooms(self, raw_objects: List[dict], client: Any) -> List[str]:
        """Infer room names via LLM-based clustering."""
        points = []
        class_names = []
        for obj in raw_objects:
            x, y, _, _, _ = _extract_bbox2d(obj)
            points.append((x, y))
            class_names.append(str(obj.get("class_name", "object")).strip() or "object")

        if not points:
            return []

        # Simple BFS clustering
        n = len(points)
        visited = [False] * n
        clusters: List[List[int]] = []
        for i in range(n):
            if visited[i]:
                continue
            queue, comp = [i], []
            visited[i] = True
            while queue:
                cur = queue.pop(0)
                comp.append(cur)
                for j in range(n):
                    if not visited[j] and math.hypot(
                            points[cur][0] - points[j][0],
                            points[cur][1] - points[j][1]) <= 2.0:
                        visited[j] = True
                        queue.append(j)
            clusters.append(comp)

        obj_to_cluster = [-1] * n
        cluster_payload = []
        for cid, indices in enumerate(clusters):
            for i in indices:
                obj_to_cluster[i] = cid
            cluster_payload.append({
                "cluster_id": cid,
                "class_names": [class_names[i] for i in indices],
            })

        cluster_room: Dict[int, str] = {}
        try:
            prompt = {
                "task": "推理每个聚类的房间名",
                "rules": {"language": "zh-CN", "room_name_max_chars": 8},
                "clusters": cluster_payload,
                "output_schema": {"items": [{"cluster_id": 0, "room": "客厅"}]},
            }
            resp = client.chat.completions.create(
                model=self.model, temperature=0.1,
                messages=[
                    {"role": "system", "content": "你是室内场景语义推理助手，只输出合法JSON。"},
                    {"role": "user", "content": json.dumps(prompt, ensure_ascii=False)},
                ],
            )
            content = resp.choices[0].message.content or ""
            parsed = json.loads(re.search(r"\{.*\}", content, re.S).group(0))
            for item in parsed.get("items", []):
                cid = int(item.get("cluster_id", -1))
                if cid >= 0:
                    cluster_room[cid] = _shorten(str(item.get("room", "未知区域")), 8)
        except Exception:
            pass

        return [cluster_room.get(obj_to_cluster[i], "未知区域") for i in range(n)]
