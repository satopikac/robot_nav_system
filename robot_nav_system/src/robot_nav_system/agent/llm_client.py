"""LLM client for task planning and summarization."""
from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

try:
    from openai import OpenAI
except ImportError:
    OpenAI = None  # type: ignore

from ..config import Config
from ..exceptions import LLMError
from .models import (
    AllocStrategy,
    ExecutionRecord,
    NavMode,
    ProgressSnapshot,
    SubTask,
    TaskPlan,
)

PLAN_SYSTEM_PROMPT = """\
你是机器人导航任务规划器，同时支持【单机器人】和【多机器人】两种模式。

可用机器人列表: {robots}

决策规则:
1. 任务需要一个机器人按顺序去多个地点 → mode="single"
2. 任务需要多个机器人各自去不同地点或并行工作 → mode="multi"
3. 如果用户明确要求"多机器人"、"三个机器人"、"分别"等关键词 → mode="multi"
4. 如果用户明确指定了哪个机器人做什么 → mode="multi" 且在 subtasks 中设置 assigned_robot

多机器人额外规则:
- alloc_strategy: "greedy"(贪心最近优先) 或 "hungarian"(匈牙利最优分配)
- depends_on: 若子任务B必须在子任务A完成后才能开始，设 B 的 depends_on 为 A 的索引(从0开始)
- 无依赖关系时 depends_on 设为 null

仅能使用语义地图中已有物体进行规划，不要添加地图中不存在的目标。
输出严格JSON，不要输出其他文字。

输出格式:
{{
  "mode": "single" | "multi",
  "alloc_strategy": "greedy" | "hungarian",
  "subtasks": [
    {{
      "action": "navigate",
      "target": "物体名",
      "reason": "原因",
      "depends_on": null | 子任务索引,
      "assigned_robot": null | "机器人名"
    }}
  ],
  "notes": "说明"
}}
子任务最多 {max_subtasks} 个。
"""

SUMMARIZE_SYSTEM_PROMPT = "你是机器人执行结果总结助手，请输出简明中文总结。"

INTERRUPT_SYSTEM_PROMPT = "用户中断了机器人任务，请给出一句中文确认回应。"


class LLMClient:
    def __init__(self, config: Config):
        self.config = config
        self.model = str(config.get("llm.model", "deepseek-chat"))
        self.temperature = float(config.get("llm.temperature", 0.2))
        self.max_tokens = int(config.get("llm.max_tokens", 1024))
        self.timeout = int(config.get("llm.timeout", 45))
        self.api_key = str(config.get("llm.api_key", "")).strip()
        base_url = str(config.get("llm.base_url", "https://api.deepseek.com")).strip()
        self.client: Optional[Any] = None
        if self.api_key:
            if OpenAI is None:
                raise LLMError("openai SDK 未安装，请先 pip install openai>=1.0.0")
            self.client = OpenAI(
                api_key=self.api_key, base_url=base_url, timeout=self.timeout
            )

    def _chat(self, messages: List[Dict[str, str]]) -> str:
        if not self.client:
            raise LLMError("No API key configured for LLM.")
        try:
            resp = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
            )
            content = resp.choices[0].message.content
            if not content:
                raise LLMError("LLM returned empty content.")
            return content
        except LLMError:
            raise
        except Exception as e:
            raise LLMError(f"LLM request failed: {e}") from e

    def _extract_json(self, text: str) -> Dict[str, Any]:
        raw = text.strip()
        if raw.startswith("```"):
            raw = raw.strip("`")
            if raw.startswith("json"):
                raw = raw[4:].strip()
        start = raw.find("{")
        end = raw.rfind("}")
        if start < 0 or end < 0 or end <= start:
            raise LLMError("Cannot find JSON object in LLM response.")
        fragment = raw[start : end + 1]
        try:
            return json.loads(fragment)
        except json.JSONDecodeError as e:
            raise LLMError(f"Failed to parse LLM JSON: {e}") from e

    def _fallback_plan(
        self, instruction: str, semantic_objects: List[Dict[str, Any]], robots: List[str]
    ) -> TaskPlan:
        instruction_l = instruction.lower()
        is_multi = any(
            kw in instruction_l
            for kw in ("多机", "多个机器人", "三个机器人", "分别", "并行", "同时")
        )
        chosen: List[Dict[str, Any]] = []
        for obj in semantic_objects:
            keys = [
                obj.get("name", ""),
                *obj.get("aliases", []),
                obj.get("category", ""),
                obj.get("room", ""),
            ]
            if any(k and k.lower() in instruction_l for k in keys):
                chosen.append(obj)

        subtasks = [
            SubTask(action="navigate", target_name=str(o["name"]), reason="fallback keyword match")
            for o in chosen
        ]

        mode = NavMode.MULTI if (is_multi and len(subtasks) > 1) else NavMode.SINGLE
        return TaskPlan(
            original_instruction=instruction,
            mode=mode,
            subtasks=subtasks,
            alloc_strategy=AllocStrategy.GREEDY,
            notes="fallback plan (no API key or LLM failed)",
        )

    def plan_tasks(
        self,
        instruction: str,
        semantic_objects: List[Dict[str, Any]],
        progress: ProgressSnapshot,
        recent_dialogue: List[Dict[str, str]],
        robots: List[str],
        max_subtasks: int = 8,
    ) -> TaskPlan:
        if not self.client:
            return self._fallback_plan(instruction, semantic_objects, robots)

        system_prompt = PLAN_SYSTEM_PROMPT.format(
            robots=", ".join(robots), max_subtasks=max_subtasks
        )
        user_payload = {
            "instruction": instruction,
            "semantic_map_objects": semantic_objects,
            "available_robots": robots,
            "current_progress": {
                "original_instruction": progress.original_instruction,
                "completed": progress.completed,
                "in_progress": progress.in_progress,
                "pending": progress.pending,
            },
            "recent_dialogue": recent_dialogue,
        }
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": json.dumps(user_payload, ensure_ascii=False)},
        ]

        content = self._chat(messages)
        parsed = self._extract_json(content)

        mode_str = str(parsed.get("mode", "single")).strip().lower()
        mode = NavMode.MULTI if mode_str == "multi" else NavMode.SINGLE

        alloc_str = str(parsed.get("alloc_strategy", "greedy")).strip().lower()
        alloc_strategy = (
            AllocStrategy.HUNGARIAN if alloc_str == "hungarian" else AllocStrategy.GREEDY
        )

        subtasks_raw = parsed.get("subtasks", [])
        if not isinstance(subtasks_raw, list):
            raise LLMError("LLM subtasks format invalid.")

        subtasks: List[SubTask] = []
        for item in subtasks_raw[:max_subtasks]:
            if not isinstance(item, dict):
                continue
            action = str(item.get("action", "navigate")).strip() or "navigate"
            target = str(item.get("target", "")).strip()
            if not target:
                continue
            reason = str(item.get("reason", "")).strip()
            depends_on_raw = item.get("depends_on")
            depends_on = int(depends_on_raw) if depends_on_raw is not None else None
            assigned_robot = item.get("assigned_robot")
            if assigned_robot is not None:
                assigned_robot = str(assigned_robot).strip() or None

            subtasks.append(
                SubTask(
                    action=action,
                    target_name=target,
                    reason=reason,
                    depends_on=depends_on,
                    assigned_robot=assigned_robot,
                )
            )

        return TaskPlan(
            original_instruction=instruction,
            mode=mode,
            subtasks=subtasks,
            alloc_strategy=alloc_strategy,
            notes=str(parsed.get("notes", "")).strip(),
        )

    def summarize_task(
        self,
        original_instruction: str,
        records: List[ExecutionRecord],
        progress: ProgressSnapshot,
        recent_dialogue: List[Dict[str, str]],
    ) -> str:
        if not records:
            return "没有可总结的任务执行记录。"

        if not self.client:
            success_count = sum(1 for r in records if r.success)
            robot_set = {r.robot for r in records if r.robot}
            mode_hint = f"（多机: {', '.join(sorted(robot_set))}）" if robot_set else "（单机）"
            return (
                f"任务结束{mode_hint}：共 {len(records)} 步，"
                f"成功 {success_count} 步，失败 {len(records) - success_count} 步。"
            )

        rec_json = [
            {
                "step_index": r.step_index,
                "action": r.action,
                "target_name": r.target_name,
                "target_obj_id": r.target_obj_id,
                "success": r.success,
                "message": r.message,
                "robot": r.robot,
            }
            for r in records
        ]
        user_payload = {
            "original_instruction": original_instruction,
            "execution_records": rec_json,
            "current_progress": {
                "original_instruction": progress.original_instruction,
                "mode": progress.mode,
                "completed": progress.completed,
                "in_progress": progress.in_progress,
                "pending": progress.pending,
            },
            "recent_dialogue": recent_dialogue,
        }
        text = self._chat(
            [
                {"role": "system", "content": SUMMARIZE_SYSTEM_PROMPT},
                {"role": "user", "content": json.dumps(user_payload, ensure_ascii=False)},
            ]
        )
        return text.strip()

    def notify_interrupt(
        self, interrupted_instruction: str, reason: str = "user_interrupt"
    ) -> str:
        if not self.client:
            return f"任务已中断：{interrupted_instruction}"
        payload = {"interrupted_instruction": interrupted_instruction, "reason": reason}
        text = self._chat(
            [
                {"role": "system", "content": INTERRUPT_SYSTEM_PROMPT},
                {"role": "user", "content": json.dumps(payload, ensure_ascii=False)},
            ]
        )
        return text.strip()
