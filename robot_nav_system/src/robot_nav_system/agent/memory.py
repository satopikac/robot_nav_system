"""Dialogue context memory (bounded circular buffer)."""
from __future__ import annotations

from collections import deque
from typing import Deque, Dict, List


class DialogueMemory:
    def __init__(self, max_turns: int = 8):
        self.max_turns = max(1, max_turns)
        self._messages: Deque[Dict[str, str]] = deque(maxlen=self.max_turns * 2)

    def add_user(self, text: str) -> None:
        self._messages.append({"role": "user", "content": text})

    def add_assistant(self, text: str) -> None:
        self._messages.append({"role": "assistant", "content": text})

    def recent(self) -> List[Dict[str, str]]:
        return list(self._messages)

    def clear(self) -> None:
        self._messages.clear()
