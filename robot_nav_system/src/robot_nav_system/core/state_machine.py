"""Finite state machine for system mode management.

Enforces valid transitions between exploration and task execution modes.
"""

from __future__ import annotations

import threading
from enum import Enum
from typing import Callable, Dict, List, Optional, Set, Tuple

from ..exceptions import StateTransitionError
from ..logging_setup import get_logger

log = get_logger("state_machine")

# Type alias for transition callbacks
_Callback = Callable[[], None]


class SystemState(str, Enum):
    """All possible system states."""
    UNINITIALIZED = "uninitialized"
    IDLE = "idle"
    EXPLORING = "exploring"
    TASK_RECEIVED = "task_received"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    ERROR = "error"
    SHUTTING_DOWN = "shutting_down"


# Legal transitions: { from_state: {to_state, ...} }
_TRANSITIONS: Dict[SystemState, Set[SystemState]] = {
    SystemState.UNINITIALIZED: {SystemState.IDLE, SystemState.SHUTTING_DOWN},
    SystemState.IDLE: {
        SystemState.EXPLORING,
        SystemState.TASK_RECEIVED,
        SystemState.SHUTTING_DOWN,
    },
    SystemState.EXPLORING: {
        SystemState.TASK_RECEIVED,
        SystemState.IDLE,
        SystemState.ERROR,
        SystemState.SHUTTING_DOWN,
    },
    SystemState.TASK_RECEIVED: {
        SystemState.PLANNING,
        SystemState.ERROR,
        SystemState.SHUTTING_DOWN,
    },
    SystemState.PLANNING: {
        SystemState.EXECUTING,
        SystemState.ERROR,
        SystemState.SHUTTING_DOWN,
    },
    SystemState.EXECUTING: {
        SystemState.COMPLETED,
        SystemState.ERROR,
        SystemState.SHUTTING_DOWN,
    },
    SystemState.COMPLETED: {
        SystemState.IDLE,
        SystemState.EXPLORING,
        SystemState.SHUTTING_DOWN,
    },
    SystemState.ERROR: {
        SystemState.IDLE,
        SystemState.SHUTTING_DOWN,
    },
    SystemState.SHUTTING_DOWN: set(),  # terminal state
}


class SystemStateMachine:
    """Thread-safe finite state machine for the navigation system.

    Supports callbacks on entry to / exit from specific states.
    """

    def __init__(self) -> None:
        self._state = SystemState.UNINITIALIZED
        self._lock = threading.Lock()
        # callbacks[(from_state, to_state)] -> list of callables
        self._callbacks: Dict[Tuple[Optional[SystemState], Optional[SystemState]], List[_Callback]] = {}
        # entry/exit callbacks
        self._on_enter: Dict[SystemState, List[_Callback]] = {}
        self._on_exit: Dict[SystemState, List[_Callback]] = {}
        self._previous_state: Optional[SystemState] = None

    # ---- properties ----

    @property
    def state(self) -> SystemState:
        return self._state

    @property
    def previous_state(self) -> Optional[SystemState]:
        return self._previous_state

    # ---- transition logic ----

    def can_transition(self, target: SystemState) -> bool:
        """Check if the transition from current state to *target* is legal."""
        return target in _TRANSITIONS.get(self._state, set())

    def transition(self, target: SystemState) -> bool:
        """Perform a state transition.

        Returns True on success. Raises :class:`StateTransitionError` if
        the transition is illegal.

        Callbacks are executed *outside* the lock in this order:
        1. on_exit callbacks for the old state
        2. specific (from, to) callbacks
        3. on_enter callbacks for the new state
        """
        with self._lock:
            if not self.can_transition(target):
                raise StateTransitionError(
                    f"Illegal transition: {self._state.value} -> {target.value}"
                )
            old = self._state
            self._previous_state = old
            self._state = target

        log.info("State: %s -> %s", old.value, target.value)

        # Fire callbacks outside lock to avoid deadlocks
        for cb in self._on_exit.get(old, []):
            self._safe_call(cb, f"on_exit({old.value})")
        for cb in self._callbacks.get((old, target), []):
            self._safe_call(cb, f"transition({old.value}->{target.value})")
        for cb in self._on_enter.get(target, []):
            self._safe_call(cb, f"on_enter({target.value})")

        return True

    # ---- callback registration ----

    def on_enter(self, state: SystemState, callback: _Callback) -> None:
        """Register a callback to fire when entering *state*."""
        self._on_enter.setdefault(state, []).append(callback)

    def on_exit(self, state: SystemState, callback: _Callback) -> None:
        """Register a callback to fire when leaving *state*."""
        self._on_exit.setdefault(state, []).append(callback)

    def on_transition(
        self,
        from_state: SystemState,
        to_state: SystemState,
        callback: _Callback,
    ) -> None:
        """Register a callback for a specific (from, to) transition."""
        self._callbacks.setdefault((from_state, to_state), []).append(callback)

    # ---- helpers ----

    @staticmethod
    def _safe_call(cb: _Callback, label: str) -> None:
        try:
            cb()
        except Exception:
            log.exception("Callback error in %s", label)

    def __repr__(self) -> str:
        return f"StateMachine(state={self._state.value})"
