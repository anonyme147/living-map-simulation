"""Read-only dashboard alert state used only by comparison baseline demos."""

from __future__ import annotations

import threading


class BaselineAlertState:
    """Expose one deliberate, persistent judge-facing baseline failure notice."""

    def __init__(self, *, title: str, detail: str) -> None:
        self._lock = threading.Lock()
        self._title = title
        self._detail = detail
        self._active = False

    def trigger(self) -> None:
        with self._lock:
            self._active = True

    def status(self) -> dict:
        with self._lock:
            return {"enabled": True, "active": self._active, "title": self._title, "detail": self._detail}
