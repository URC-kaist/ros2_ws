"""Thread-safe-independent state rules for one rover uplink trial."""

from __future__ import annotations

import re
from dataclasses import dataclass, field


TRIAL_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
ACTIVE_PHASES = {"prepared", "capturing", "uploading", "restoring_streams"}
TERMINAL_PHASES = {"completed", "failed", "cancelled"}


@dataclass
class UplinkTrial:
    trial_id: str
    stream_ids: list[str]
    phase: str = "prepared"
    progress: float = 0.0
    error_code: str = ""
    message: str = "Prepared"
    cancel_requested: bool = False
    metadata: dict[str, object] = field(default_factory=dict)
    lease_acquired: bool = False

    def transition(self, phase: str, message: str, progress: float) -> None:
        if self.phase in TERMINAL_PHASES:
            return
        self.phase = phase
        self.message = message
        self.progress = min(1.0, max(0.0, progress))

    def fail(self, code: str, message: str) -> None:
        if self.phase in TERMINAL_PHASES:
            return
        self.phase = "failed"
        self.error_code = code
        self.message = message

    def cancel(self) -> None:
        if self.phase in TERMINAL_PHASES:
            return
        self.cancel_requested = True
        self.phase = "cancelled"
        self.error_code = "trial_cancelled"
        self.message = "Trial cancelled"


def validate_trial_id(value: str) -> str:
    if not TRIAL_ID_PATTERN.fullmatch(value):
        raise ValueError("invalid trial_id")
    return value


def validate_duration(value: float, maximum: float) -> float:
    duration = float(value)
    if duration < 2.0 or duration > maximum:
        raise ValueError(f"duration_s must be between 2 and {maximum:g}")
    return duration
