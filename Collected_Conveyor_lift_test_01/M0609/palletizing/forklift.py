"""Pallet/forklift policies whose timing differs between A and B."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional

import numpy as np


@dataclass
class PalletLoweringCoordinator:
    """Run B's pallet lowering alongside home return without stepping the World."""

    enabled: bool
    use_home_steps: bool
    pallet_path: str
    get_translation: Callable[[Any, str], Any]
    get_center: Callable[[Any], Any]
    set_translation: Callable[[Any, str, Any], Any]
    log_boxes: Callable[..., Any]
    lower_steps: int
    home_steps: int
    extra_z: float
    log_interval: int
    active: bool = False
    step: int = 0
    steps: int = 0
    start: Optional[np.ndarray] = None
    target: Optional[np.ndarray] = None
    baseline: Optional[Mapping[str, np.ndarray]] = None
    label: str = "idle"

    def reset(self) -> None:
        self.active = False
        self.step = 0
        self.steps = 0
        self.start = None
        self.target = None
        self.baseline = None
        self.label = "idle"

    def schedule(self, stage, box_height: float, baseline: Mapping[str, np.ndarray], label: str) -> bool:
        if not self.enabled:
            return False
        current = self.get_translation(stage, self.pallet_path)
        if current is None:
            current = self.get_center(stage)
        if current is None:
            print(f"[PALLET_LOWER_PARALLEL][SKIP] cannot read {self.pallet_path}")
            return False
        self.start = np.array(current, dtype=float)
        self.target = self.start.copy()
        self.target[2] -= float(max(0.01, box_height)) + float(self.extra_z)
        self.steps = max(1, int(self.home_steps if self.use_home_steps else self.lower_steps))
        self.step = 0
        self.baseline = baseline
        self.label = label
        self.active = True
        print(
            f"[PALLET_LOWER_PARALLEL][START] label={label}, path={self.pallet_path}, "
            f"steps={self.steps}, start={self.start}, target={self.target}"
        )
        self.log_boxes(stage, "parallel_start", baseline=baseline, force=True)
        return True

    def update(self, stage) -> bool:
        if not self.active or self.start is None or self.target is None:
            return False
        self.step = min(self.steps, self.step + 1)
        alpha = float(self.step) / float(max(1, self.steps))
        smooth = alpha * alpha * (3.0 - 2.0 * alpha)
        current = self.start * (1.0 - smooth) + self.target * smooth
        self.set_translation(stage, self.pallet_path, current)
        if self.step == 1 or self.step >= self.steps or self.step % max(1, self.log_interval) == 0:
            self.log_boxes(
                stage,
                f"parallel_home_lower={self.step}/{self.steps}",
                baseline=self.baseline,
                force=True,
            )
        if self.step >= self.steps:
            self.set_translation(stage, self.pallet_path, self.target)
            self.log_boxes(stage, "parallel_done", baseline=self.baseline, force=True)
            print(f"[PALLET_LOWER_PARALLEL][DONE] label={self.label}, final={self.target}")
            self.active = False
        return True

