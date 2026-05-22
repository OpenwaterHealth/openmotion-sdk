"""RollingAverageStage — sliding-window mean of bfi_live, bvi_live.

State carried across batches: a ring of the most recent N samples per camera.
reset() clears the ring.
"""

from __future__ import annotations

from collections import deque
from typing import Deque

import numpy as np

from ..batch import FrameBatch


class RollingAverageStage:
    name = "rolling_average"

    def __init__(self, window: int):
        if window < 1:
            raise ValueError(f"window must be >= 1, got {window}")
        self.window = int(window)
        self._bfi_window: Deque[np.ndarray] = deque(maxlen=self.window)
        self._bvi_window: Deque[np.ndarray] = deque(maxlen=self.window)

    def process(self, batch: FrameBatch) -> FrameBatch:
        n = batch.bfi_live.shape[0]
        bfi_rolling = np.empty_like(batch.bfi_live)
        bvi_rolling = np.empty_like(batch.bvi_live)

        for i in range(n):
            self._bfi_window.append(batch.bfi_live[i])
            self._bvi_window.append(batch.bvi_live[i])
            bfi_rolling[i] = np.mean(np.stack(self._bfi_window, axis=0), axis=0)
            bvi_rolling[i] = np.mean(np.stack(self._bvi_window, axis=0), axis=0)

        batch.bfi_rolling = bfi_rolling
        batch.bvi_rolling = bvi_rolling
        return batch

    def reset(self) -> None:
        self._bfi_window.clear()
        self._bvi_window.clear()
