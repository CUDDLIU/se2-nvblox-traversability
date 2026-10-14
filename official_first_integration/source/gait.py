"""Nonblocking reconstruction of AOS VelControl::SwitchGait's transaction.

The target gait is an INPUT, not a guessed terrain classification. NavWorker
loads it from taskInfo_t+0x1c before sendGoal (0x1a1954/0x1a195c). This module
has no transport; robot.py --source-planner supplies the guarded sender. Three sends with
up to 3000 ms confirmation per send come from 0x1adc98..0x1adcc4.
"""
from dataclasses import dataclass
import math

SUPPORTED = frozenset((*range(0x1001,0x1006), *range(0x2001,0x2005),
                       *range(0x3001,0x3005), *range(0x5001,0x5004),
                       *range(0xf001,0xf004)))


@dataclass
class GaitSwitch:
    target: int = 0
    attempts: int = 0
    deadline: float = 0.
    state: str = 'idle'

    def begin(self, target, actual, now, *, lying=False, force=False):
        if not math.isfinite(now):
            raise ValueError('invalid monotonic time')
        self.target, self.attempts, self.deadline = int(target), 0, now
        if lying and not force:
            self.state = 'rejected_lying'
            return None
        # Native checks equality before rejecting an unsupported gait ID.
        if not force and actual == target:
            self.state = 'confirmed'
            return None
        if target not in SUPPORTED:
            self.state = 'rejected_gait'
            return None
        return self._send(now)

    def _send(self, now):
        self.attempts += 1
        self.deadline = now + 3.
        self.state = 'waiting'
        return self.target

    def update(self, actual, now):
        if not math.isfinite(now):
            raise ValueError('invalid monotonic time')
        if self.state != 'waiting':
            return None
        if actual == self.target:
            self.state = 'confirmed'
            return None
        if now >= self.deadline:
            if self.attempts < 3:
                return self._send(now)
            self.state = 'timeout'
        return None

    def cancel(self):
        """Integration addition: cancellation invalidates a pending transaction."""
        self.state = 'cancelled'
