"""AOS persistent-warning timer, with Go2Point disabled as captured.

recvPlannerStatus 0x199f20 and CheckContinuousWarningStatus 0x19b920.
All warning classes share the last healthy timestamp. A change from one
warning to another must not restart the timeout. A new goal resets it.
"""
from dataclasses import dataclass

ERRORS={1001:0xa406,1002:0xa407,1003:0xa405,
        1101:0xa34c,1102:0xa34c,2004:0xa34b,2005:0xa34b}


@dataclass
class WarningTimer:
    last_good_ms: int
    obstruction_seconds: int = 30
    localization_seconds: int = 30
    obstacle_wait: bool = False
    error: int = 0

    def update(self,now_ms,warning,*,localized=True,command_fresh=True):
        if self.error:return self.error  # Integration latches task failure.
        if warning in (2004,2005):self.obstacle_wait=True
        elif warning==0:self.obstacle_wait=False
        elapsed=now_ms-self.last_good_ms
        if self.obstacle_wait:
            if elapsed>1000*self.obstruction_seconds:self.error=0xa34b
        elif warning in (1002,1101,1102,2004,2005):
            if elapsed>30000:self.error=ERRORS.get(warning,0xa34c)
        elif not localized:
            if elapsed>1000*self.localization_seconds:self.error=0xa313
        elif not command_fresh:
            if elapsed>30000:self.error=0xa34d
        else:self.last_good_ms=now_ms
        return self.error
