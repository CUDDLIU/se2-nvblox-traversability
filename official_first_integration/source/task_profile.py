"""AOS saved-task parameter selection, separate from live terrain perception.

SetPlannerParam 0x19f0e0 and init 0x19c350/0x19c5d0. The captured binary
actually selects FLAT for non-stair gait IDs, even though ORIENTEER is loaded.
Strict input enums are integration validation (native indexes unchecked).
"""
import json
import math
from pathlib import Path

CONFIG = json.loads(Path(__file__).with_name('navigation_config.json').read_text())
DEFAULT = dict(maxSpeedX=1.5, maxSpeedY=.6, maxTheta=1.,
               xy_tolerance=.1, yaw_tolerance=.1, barking_deceleration=1.,
               stop_distance=.7, dl_min_x=.25, dl_min_y=.25,
               dl_min_yaw=.7, dl_line_yaw_min=.2,
               directLine_mode=False, judge_close=False, backward_mode=False)
FIELDS = tuple(DEFAULT)


def task_parameters(task):
    values = {}
    for key, choices in [('Gait', (0x3002, 0x3003)), ('NavMode', (0, 1)),
                         ('Speed', (0, 1, 2)), ('Manner', (0, 1)), ('ObsMode', (0, 1))]:
        v = task.get(key)
        if type(v) is not int or v not in choices:
            raise ValueError('invalid or missing task ' + key)
        values[key] = v
    mode = 'AUTO' if values['NavMode'] == 1 else 'DIRECTLINE'
    terrain = 'STAIR' if values['Gait'] == 0x3003 else 'FLAT'
    speed = ('NORMAL_SPEED', 'LOW_SPEED', 'HIGH_SPEED')[values['Speed']]
    return dict(CONFIG['COMMON'], **CONFIG[mode][terrain][speed],
                directLine_mode=mode == 'DIRECTLINE',
                judge_close=values['ObsMode'] != 0,
                backward_mode=values['Manner'] == 1)


def validate_parameters(p):
    if set(p) != set(FIELDS):
        raise ValueError('incomplete or unknown planner parameters')
    for name in FIELDS:
        if name in ('directLine_mode', 'judge_close', 'backward_mode'):
            if type(p[name]) is not bool:raise ValueError(name)
        elif type(p[name]) not in (int, float) or not math.isfinite(p[name]) or p[name] <= 0:
            raise ValueError(name)
    if p['maxSpeedX'] > 2 or p['maxSpeedY'] > 1 or p['maxTheta'] > 1:
        raise ValueError('planner limits exceed captured task configuration')
    return dict(p)
