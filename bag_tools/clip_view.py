"""Frame an extracted clip in RViz without changing its recorded coordinates."""
import csv
import json
import math


def center_clip_view(visualization, bag):
    saved = bag / 'rviz_view.json'
    if saved.is_file():
        view = json.loads(saved.read_text())
        visualization.setdefault('Views', {})['Current'] = view
        return
    try:
        with (bag / 'maps/trajectory.csv').open() as stream:
            points = [tuple(float(row[k]) for k in ('x', 'y', 'z'))
                      for row in csv.DictReader(stream)]
    except (OSError, KeyError, ValueError):
        return
    points = [p for p in points if all(math.isfinite(v) for v in p)]
    if not points:
        return
    bounds = [(min(axis), max(axis)) for axis in zip(*points)]
    view = visualization.setdefault('Views', {}).setdefault('Current', {})
    view.update({
        'Class': 'rviz_default_plugins/Orbit',
        'Target Frame': '<Fixed Frame>',
        'Focal Point': {k: (lo + hi) / 2 for k, (lo, hi) in zip('XYZ', bounds)},
        'Distance': max(12., 1.4 * math.sqrt(sum((hi - lo) ** 2 for lo, hi in bounds))),
        'Pitch': .55, 'Yaw': 3.14,
    })
