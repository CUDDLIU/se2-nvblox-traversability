"""Run the production classifier on metric corridor fixtures, without ROS."""
from pathlib import Path
import math
import os
import sys

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '3')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'terrain_variants/local_window'))

import numpy as np
from paper_pipeline import Config, Engine


def corridor(width):
    vertices, triangles = [], []

    def quad(points):
        index = len(vertices)
        vertices.extend(points)
        triangles.extend([(index, index + 1, index + 2), (index, index + 2, index + 3)])

    low, high = .05 - width / 2, .05 + width / 2
    quad([(-2, low, 0), (2, low, 0), (2, high, 0), (-2, high, 0)])
    for y in (low, high):
        quad([(-2, y, 0), (2, y, 0), (2, y, 1.2), (-2, y, 1.2)])
    return {(0, 0, 0): (np.asarray(vertices), np.asarray(triangles))}


def main():
    config = Config(tile_cells=8, max_step=.2, max_slope_deg=40.,
                    surface_normal_filter=True, stair_riser_filter=True)
    print(f'Body width: {config.width:.3f} m; lateral margin: {config.side_margin:.2f} m')
    print('Corridor    Center cell    Feasible heading angle (degrees, summed over 360)')
    for width in (.500, .506, .540, .600, .750):
        engine = Engine(config, native_mesh=True)
        engine.current_position = (0., .05, .565)
        engine.update(corridor(width))
        engine.process(10000, defer_polygons=True, priority_position=engine.current_position)
        states = [result.pose_states[i] for result in engine.results.values()
                  for i, span in enumerate(result.spans)
                  if span.ix == 0 and span.iy == 0 and span.z < .2 and int(result.masks[i])]
        angle = sum(float(np.nansum(state[:, 5] - state[:, 4])) for state in states)
        label = 'feasible' if states else 'blocked'
        print(f'{width:.3f} m     {label:10s}     {math.degrees(angle):7.3f}')
    print('A zero-width interval can contain an exact heading; it is not free rotation.')


if __name__ == '__main__':
    main()
