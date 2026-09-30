"""Reproduce the complete-input stair baseline selected on 2026-09-28."""
DEFAULT_SENSOR_MODE = 'lidar'
STAIRS_PROFILE = 'stairs_20260928'
SOURCE_EXPERIMENT = '20260928_172204_lidar_5e9e7'
STAIRS_DEFAULTS = {'max_step': .20, 'max_slope_deg': 40.}
STAIRS_CHECKER = dict(resolution=.1, vertical_resolution=.01, tile_cells=8,
                      surface_normal_filter=True, stair_riser_filter=True)


def initial_settings(recorded, explicit, profile):
    if profile not in ('recorded', STAIRS_PROFILE):
        raise ValueError('Unknown replay terrain profile: ' + str(profile))
    settings = dict(recorded)
    if profile == STAIRS_PROFILE:
        settings.update(STAIRS_DEFAULTS)
    if explicit is not None:
        settings.update(explicit)
    return settings
