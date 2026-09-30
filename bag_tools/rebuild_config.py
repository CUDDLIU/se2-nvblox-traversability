"""Pure replay configuration; no camera driver, LIO or robot control nodes."""
from pathlib import Path
import yaml
from motion_status import TOPICS as MOTION_TOPICS, MODEL_TOPICS
from replay_profiles import STAIRS_PROFILE, STAIRS_CHECKER, initial_settings

DEPTH = '/camera/d435i/depth/image_rect_raw'
INFO = '/camera/d435i/depth/camera_info'
LIDAR = '/LIDAR/POINTS_NX'
DESKEWED = '/terrain_variants/lidar_deskewed'
MODES = {'depth': '深度相机', 'fusion': '深度相机 + 雷达', 'lidar': '纯雷达'}
SENSOR_TOPICS = ('/tf', '/tf_static', DEPTH, INFO, '/nvblox_lio/odom', '/nvblox_lio/odom_lidar', '/LIDAR/POINTS_NX') + MOTION_TOPICS + MODEL_TOPICS
REFERENCE_TOPICS = ('/nvblox/height_mesh', '/se2_navmesh/local_display', '/se2_terrain/status')
TUNABLE = ('max_slope_deg', 'max_step', 'height', 'top_margin', 'side_margin', 'length', 'width')


def validate_mode(sensor_mode):
    if sensor_mode not in MODES:
        raise ValueError('无效的重建输入方案：' + str(sensor_mode))
    return sensor_mode


def required_inputs(sensor_mode='depth'):
    validate_mode(sensor_mode)
    required = ['/tf', '/tf_static', '/nvblox_lio/odom']
    if sensor_mode != 'lidar':
        required += [DEPTH, INFO]
    if sensor_mode != 'depth':
        required += [LIDAR]
    return required


def write_configs(root, terrain, bag, session, settings, sensor_mode='depth', terrain_profile='recorded'):
    validate_mode(sensor_mode)
    root, terrain, bag, session = map(Path, (root, terrain, bag, session))
    source = bag / 'nvblox_config.yaml'
    if not source.exists():
        source = root / 'config/m20_nvblox_shadow.yaml'
    base = yaml.safe_load(source.read_text())
    params = base['nvblox_node']['ros__parameters']
    params.update(global_frame='nvblox_odom', pose_frame='base_link_dog',
        map_clearing_frame_id='base_link_dog',
        esdf_slice_bounds_visualization_attachment_frame_id='base_link_dog',
        workspace_height_bounds_visualization_attachment_frame_id='base_link_dog',
        use_tf_transforms=True, use_lidar=False, use_depth=True, use_color=False,
        num_cameras=1, use_sim_time=True, input_qos='SENSOR_DATA', voxel_size=.05,
        integrate_depth_rate_hz=15., print_rates_to_console=True,
        # Override old bag snapshots too: upper floors must reach RViz/checker.
        layer_visualization_exclusion_height_m=0.,
        print_statistics_on_console_period_ms=10000)
    mapper = params.setdefault('static_mapper', {})
    mapper.update(projective_integrator_max_integration_distance_m=4., workspace_bounds_type='unbounded')
    if sensor_mode != 'depth':
        # Scoped replay preset, derived from the single-origin comparisons.
        # Motion compensation is performed by the dedicated input node; the
        # similarly named legacy nvblox parameter does not implement it.
        params.update(use_lidar=True, use_depth=sensor_mode == 'fusion', voxel_size=.07,
            use_lidar_motion_compensation=False, integrate_lidar_rate_hz=15.,
            integrate_depth_rate_hz=30., publish_layer_rate_hz=15.,
            lidar_width=1800, lidar_height=192,
            use_non_equal_vertical_fov_lidar_params=True,
            min_angle_below_zero_elevation_rad=1., max_angle_above_zero_elevation_rad=1.2,
            lidar_min_valid_range_m=.30, lidar_max_valid_range_m=18.,
            layer_streamer_bandwidth_limit_mbps=30., layer_visualization_exclusion_radius_m=5.,
            map_clearing_radius_m=0., update_esdf_rate_hz=0., publish_debug_vis_rate_hz=0.,
            decay_tsdf_rate_hz=0., publish_esdf_distance_slice=False,
            print_queue_drops_to_console=True, print_delays_to_console=True)
        mapper.update(lidar_projective_integrator_max_integration_distance_m=4.,
            projective_integrator_truncation_distance_vox=4., mesh_integrator_min_weight=.5)
    nvblox = session / 'nvblox_config.yaml'
    nvblox.write_text(yaml.safe_dump({'nvblox_node': {'ros__parameters': params}}, sort_keys=False))
    source = bag / 'paper_config.yaml'
    if not source.exists():
        source = terrain / 'paper_config.yaml'
    document = yaml.safe_load(source.read_text())
    checker = document['se2_terrain_check']['ros__parameters']
    settings = initial_settings({}, settings, terrain_profile)
    if terrain_profile == STAIRS_PROFILE:
        checker.update(STAIRS_CHECKER)
    checker.update({k: float(v) for k, v in settings.items() if k in TUNABLE})
    checker.update(use_sim_time=True, output_frame='nvblox_odom',
        mesh_topic='/nvblox_node/mesh', odom_topic='/nvblox_lio/odom',
        output_json=str(session / 'latest_navmesh.json'),
        history_path=str(session / 'local_history.json'))
    tuning = session / 'terrain_config.yaml'
    tuning.write_text(yaml.safe_dump(document, sort_keys=False))
    return nvblox, tuning


def playback_topics(available, rebuild, sensor_mode='depth'):
    if not rebuild:
        return list(available), []
    required = required_inputs(sensor_mode)
    missing = [t for t in required if not available.get(t)]
    if missing:
        raise ValueError('此包缺少重建输入：' + ', '.join(missing))
    # Pure LiDAR replay works without a camera recording. Do not spend replay
    # bandwidth on depth images that the selected mapper never consumes.
    excluded = {DEPTH, INFO} if sensor_mode == 'lidar' else set()
    topics = [t for t in SENSOR_TOPICS + REFERENCE_TOPICS if available.get(t) and t not in excluded]
    remaps = [f'{t}:=/bag_reference{t}' for t in REFERENCE_TOPICS if available.get(t)]
    return topics, remaps
