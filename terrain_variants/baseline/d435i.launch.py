"""D435i depth-only shadow reconstruction with Viewer settings.

pose_mode=odom uses the existing calibrated robot TF and localization.
pose_mode=camera is a stationary-camera inspection mode; it is not odometry.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def create_nodes(context):
    share = get_package_share_directory('se2_nvblox_surface_shadow')
    mode = LaunchConfiguration('pose_mode').perform(context)
    if mode not in ('odom', 'camera', 'stationary'):
        raise ValueError('pose_mode must be odom, camera or stationary')
    local = mode != 'odom'
    frame = ('lidar_link' if mode == 'stationary' else 'd435i_depth_optical_frame') if local else LaunchConfiguration('odom_frame').perform(context)
    pose = frame if local else 'base_link_dog'
    depth = '/camera/d435i/depth/image_rect_raw'
    params = os.path.join(share, 'config', 'm20_nvblox_shadow.yaml')
    nodes = [
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             name='d435i', namespace='camera', output='screen',
             parameters=[os.path.join(share, 'config', 'd435i_viewer.yaml')]),
        Node(package='nvblox_ros', executable='nvblox_node', name='nvblox_node',
             output='screen', parameters=[params, {
                 'global_frame': frame, 'pose_frame': pose,
                 'map_clearing_frame_id': pose,
                 'esdf_slice_bounds_visualization_attachment_frame_id': pose,
                 'workspace_height_bounds_visualization_attachment_frame_id': pose,
                 'use_tf_transforms': True, 'use_lidar': False,
                 'use_depth': True, 'use_color': False, 'num_cameras': 1,
                 'input_qos': 'SENSOR_DATA', 'voxel_size': 0.05,
                 'integrate_depth_rate_hz': 15.0,
                 'static_mapper.projective_integrator_max_integration_distance_m': 4.0,
                 'static_mapper.workspace_bounds_type': 'unbounded',
                 'print_rates_to_console': True,
                 'print_statistics_on_console_period_ms': 10000,
             }], remappings=[
                 ('camera_0/depth/image', depth),
                 ('camera_0/depth/camera_info', '/camera/d435i/depth/camera_info')]),
    ]
    if not local and LaunchConfiguration('start_localization').perform(context).lower() == 'true':
        nodes.insert(0, ExecuteProcess(cmd=['bash', '-lc',
            'source /home/nvidia/scanplanner_test/M-detector/install/setup.bash; '
            'source /home/nvidia/scanplanner_test/localization.sh'], output='screen'))
    if not local and LaunchConfiguration('use_legacy_surface_adapter').perform(context).lower() == 'true':
        nodes.extend([
            Node(package='se2_nvblox_surface_shadow', executable='surface_adapter',
                 name='se2_nvblox_surface_adapter', output='screen',
                 parameters=[params, {'output_frame': frame, 'depth_topic': depth,
                                      'odom_topic': LaunchConfiguration('odom_topic').perform(context),
                                      'window_radius_m': 4.0}]),
        ])
    nodes.append(ExecuteProcess(cmd=['/usr/bin/python3',
        os.path.join(share, 'scripts', 'mesh_height.py')], output='screen'))
    if LaunchConfiguration('use_rviz').perform(context).lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='se2_shadow_rviz',
                          arguments=['-d', os.path.join(share, 'rviz', 'd435i_mesh.rviz'),
                                     '-f', frame], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pose_mode', default_value='odom'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('odom_topic', default_value='/lio/odom'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_localization', default_value='false'),
        DeclareLaunchArgument('use_legacy_surface_adapter', default_value='true'),
        OpaqueFunction(function=create_nodes),
    ])
