from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def lidar_launch_description():
    default_config = os.path.join(
        get_package_share_directory('se2_nvblox_surface_shadow'), 'config',
        'm20_nvblox_shadow.yaml')
    shm_config = os.path.join(
        get_package_share_directory('se2_nvblox_surface_shadow'), 'config',
        'fastdds_shm_only.xml')
    config = LaunchConfiguration('config')
    cloud_topic = LaunchConfiguration('cloud_topic')
    global_frame = LaunchConfiguration('global_frame')
    pose_frame = LaunchConfiguration('pose_frame')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('cloud_topic', default_value='/LIDAR/POINTS_NX'),
        DeclareLaunchArgument('global_frame', default_value='odom'),
        DeclareLaunchArgument('pose_frame', default_value='base_link_dog'),
        DeclareLaunchArgument('crop_z_min_m', default_value='-0.587077'),
        DeclareLaunchArgument('crop_z_max_m', default_value='0.312923'),
        DeclareLaunchArgument('crop_max_xy_range_m', default_value='2.0'),
        DeclareLaunchArgument('crop_front_only', default_value='true'),
        DeclareLaunchArgument('crop_front_x_min_m', default_value='0.0'),
        Node(package='se2_nvblox_surface_shadow', executable='crop_cloud',
             name='se2_nvblox_cloud_crop', output='screen', parameters=[{
                 'z_min_m': LaunchConfiguration('crop_z_min_m'),
                 'z_max_m': LaunchConfiguration('crop_z_max_m'),
                 'max_xy_range_m': LaunchConfiguration('crop_max_xy_range_m'),
                 'front_only': LaunchConfiguration('crop_front_only'),
                 'front_x_min_m': LaunchConfiguration('crop_front_x_min_m')}]),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_localization', default_value='false'),
        ExecuteProcess(
            cmd=['bash', '-lc',
                 'source /home/nvidia/scanplanner_test/M-detector/install/setup.bash; '
                 'source /home/nvidia/scanplanner_test/localization.sh'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_localization')),
        ),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            get_package_share_directory('se2_nvblox_surface_shadow'),
            'rviz', 'surface_shadow.rviz')),
        Node(package='rviz2', executable='rviz2', name='se2_shadow_rviz',
             arguments=['-d', LaunchConfiguration('rviz_config'), '-f', global_frame],
             condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen'),
        Node(
            package='nvblox_ros',
            executable='nvblox_node',
            name='nvblox_node',
            output='screen',
            parameters=[config, {
                'global_frame': global_frame,
                'pose_frame': pose_frame,
                # The stock nvblox 3.2.5 defaults these helper frames to
                # ``base_link``. M20 publishes ``base_link_dog``; leaving the
                # defaults causes map clearing and point-cloud integration to
                # fail even though pose_frame itself is overridden.
                'map_clearing_frame_id': pose_frame,
                'esdf_slice_bounds_visualization_attachment_frame_id': pose_frame,
                'workspace_height_bounds_visualization_attachment_frame_id': pose_frame,
                'use_tf_transforms': True,
                # 3.2.5 cannot consume the AIRY uint64 absolute timestamp
                # field for motion compensation yet; validate geometry first.
                'use_lidar_motion_compensation': False,
                # nvblox 3.2.5's camera model is verified below; dimensions
                # are intentionally overridden during the compatibility probe.
                'lidar_width': 1800,
                'lidar_height': 192,
            }],
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                # This profile contains both UDPv4 (cross-host input) and SHM
                # (same-host shadow topics).  SHM-only breaks discovery/TF.
                'FASTRTPS_DEFAULT_PROFILES_FILE': '/home/nvidia/.config/fastdds/eno1.xml',
                'FASTDDS_DEFAULT_PROFILES_FILE': '/home/nvidia/.config/fastdds/eno1.xml',
            },
            remappings=[('pointcloud', '/se2_nvblox/lidar_points_cropped'), ('pose', '/se2_nvblox/pose')],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='se2_nvblox_base_to_lidar_identity',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link_dog', 'lidar_link'],
            output='screen',
        ),
        Node(
            package='se2_nvblox_surface_shadow',
            executable='odom_pose_adapter',
            name='se2_nvblox_odom_pose_adapter',
            output='screen',
            parameters=[{'odom_topic': '/lio/odom', 'pose_topic': '/se2_nvblox/pose',
                         'frame_id': global_frame}],
        ),
        Node(
            package='se2_nvblox_surface_shadow',
            executable='surface_adapter',
            name='se2_nvblox_surface_adapter',
            output='screen',
            # The raw 110k-point cloud is audited by cloud_audit.  The adapter
            # only needs a lightweight liveness count, so subscribe to the
            # cropped nvblox input to avoid a second full DDS deserialization.
            parameters=[{'cloud_topic': '/se2_nvblox/lidar_points_cropped',
                         'output_frame': global_frame}, config],
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'FASTRTPS_DEFAULT_PROFILES_FILE': '/home/nvidia/.config/fastdds/eno1.xml',
                'FASTDDS_DEFAULT_PROFILES_FILE': '/home/nvidia/.config/fastdds/eno1.xml',
            },
        ),
        Node(
            package='se2_nvblox_surface_shadow',
            executable='cloud_audit',
            name='se2_cloud_audit',
            output='screen',
            parameters=[{'input_topic': cloud_topic}],
        ),
    ])


def select_input(context):
    mode = LaunchConfiguration('input_mode').perform(context)
    if mode == 'lidar':
        return list(lidar_launch_description().entities)
    if mode != 'd435i':
        raise ValueError('input_mode must be d435i or lidar')
    share = get_package_share_directory('se2_nvblox_surface_shadow')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share, 'launch', 'd435i.launch.py')))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_mode', default_value='d435i'),
        OpaqueFunction(function=select_input),
    ])
