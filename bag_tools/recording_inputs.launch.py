"""Recording-only camera and measured robot model. No nvblox, Mesh or ESDF."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parent.parent
    share=Path(get_package_share_directory('se2_nvblox_surface_shadow'))
    return LaunchDescription([
        Node(package='realsense2_camera',executable='realsense2_camera_node',
             name='d435i',namespace='camera',output='screen',
             parameters=[str(share/'config/d435i_viewer.yaml')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(root/'robot_model/nvblox_robot.launch.py'))),
    ])
