"""Independent nvblox robot description and measured leg TF. No control outputs."""
from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def robot_description():
    root=Path(__file__).resolve().parent
    model=ET.parse(root/'upstream/M20/urdf/M20.urdf').getroot()
    def frame(name):return 'base_link_dog' if name=='base_link' else 'nvblox_robot/'+name
    for link in model.findall('link'):link.set('name',frame(link.get('name')))
    for joint in model.findall('joint'):
        joint.set('name','nvblox_robot/'+joint.get('name'))
        for tag in ('parent','child'):
            item=joint.find(tag);item.set('link',frame(item.get('link')))
    for mesh in model.findall('.//mesh'):
        mesh.set('filename',(root/'upstream/M20/urdf'/mesh.get('filename')).resolve().as_uri())
    # CAD-only MuJoCo tags are not needed by robot_state_publisher.
    for tag in ('mujoco','gazebo'):
        for item in model.findall(tag):model.remove(item)
    return ET.tostring(model,encoding='unicode')


def generate_launch_description():
    root=Path(__file__).resolve().parent
    return LaunchDescription([
        ExecuteProcess(cmd=['/usr/bin/python3',str(root.parent/'bag_tools/motion_status.py'),
            str(root/'runtime'),'--ros','--latest-only'],output='screen'),
        ExecuteProcess(cmd=['/usr/bin/python3',str(root/'joint_bridge.py')],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
            namespace='nvblox_robot',name='state_publisher',output='screen',
            parameters=[{'robot_description':robot_description(),'publish_frequency':30.}],
            remappings=[('joint_states','/nvblox_robot/joint_states'),
                        ('robot_description','/nvblox_robot/robot_description')]),
    ])
