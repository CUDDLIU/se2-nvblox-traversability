"""Short lived ROS RPC in the replay domain; never shares the live GUI context."""
import sys
import time
import json
import rclpy
from rosbag2_interfaces.srv import Pause, Resume
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from rebuild_config import TUNABLE, DEPTH, INFO, LIDAR, DESKEWED, validate_mode


def main():
    rclpy.init()
    node = rclpy.create_node('se2_bag_control_probe')
    try:
        action = sys.argv[1]
        if action == 'live-settings':
            client = node.create_client(GetParameters, '/se2_terrain_check/get_parameters')
            if not client.wait_for_service(timeout_sec=8):
                raise RuntimeError('无法读取实时判据参数，请稍后重试')
            future = client.call_async(GetParameters.Request(names=list(TUNABLE)))
            rclpy.spin_until_future_complete(node, future, timeout_sec=8)
            if not future.done() or future.exception():
                raise RuntimeError('读取实时判据参数超时')
            print(json.dumps({key: value.double_value for key, value in zip(TUNABLE, future.result().values)}))
        elif action == 'configure-live':
            from pathlib import Path
            config = json.loads(Path(sys.argv[2]).read_text()).get('settings', {})
            client = node.create_client(SetParametersAtomically, '/se2_terrain_check/set_parameters_atomically')
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError('新建图的判据节点尚未就绪')
            request = SetParametersAtomically.Request(parameters=[
                Parameter(k, value=float(config[k])).to_parameter_msg() for k in TUNABLE if k in config])
            if request.parameters:
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=10)
                if not future.done() or not future.result().result.successful:
                    raise RuntimeError('未能恢复本次录制的判据参数')
        elif action == 'ensure-empty':
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            others = [n for n in node.get_node_names() if n != node.get_name()]
            if others:
                raise RuntimeError('回放 ROS domain 已被占用：' + ', '.join(others))
        elif action == 'wait-rviz':
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if all(any(s.node_name == 'se2_bag_rviz' for s in
                           node.get_subscriptions_info_by_topic(t)) for t in sys.argv[2:]):
                    return
            raise RuntimeError('等待回放 RViz 订阅超时，请检查回放 RViz 日志')
        elif action == 'wait-replay-model':
            deadline = time.monotonic()+15
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                names=[s.node_name for s in node.get_subscriptions_info_by_topic('/bag_rebuild/joint_states_timed')]
                if 'se2_bag_measured_model' in names:return
            raise RuntimeError('实测关节模型尚未订阅回放关节数据')
        elif action == 'wait-rebuild':
            sensor_mode = validate_mode(sys.argv[2] if len(sys.argv) > 2 else 'depth')
            client = node.create_client(GetParameters, '/se2_terrain_check/get_parameters')
            required = [('/nvblox_node/mesh', 'se2_navmesh_input'),
                        ('/nvblox_lio/odom', 'se2_navmesh_input')]
            if sensor_mode != 'lidar':
                required += [(DEPTH, 'nvblox_node'), (INFO, 'nvblox_node')]
            if sensor_mode != 'depth':
                required += [(LIDAR, 'terrain_lidar_deskew'), (DESKEWED, 'nvblox_node'),
                             ('/nvblox_lio/odom', 'terrain_lidar_deskew')]
            # Reference Mesh is incremental too: its cache must precede playback.
            required.append(('/bag_reference/nvblox/height_mesh', 'se2_bag_reference_cache'))
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if client.service_is_ready() and all(any(s.node_name == name for s in
                        node.get_subscriptions_info_by_topic(topic)) for topic, name in required):
                    return
            found={topic:[s.node_name for s in node.get_subscriptions_info_by_topic(topic)]
                   for topic, _ in required}
            raise RuntimeError('重建节点输入未就绪：'+str(found))
        elif action == 'saved-map':
            data = {}
            node.create_subscription(PointCloud2, '/bag_reference/superlio_map',
                lambda msg: data.update(points=msg.width*msg.height, frame=msg.header.frame_id),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            deadline = time.monotonic()+8
            while not data and time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if not data:
                raise RuntimeError('绑定地图发布者未提供点云')
            print(json.dumps(data))
        elif action == 'snapshot':
            data = {}
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            def status(key):
                return lambda msg: data.update({key: json.loads(msg.data)})
            node.create_subscription(String, '/se2_terrain/status', status('rebuilt'), qos)
            node.create_subscription(String, '/bag_rebuild/reference_status', status('recorded'), qos)
            node.create_subscription(Clock, '/bag_rebuild/clock', lambda msg: data.update(
                clock=msg.clock.sec + msg.clock.nanosec / 1e9), qos)
            client = node.create_client(GetParameters, '/se2_terrain_check/get_parameters')
            if not client.wait_for_service(timeout_sec=8):
                raise RuntimeError('回放判定节点不可用')
            future = client.call_async(GetParameters.Request(names=list(TUNABLE)))
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                mesh_publishers=[p.node_name for p in node.get_publishers_info_by_topic('/nvblox_node/mesh')]
                if (future.done() and 'rebuilt' in data and 'recorded' in data and 'clock' in data
                        and 'nvblox_node' in mesh_publishers):
                    break
            if not future.done() or 'rebuilt' not in data:
                raise RuntimeError('没有可保存的重建状态')
            data['parameters'] = {key: value.double_value for key, value in zip(TUNABLE, future.result().values)}
            data['settled'] = (data['rebuilt'].get('pending_slabs', 1) == 0 and
                data['rebuilt'].get('inbox_blocks', 1) == 0 and all(
                    data['rebuilt'].get('config', {}).get(k) == v for k,v in data['parameters'].items()))
            data['source_publishers'] = {topic:[p.node_name for p in node.get_publishers_info_by_topic(topic)]
                for topic in ('/nvblox_node/mesh','/se2_terrain/status','/bag_reference/nvblox/height_mesh')}
            print(json.dumps(data, ensure_ascii=False))
        else:
            srv = {'pause': Pause, 'resume': Resume}[action]
            client = node.create_client(srv, '/rosbag2_player/' + action)
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('回放控制服务未就绪')
            future = client.call_async(srv.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done() or future.exception():
                raise RuntimeError('回放控制请求超时或失败')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
