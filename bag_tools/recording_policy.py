"""Recorder-only transport and bounded buffering; leave live publishers untouched."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

CACHE_BYTES = 128 * 1024**2  # rosbag2 double buffers: at most 256 MiB.
NS = 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'


def critical_qos():
    depths = {'/tf': 2048, '/nvblox_lio/odom': 4096,
              '/nvblox_lio/odom_lidar': 512, '/nvblox_robot/joint_states': 512,
              '/nvblox_robot/motion_status/raw': 512}
    return {topic: dict(reliability='reliable', durability='volatile',
                        history='keep_last', depth=depth) for topic, depth in depths.items()}


def recorder_env(base, destination):
    """Give this reader a larger SHM receive queue for mixed large/small topics.

    Preserve the installed UDP interface whitelist and discovery configuration.
    Do not enlarge every participant's shared-memory allocation or edit its file.
    """
    env = dict(base)
    source = env.get('FASTDDS_DEFAULT_PROFILES_FILE') or env.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    if not source:
        raise RuntimeError('录制需要现有 Fast DDS 网络配置，未找到配置路径')
    tree = ET.parse(source); root = tree.getroot(); q = lambda name: '{'+NS+'}'+name
    transports = root.find(q('transport_descriptors'))
    if transports is None:
        raise RuntimeError('录制网络配置缺少 transport_descriptors')
    shm = 0
    for descriptor in list(transports):
        if descriptor.findtext(q('type')) == 'SHM':
            shm += 1
            for name,value in [('segment_size',67108864),('maxMessageSize',8388608),
                               ('port_queue_capacity',2048)]:
                element=descriptor.find(q(name))
                if element is None:element=ET.SubElement(descriptor,q(name))
                element.text=str(max(int(element.text or 0),value))
    if not shm:
        raise RuntimeError('录制网络配置没有 SHM 传输')
    ET.register_namespace('',NS)
    destination = Path(destination); tree.write(destination, encoding='UTF-8', xml_declaration=True)
    env.pop('FASTDDS_DEFAULT_PROFILES_FILE', None)
    env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(destination)
    return env


def cache_losses(path):
    try: text = Path(path).read_text(errors='replace')
    except OSError: return 0
    return max((int(n) for n in re.findall(r'Total lost:\s*(\d+)',text)),default=0)
