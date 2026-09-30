"""Read-only live input diagnostics. Bounded JSONL logs; never restarts hardware."""
from collections import deque
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import struct
import subprocess
import threading
import time
import uuid


class JsonLog:
    def __init__(self, path, limit=8*1024**2, backups=3):
        self.path, self.limit, self.backups = Path(path), limit, backups

    def append(self, row):
        data = json.dumps(row, ensure_ascii=False, separators=(',', ':'))+'\n'
        if self.path.exists() and self.path.stat().st_size + len(data.encode()) > self.limit:
            for i in range(self.backups, 0, -1):
                source = self.path if i == 1 else Path(str(self.path)+f'.{i-1}')
                if source.exists():
                    source.replace(Path(str(self.path)+f'.{i}'))
        with self.path.open('a') as stream:
            stream.write(data)


def atomic_json(path, row):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(row, ensure_ascii=False, indent=2))
    tmp.replace(path)


def process_start(pid):
    try:
        return Path(f'/proc/{int(pid)}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, ValueError, IndexError):
        return None


def bound_bag(root):
    try:
        doc = json.loads((root/'bag_tools/runtime/diagnostic_target.json').read_text())
        path = Path(doc['path'])
        if path.is_symlink() or path.resolve().parent != (root/'bags').resolve() or path.name.startswith('.'):
            return None
        if not doc.get('process_start') or process_start(doc['pid']) != doc['process_start']:
            return None
        if json.loads((path/'session.json').read_text()).get('state') != 'recording':
            return None
        return path
    except (OSError, ValueError, KeyError, TypeError):
        return None


def header_stamp(raw):
    if len(raw) < 12:
        raise ValueError('short CDR header')
    sec, nano = struct.unpack_from('<iI' if raw[1] & 1 else '>iI', raw, 4)
    return sec+nano/1e9


def lidar_ring_counts(msg):
    """Count M20 sensor groups independently; a merged topic can hide one loss.

    No geometry changes. The mapping is verified against the vendor merge's
    +96 ring offset and raw scan-cone geometry for this robot.
    """
    import numpy as np
    if msg.header.frame_id not in ('lidar_link', 'base_link_dog'):
        raise ValueError('unverified LiDAR frame for ring diagnostics')
    ring = next((f for f in msg.fields if f.name == 'ring'), None)
    if ring is None or ring.datatype != 4 or ring.count != 1 or ring.offset+2 > msg.point_step:
        raise ValueError('invalid UINT16 ring field')
    row_bytes = msg.width*msg.point_step
    required = (msg.height-1)*msg.row_step+row_bytes if msg.height else 0
    if row_bytes > msg.row_step or required > len(msg.data):
        raise ValueError('invalid point cloud dimensions')
    if not msg.height or not msg.width:
        return dict(front=0, rear=0, unknown=0)
    rings = np.ndarray((msg.height,msg.width), dtype='>u2' if msg.is_bigendian else '<u2',
        buffer=msg.data, offset=ring.offset, strides=(msg.row_step,msg.point_step))
    return dict(front=int(np.count_nonzero(rings < 96)),
        rear=int(np.count_nonzero((rings >= 96) & (rings < 192))),
        unknown=int(np.count_nonzero(rings >= 192)))


class Stream:
    def __init__(self):
        self.count = self.previous_count = 0
        self.last_receive = self.last_stamp = None
        self.max_gap = 0.
        self.rollbacks = self.duplicates = self.decode_errors = 0

    def receive(self, stamp, now):
        self.count += 1
        if self.last_receive is not None:
            self.max_gap = max(self.max_gap, now-self.last_receive)
        if self.last_stamp is not None:
            self.rollbacks += stamp < self.last_stamp
            self.duplicates += stamp == self.last_stamp
        self.last_receive, self.last_stamp = now, stamp

    def sample(self, now, wall, interval):
        delta = self.count-self.previous_count
        self.previous_count = self.count
        return dict(count=self.count, received=delta, hz=round(delta/max(interval,.001),2),
            receive_age_s=round(now-self.last_receive,3) if self.last_receive is not None else None,
            header_age_s=round(wall-self.last_stamp,3) if self.last_stamp is not None else None,
            stamp=self.last_stamp, max_receive_gap_s=round(self.max_gap,3),
            timestamp_rollbacks=self.rollbacks, duplicate_stamps=self.duplicates, decode_errors=self.decode_errors)


def read_text(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def network_counters():
    lines = Path('/proc/net/snmp').read_text().splitlines()
    values = {}
    for a,b in zip(lines[::2],lines[1::2]):
        names, numbers = a.split(), b.split()
        if names[0] in ('Ip:', 'Udp:'):
            for name, value in zip(names[1:], numbers[1:]):
                if name.startswith('Reasm') or name in ('InErrors','RcvbufErrors','InDiscards'):
                    values[names[0][:-1]+name] = int(value)
    for name in ('rx_bytes','rx_packets','rx_dropped','rx_errors','tx_bytes'):
        v = read_text('/sys/class/net/eno1/statistics/'+name)
        if v is not None:
            values['eno1_'+name] = int(v)
    return values


def usb_state():
    result = []
    for path in Path('/sys/bus/usb/devices').glob('*'):
        if read_text(path/'idVendor') == '8086' and 'RealSense' in (read_text(path/'product') or ''):
            result.append(dict(port=path.name, **{key:read_text(path/key) for key in
                ('product','idProduct','devnum','speed','authorized','power/control',
                 'power/runtime_status','power/autosuspend_delay_ms')}))
    return result


def classify(streams, services, paused, network_delta, usb_event=False, recording=False):
    def stale(key):
        age = streams.get(key, {}).get('receive_age_s')
        return age is None or age > 2.
    problems = []
    if stale('lidar'):
        problems.append('雷达入口无新消息')
        if network_delta.get('IpReasmFails',0) > 0:
            problems.append('同时发生 IP 分片重组失败，需检查网络计数')
    else:
        for key,label in (('lidar_front','前雷达'),('lidar_rear','后雷达')):
            if key in streams and stale(key):
                problems.append(label+'分组超过 2 秒没有点，总点云仍在更新')
    lio = services.get('nvblox-lio.service',{}).get('ActiveState') == 'active'
    mapping = services.get('nvblox-d435i-shadow.service',{}).get('ActiveState') == 'active'
    if lio:
        if stale('imu'):problems.append('LIO IMU 无新消息')
        if stale('odom'):problems.append('SuperLIO 位姿无新消息')
        if not stale('odom') and stale('pose_tf'):problems.append('位姿 TF 未更新')
    if mapping and (not paused or recording):
        if stale('depth'):
            problems.append('D435i 深度无新消息'+('（同期有 USB/相机错误）' if usb_event else ''))
        elif not recording and not stale('odom') and stale('mesh'):
            problems.append('深度和位姿正常，但 Mesh 未更新')
    mode = '回放中，实时深度建图已暂停' if paused else '实时建图运行中' if mapping else '实时建图服务未运行'
    if recording:mode = '传感器录制模式，Mesh 和通行计算已关闭'
    rates = ' / '.join(f'{label} {streams.get(key,{}).get("hz",0):g} Hz' for key,label in
                        [('lidar','雷达'),('imu','IMU'),('odom','位姿'),('depth','深度'),('mesh','Mesh')])
    return dict(mode=mode, problems=problems, summary=mode+'；'+rates+('；'+'；'.join(problems) if problems else ''))


class Recorder:
    def __init__(self, root):
        self.root = root
        self.output = root/'diagnostics/live'
        self.output.mkdir(parents=True, exist_ok=True)
        self.preroll = deque(maxlen=20)
        self.last_bag = None
        self.session = uuid.uuid4().hex

    def emit(self, kind, data):
        row = dict(time_unix=time.time(), monotonic_s=time.monotonic(),
                   local_time=datetime.now().astimezone().isoformat(timespec='milliseconds'),
                   diagnostic_session=self.session, **data)
        JsonLog(self.output/(kind+'.jsonl')).append(row)
        bag = bound_bag(self.root)
        if bag:
            folder = bag/'diagnostics'
            # Never recreate a bag that has already been moved/deleted.
            try:
                folder.mkdir(exist_ok=True)
                if bag != self.last_bag:
                    for previous in self.preroll:
                        JsonLog(folder/'pre_record.jsonl').append(previous)
                JsonLog(folder/(kind+'.jsonl')).append(row)
            except FileNotFoundError:
                pass
        self.last_bag = bag
        if kind == 'samples':
            self.preroll.append(row)
            atomic_json(self.output/'latest.json', row)
        return row


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, Image, Imu
    from nav_msgs.msg import Odometry
    from nvblox_msgs.msg import Mesh
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from rclpy.serialization import deserialize_message

    root = Path(__file__).resolve().parent.parent
    recorder = Recorder(root)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('se2_input_diagnostics')
    streams = {k:Stream() for k in ('lidar','imu','odom','odom_lidar','depth','mesh','pose_tf',
                                  'lidar_front','lidar_rear')}
    lidar_components = dict(front=0,rear=0,unknown=0,frames_without_front=0,
        frames_without_rear=0,unknown_ring_frames=0,decode_errors=0)
    topics = [('lidar','/LIDAR/POINTS_NX',PointCloud2), ('imu','/IMU',Imu),
              ('odom','/nvblox_lio/odom',Odometry), ('odom_lidar','/nvblox_lio/odom_lidar',Odometry),
              ('depth','/camera/d435i/depth/image_rect_raw',Image), ('mesh','/nvblox_node/mesh',Mesh)]
    def raw_received(key, raw):
        try:
            stamp,now=header_stamp(raw),time.monotonic()
            streams[key].receive(stamp,now)
        except (ValueError,struct.error):streams[key].decode_errors+=1
        else:
            if key == 'lidar':
                try:
                    counts=lidar_ring_counts(deserialize_message(raw,PointCloud2))
                    lidar_components.update(counts)
                    for sensor in ('front','rear'):
                        if counts[sensor]:streams['lidar_'+sensor].receive(stamp,now)
                        else:lidar_components['frames_without_'+sensor]+=1
                    if counts['unknown']:lidar_components['unknown_ring_frames']+=1
                except (ValueError,TypeError,struct.error,RuntimeError):
                    lidar_components['decode_errors']+=1
    subscriptions = [node.create_subscription(typ, topic, lambda msg,k=key:raw_received(k,msg),
                        qos_profile_sensor_data, raw=True) for key,topic,typ in topics]
    def tf_received(msg):
        for tf in msg.transforms:
            if tf.header.frame_id == 'nvblox_odom' and tf.child_frame_id == 'base_link_dog':
                streams['pose_tf'].receive(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9,time.monotonic())
    subscriptions.append(node.create_subscription(TFMessage,'/tf',tf_received,qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(String,'/nvblox_lio/diagnostics',
        lambda msg:recorder.emit('lio_events',dict(message=msg.data)),qos_profile_sensor_data))

    commands = ['journalctl','--follow','--since','now','--output=json','--no-pager',
        '_TRANSPORT=kernel','+','_SYSTEMD_UNIT=lidar-points-bridge.service','+',
        '_SYSTEMD_USER_UNIT=nvblox-lio.service','+','_SYSTEMD_USER_UNIT=nvblox-d435i-shadow.service',
        '+','_SYSTEMD_USER_UNIT=se2-terrain-check.service']
    journal = subprocess.Popen(commands,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
    pending = queue.Queue(maxsize=2000)
    journal_stats = dict(dropped=0,duplicate_messages_suppressed=0)
    def read_journal():
        recent = {}
        for line in journal.stdout:
            try:
                d=json.loads(line); message=d.get('MESSAGE','')
                if not isinstance(message,str):continue
                now=time.monotonic()
                signature=re.sub(r'\d+(?:\.\d+)?','N',message)
                if now-recent.get(signature,-100) < 1:
                    journal_stats['duplicate_messages_suppressed']+=1
                    continue
                if len(recent)>2048:recent.clear()
                recent[signature]=now
                row=dict(journal_time_us=d.get('__REALTIME_TIMESTAMP'),message=message,
                         unit=d.get('_SYSTEMD_USER_UNIT',d.get('_SYSTEMD_UNIT',d.get('_TRANSPORT'))),pid=d.get('_PID'))
                pending.put_nowait(row)
            except queue.Full:journal_stats['dropped']+=1
            except (ValueError,TypeError):pass
    reader=threading.Thread(target=read_journal,daemon=True);reader.start()
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    previous_net=network_counters();last_sample=time.monotonic();last_slow=-100.
    services={};publishers={};usb=[];previous_problems=None;last_usb=-100.;bridge={}
    recorder.emit('events',dict(event='diagnostics_started',pid=os.getpid(),boot_id=read_text('/proc/sys/kernel/random/boot_id')))
    try:
        while running:
            rclpy.spin_once(node,timeout_sec=.1)
            now=time.monotonic()
            if now-last_sample<1:continue
            for _ in range(500):
                try:row=pending.get_nowait()
                except queue.Empty:break
                recorder.emit('journal',row)
                message=row['message']
                if (re.search(r'usb|uvc|VIDIOC',message,re.I) and
                    re.search(r'error|fail|reset|disconnect',message,re.I)) or 'No such device' in message:
                    last_usb=now
                match=re.search(r'rx=([\d.]+) Hz.*total=(\d+)',message)
                if match:bridge.update(ingress_hz=float(match[1]),ingress_total=int(match[2]),updated_unix=time.time())
                match=re.search(r'ipc_sequence=(\d+) publish_completed=(\d+)',message)
                if match:bridge.update(ipc_sequence=int(match[1]),publish_completed=int(match[2]))
            if now-last_slow>5:
                result=subprocess.run(['systemctl','--user','show','nvblox-lio','nvblox-d435i-shadow','se2-terrain-check',
                    '-p','Id,ActiveState,SubState,MainPID,ExecMainStatus,Result'],capture_output=True,text=True,timeout=3)
                services={}
                for block in result.stdout.strip().split('\n\n'):
                    d=dict(line.split('=',1) for line in block.splitlines() if '=' in line)
                    if 'Id' in d:services[d['Id']]=d
                publishers={topic:[p.node_name for p in node.get_publishers_info_by_topic(topic)] for _,topic,_ in topics}
                usb=usb_state();last_slow=now
            wall=time.time();current_net=network_counters()
            delta={k:max(0,v-previous_net.get(k,v)) for k,v in current_net.items()};previous_net=current_net
            sampled={key:s.sample(now,wall,now-last_sample) for key,s in streams.items()}
            sampled['lidar']['sensor_groups']=dict(lidar_components)
            paused=(root/'bag_tools/runtime/live_mapping_suspended.json').exists()
            recording=(root/'bag_tools/runtime/recording_inputs_active.json').exists()
            diagnosis=classify(sampled,services,paused,delta,now-last_usb<30,recording=recording)
            mem={line.split(':')[0]:int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()
                 if line.split(':')[0] in ('MemAvailable','MemFree','SwapFree')}
            recorder.emit('samples',dict(streams=sampled,network_delta=delta,network_total=current_net,
                fragment_memory=read_text('/proc/net/sockstat'),ipfrag_high_thresh=read_text('/proc/sys/net/ipv4/ipfrag_high_thresh'),
                usb=usb,services=services,publishers=publishers,bridge=bridge,memory_kib=mem,
                disk_free_bytes=shutil.disk_usage(root).free,journal_process_alive=journal.poll() is None,
                journal_stats=journal_stats.copy(),**diagnosis))
            if diagnosis['problems']!=previous_problems:
                recorder.emit('events',dict(event='input_state_changed',**diagnosis))
                previous_problems=diagnosis['problems']
            last_sample=now
    finally:
        recorder.emit('events',dict(event='diagnostics_stopped'))
        journal.terminate()
        try:journal.wait(timeout=3)
        except subprocess.TimeoutExpired:journal.kill();journal.wait()
        reader.join(timeout=1)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
