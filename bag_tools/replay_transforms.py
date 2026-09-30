"""Replay measured poses/joints by acquisition time, never bag arrival time.

Prepares a small sidecar under the replay session. Original bags are read-only.
The player remaps its /tf away, preventing competing transform authorities.
Joint transforms are computed by the standard robot_state_publisher from the
recorded URDF and JointState. No robot receiver or motion commands are started.
"""
import argparse
import bisect
import json
import hashlib
import shutil
import tempfile
import time
from pathlib import Path
import sqlite3

import yaml


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def progress(session, message):
    target=session/'prepare_progress.json'
    temporary=target.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(message=message,updated=time.time()),ensure_ascii=False))
    temporary.replace(target)


def source_key(bag):
    metadata=(bag/'data/metadata.yaml').read_bytes()
    meta=yaml.safe_load(metadata)['rosbag2_bagfile_information']
    files=[]
    for name in meta['relative_file_paths']:
        stat=(bag/'data'/name).stat();files.append((name,stat.st_size,stat.st_mtime_ns))
    return hashlib.sha256(json.dumps(dict(version=1,metadata=hashlib.sha256(metadata).hexdigest(),
                                         files=files),sort_keys=True).encode()).hexdigest()


def cache_files(directory):
    result=json.loads((directory/'timed_transforms.json').read_text())
    names=['timed_transforms.db3','timed_transforms.json']
    if result['model']:names.append('recorded_robot.yaml')
    if not all((directory/name).is_file() and (directory/name).stat().st_size for name in names):
        raise ValueError('incomplete replay TF cache')
    try:
        with sqlite3.connect((directory/'timed_transforms.db3').resolve().as_uri()+'?mode=ro',uri=True) as db:
            db.execute('SELECT stamp,kind,data FROM events LIMIT 1').fetchone()
    except sqlite3.Error as exc:
        raise ValueError('invalid replay TF cache') from exc
    return names


def save_cache(bag, session, key):
    parent=bag/'.cache';parent.mkdir(exist_ok=True)
    target=parent/('timed_tf_'+key)
    if target.exists():return
    temporary=Path(tempfile.mkdtemp(prefix='timed_tf_build_',dir=parent))
    try:
        for name in cache_files(session):shutil.copy2(session/name,temporary/name)
        (temporary/'source.json').write_text(json.dumps(dict(source_key=key)))
        temporary.rename(target)
    finally:
        if temporary.exists():shutil.rmtree(temporary)


def prepare(bag, session):
    began=time.monotonic();key=source_key(bag);cached=bag/'.cache'/('timed_tf_'+key)
    progress(session,'正在检查位姿/关节 TF 缓存…')
    try:
        names=cache_files(cached)
        if json.loads((cached/'source.json').read_text())['source_key']!=key:raise ValueError('stale cache')
    except (OSError,ValueError,KeyError):
        _build(bag,session)
        if source_key(bag)!=key:raise RuntimeError('准备期间 bag 发生变化，请停止录制后重试')
        if cached.exists():shutil.rmtree(cached)
        save_cache(bag,session,key);hit=False
    else:
        for name in names:shutil.copy2(cached/name,session/name)
        hit=True
    (session/'prepare_timing.json').write_text(json.dumps(dict(cache_hit=hit,elapsed_s=time.monotonic()-began,
                                                             source_key=key),indent=2))
    progress(session,'TF 缓存已就绪，正在启动回放节点…')


def _build(bag, session):
    from rclpy.serialization import deserialize_message, serialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    meta = yaml.safe_load((bag/'data/metadata.yaml').read_text())['rosbag2_bagfile_information']
    classes = {'/tf':TFMessage, '/nvblox_lio/odom':Odometry,
               '/nvblox_robot/joint_states':JointState, '/nvblox_robot/robot_description':String}
    transforms=[]; poses=[]; joints=[]; description=None
    for index,name in enumerate(meta['relative_file_paths']):
        progress(session,f'首次准备位姿/关节 TF：读取分片 {index+1}/{len(meta["relative_file_paths"])}，完成后可复用…')
        with sqlite3.connect((bag/'data'/name).resolve().as_uri()+'?mode=ro',uri=True) as db:
            topics={tid:topic for tid,topic in db.execute('SELECT id,name FROM topics') if topic in classes}
            if topics:
                # A single table scan; per-topic ORDER BY timestamp repeatedly
                # walked each large shard. Events are sorted by header below.
                query='SELECT topic_id,data FROM messages WHERE topic_id IN ('+','.join('?' for _ in topics)+')'
                for tid,raw in db.execute(query,tuple(topics)):
                    topic=topics[tid]
                    msg=deserialize_message(raw,classes[topic])
                    if topic=='/tf': transforms.extend(msg.transforms)
                    elif topic.endswith('robot_description'): description=msg.data
                    elif topic.endswith('joint_states'): joints.append(msg)
                    elif msg.header.frame_id=='nvblox_odom' and msg.child_frame_id=='base_link_dog':
                        t=TransformStamped();t.header=msg.header;t.child_frame_id=msg.child_frame_id
                        t.transform.translation.x=msg.pose.pose.position.x
                        t.transform.translation.y=msg.pose.pose.position.y
                        t.transform.translation.z=msg.pose.pose.position.z
                        t.transform.rotation=msg.pose.pose.orientation;poses.append(t)
    progress(session,'正在生成 TF 时间索引并保存缓存…')
    model=bool(description and joints)
    if model:
        config={'/**':{'ros__parameters':{'robot_description':description,
                  'use_sim_time':True,'publish_frequency':1000.,'ignore_timestamp':True}}}
        (session/'recorded_robot.yaml').write_text(yaml.safe_dump(config))
    retained=[t for t in transforms if not (
        (poses and t.header.frame_id=='nvblox_odom' and t.child_frame_id=='base_link_dog') or
        (model and t.child_frame_id.startswith('nvblox_robot/')))]
    events=[(stamp_ns(t.header.stamp),'tf',bytes(serialize_message(TFMessage(transforms=[t]))))
            for t in retained+poses]
    if model:
        events.extend((stamp_ns(m.header.stamp),'joint',bytes(serialize_message(m))) for m in joints)
    events.sort(key=lambda row: row[0])
    cache=session/'timed_transforms.db3'
    with sqlite3.connect(cache) as db:
        db.execute('CREATE TABLE IF NOT EXISTS events (stamp INTEGER,kind TEXT,data BLOB)')
        db.execute('DELETE FROM events')
        db.executemany('INSERT INTO events VALUES (?,?,?)',events)
        db.execute('CREATE INDEX IF NOT EXISTS by_stamp ON events(stamp)')
    gaps={}
    for label,messages in [('body',poses),('joints',joints if model else [])]:
        times=sorted(set(stamp_ns(m.header.stamp) for m in messages))
        gaps[label]=max((b-a for a,b in zip(times,times[1:])),default=0)/1e9
    result=dict(model=model,body_poses=len(poses),joint_states=len(joints),
                original_transforms=len(transforms),retained_transforms=len(retained),
                events=len(events),max_source_gap_s=gaps,
                policy='original message header timestamps; no extrapolation or fabricated measurements')
    (session/'timed_transforms.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


def run(session):
    import rclpy
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import JointState
    with sqlite3.connect((session/'timed_transforms.db3').resolve().as_uri()+'?mode=ro',uri=True) as db:
        events=list(db.execute('SELECT stamp,kind,data FROM events ORDER BY stamp'))
    stamps=[e[0] for e in events]
    rclpy.init();node=rclpy.create_node('se2_bag_timed_transforms')
    tf_pub=node.create_publisher(TFMessage,'/tf',QoSProfile(depth=2048,reliability=ReliabilityPolicy.RELIABLE))
    joint_pub=node.create_publisher(JointState,'/bag_rebuild/joint_states_timed',512)
    index=0;last=None
    def clock(message):
        nonlocal index,last
        now=stamp_ns(message.clock)
        if last is None or now<last or now-last>10**10:
            index=bisect.bisect_left(stamps,now-10**10)
        stop=bisect.bisect_right(stamps,now)
        batch=[]
        for _,kind,data in events[index:stop]:
            if kind=='joint':joint_pub.publish(deserialize_message(data,JointState))
            else:batch.extend(deserialize_message(data,TFMessage).transforms)
        for start in range(0,len(batch),128):tf_pub.publish(TFMessage(transforms=batch[start:start+128]))
        index=stop;last=now
    # rosbag2's /clock writer is BEST_EFFORT in Humble. A default RELIABLE
    # subscriber silently receives no clock, leaving every dynamic TF absent.
    node.create_subscription(Clock,'/clock',clock,
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    (session/'timed_transforms.ready').write_text('ready\n')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['prepare','run'])
    parser.add_argument('path',type=Path);parser.add_argument('session',type=Path,nargs='?')
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.path,args.session)
    else:run(args.path)
