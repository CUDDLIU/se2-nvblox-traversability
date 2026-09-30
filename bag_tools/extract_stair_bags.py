"""Create independent read-only-source stair clips with provenance and TF bootstrap.

Original sensor CDR and receive timestamps are copied exactly. Only latched
bootstrap receive times move to the clip start; original header stamps remain.
"""
import argparse
from collections import Counter,defaultdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import struct
import time
import uuid
import yaml

WARM_TOPICS={'/tf','/nvblox_lio/odom','/nvblox_lio/odom_lidar',
             '/nvblox_robot/joint_states','/nvblox_robot/motion_status/joints'}
LATCHED={'/tf_static','/nvblox_robot/robot_description'}


def readonly(path):return sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)
def digest_update(digests,topic,stamp,raw):
    digests[topic].update(struct.pack('<qQ',stamp,len(raw)));digests[topic].update(raw)
def fingerprints(source):
    return {str(p.relative_to(source)):dict(size=p.stat().st_size,mtime_ns=p.stat().st_mtime_ns)
            for p in (source/'data').iterdir() if p.is_file()}

def main():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from record_map import VoxelMap,cloud_xyz
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);args=parser.parse_args()
    source=args.source.resolve();meta=yaml.safe_load((source/'data/metadata.yaml').read_text())['rosbag2_bagfile_information']
    original=json.loads((source/'session.json').read_text());before=fingerprints(source)
    if original.get('state')!='ready':raise RuntimeError('Only finished source bags can be extracted')
    if shutil.disk_usage(source).free<8*1024**3:raise RuntimeError('Need at least 8 GiB free for two independent clips')
    start=meta['starting_time']['nanoseconds_since_epoch'];files=[]
    for name in meta['relative_file_paths']:
        db=readonly(source/'data'/name)
        lo,hi=db.execute('SELECT MIN(timestamp),MAX(timestamp) FROM messages').fetchone()
        topics={row[0]:row[1] for row in db.execute('SELECT id,name FROM topics')}
        files.append((db,lo,hi,topics))
    result=[]
    for label,title,begin_s,end_s in [('stairs_down','踏板楼梯 · 下楼 195–275s',195,275),('stairs_up','踏板楼梯 · 上楼 335–415s',335,415)]:
        begin=start+begin_s*10**9;end=start+end_s*10**9;warm=begin-5*10**9
        key=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+label+'_'+uuid.uuid4().hex[:4]
        target=source.parent/key;tmp=source.parent/('.extract_'+key);tmp.mkdir()
        counts=Counter();hashes=defaultdict(hashlib.sha256);bootstrap=[]
        # Keep original static payloads: four publishers may own different edges.
        # Preserve all pre-window static updates in order so the last value wins.
        last_joint=None;last_description=None
        for db,lo,hi,topics in files:
            if lo>=warm:break
            for tid,topic in topics.items():
                if topic not in LATCHED and topic!='/nvblox_robot/joint_states':continue
                if topic=='/tf_static':
                    for stamp,raw in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp<? ORDER BY timestamp,id',(tid,warm)):
                        bootstrap.append((stamp,topic,raw))
                else:
                    row=db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp<? ORDER BY timestamp DESC,id DESC LIMIT 1',(tid,warm)).fetchone()
                    if row:
                        value=(row[0],topic,row[1])
                        if topic.endswith('robot_description'):last_description=value
                        else:last_joint=value
        if last_description:bootstrap.append(last_description)
        if last_joint:bootstrap.append(last_joint)
        if not any(t=='/tf_static' for _,t,_ in bootstrap):raise RuntimeError('Source static TF unavailable')
        writer=rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(tmp/'data'),storage_id='sqlite3',
                    max_bagfile_size=1024**3,max_cache_size=64*1024**2),
                    rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr'))
        for t in meta['topics_with_message_count']:
            writer.create_topic(rosbag2_py.TopicMetadata(**t['topic_metadata']))
        bootstrap_manifest=[]
        for old_stamp,topic,raw in sorted(bootstrap,key=lambda x:x[0]):
            writer.write(topic,raw,warm);counts[topic]+=1;digest_update(hashes,topic,warm,raw)
            bootstrap_manifest.append(dict(topic=topic,original_receive_ns=old_stamp,new_receive_ns=warm,
                                           payload_sha256=hashlib.sha256(raw).hexdigest()))
        maps=tmp/'maps';maps.mkdir();voxels=VoxelMap(.2);clouds=0;map_frames=set();last_report=time.monotonic()
        pose_file=(maps/'trajectory.csv').open('w');pose_file.write('stamp,x,y,z,qx,qy,qz,qw\n')
        for db,lo,hi,topics in files:
            if hi<warm or lo>=end:continue
            for tid,stamp,raw in db.execute('SELECT topic_id,timestamp,data FROM messages WHERE timestamp>=? AND timestamp<? ORDER BY timestamp,id',(warm,end)):
                topic=topics[tid]
                if stamp<begin and topic not in WARM_TOPICS:continue
                writer.write(topic,raw,stamp);counts[topic]+=1;digest_update(hashes,topic,stamp,raw)
                if topic=='/nvblox_lio/cloud_world':
                    msg=deserialize_message(raw,PointCloud2);map_frames.add(msg.header.frame_id)
                    if msg.header.frame_id!='nvblox_odom':raise RuntimeError('Unexpected world cloud frame')
                    voxels.add(cloud_xyz(msg));clouds+=1
                elif topic=='/nvblox_lio/odom_lidar':
                    m=deserialize_message(raw,Odometry);p=m.pose.pose.position;q=m.pose.pose.orientation
                    pose_file.write(f'{m.header.stamp.sec+m.header.stamp.nanosec/1e9:.9f},{p.x},{p.y},{p.z},{q.x},{q.y},{q.z},{q.w}\n')
                if time.monotonic()-last_report>10:
                    print(json.dumps(dict(clip=label,source_second=round((stamp-start)/1e9,1),messages=sum(counts.values()),map_points=len(voxels.points))),flush=True);last_report=time.monotonic()
        pose_file.close();del writer
        voxels.save(maps/'superlio_map.pcd')
        map_info=dict(frame='nvblox_odom',source='/nvblox_lio/cloud_world',mode='extracted_recorded_world_clouds',
            loaded_old_map=False,voxel_size_m=.2,points=len(voxels.points),counts={'map':clouds},final=True,
            scope='Only world point-cloud messages within this clip; original source coordinates; no new SLAM',
            source_bag=source.name,source_interval_s=[begin_s,end_s])
        (maps/'map.json').write_text(json.dumps(map_info,ensure_ascii=False,indent=2))
        for name in ['nvblox_config.yaml','paper_config.yaml','d435i_viewer.yaml','record_qos.yaml','superlio_base_config.yaml']:
            if (source/name).is_file():shutil.copy2(source/name,tmp/name)
        actual=defaultdict(hashlib.sha256);actual_counts=Counter();checks=[]
        output_meta=yaml.safe_load((tmp/'data/metadata.yaml').read_text())['rosbag2_bagfile_information']
        for name in output_meta['relative_file_paths']:
            with readonly(tmp/'data'/name) as db:
                check=db.execute('PRAGMA quick_check').fetchone()[0];checks.append([name,check])
                if check!='ok':raise RuntimeError('Output SQLite integrity failed')
                topics=dict(db.execute('SELECT id,name FROM topics'))
                for tid,stamp,raw in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
                    topic=topics[tid];actual_counts[topic]+=1;digest_update(actual,topic,stamp,raw)
        if counts!=actual_counts or any(hashes[t].digest()!=actual[t].digest() for t in counts):raise RuntimeError('Copied payload/timestamp digest mismatch')
        meta_counts={t['topic_metadata']['name']:t['message_count'] for t in output_meta['topics_with_message_count'] if t['message_count']}
        if dict(counts)!=meta_counts:raise RuntimeError('Output metadata counts mismatch')
        if fingerprints(source)!=before:raise RuntimeError('Source changed during extraction')
        provenance=dict(source_bag=source.name,source_metadata_sha256=hashlib.sha256((source/'data/metadata.yaml').read_bytes()).hexdigest(),
            source_interval_s=[begin_s,end_s],source_recording_start_ns=start,interval_ns=[begin,end],warmup_start_ns=warm,
            warmup_topics=sorted(WARM_TOPICS),warmup_duration_s=5,bootstrap=bootstrap_manifest,
            source_files_unchanged=True,source_files=before,payload_timestamp_digest_verified=True,
            output_topic_sha256={t:h.hexdigest() for t,h in actual.items()},sqlite_checks=checks,
            copied_message_counts=dict(counts),coordinates='Original world frame and original sensor/pose header stamps; no rebasing')
        (tmp/'extraction.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2))
        info=dict(title=title,created=datetime.now().isoformat(timespec='seconds'),state='ready',
            settings=original.get('settings',{}),mapping_mode='recorded_superlio_clip',loaded_old_map=False,
            fresh_map=False,recording_mode='sensor_clip',mesh_enabled_during_recording=False,
            source_bag=source.name,source_interval_s=[begin_s,end_s],warmup_duration_s=5,map_directory='maps',
            replay_directory='replays',map_summary=map_info,duration=output_meta['duration']['nanoseconds']/1e9,
            messages=sum(counts.values()),topics=dict(counts))
        (tmp/'session.json').write_text(json.dumps(info,ensure_ascii=False,indent=2))
        (tmp/'README.md').write_text(f'# {title}\n\n来源：{source.name}，原包第 {begin_s}～{end_s} 秒（右端不含）。\n\n开头增加 5 秒位姿/TF/关节预热，没有相机/雷达画面属于预期。静态 TF、机器人模型与上次关节状态从原包补齐。传感器 payload、收包时间和消息头时间戳均保持原值；补齐消息仅调整收包时间。\n\n绑定地图仅使用本片段的已记录世界点云生成，坐标不归零。原包不修改，各包可独立回放与删除。校验记录见 extraction.json。\n')
        tmp.rename(target);result.append(str(target));print('CLIP_READY '+str(target),flush=True)
    for db,*_ in files:db.close()
    report=source/'diagnostics/stair_clips';report.mkdir(parents=True,exist_ok=True)
    (report/'clips.json').write_text(json.dumps(dict(source=str(source),clips=result),indent=2))
    print(json.dumps(dict(result='PASS',clips=result)),flush=True)

if __name__=='__main__':main()
