"""Read-only review of a finished ROS 2 SQLite bag, including recorded TF.

Receive-time gaps describe the bag, not necessarily a physical sensor outage.
Large sensor payloads are sampled; their counts and receive gaps use every row.
"""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import re
import sqlite3

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState, PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def intervals(values, begin):
    a=np.asarray(values,dtype=np.int64)
    if not len(a):return {'count':0}
    a=np.sort(a);d=np.diff(a)*1e-9
    idx=np.flatnonzero(d>.3)
    return dict(count=len(a),first_s=float((a[0]-begin)*1e-9),last_s=float((a[-1]-begin)*1e-9),
        median_interval_s=float(np.median(d)) if len(d) else None,
        max_gap_s=float(d.max()) if len(d) else 0.,gaps_over_300ms=len(idx),
        gaps_over_1s=int(np.count_nonzero(d>1.)),
        largest_gaps=[dict(start_s=float((a[i]-begin)*1e-9),end_s=float((a[i+1]-begin)*1e-9),gap_s=float(d[i]))
                      for i in sorted(idx,key=lambda i:d[i],reverse=True)[:10]])


def stamp(msg):
    return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


def main():
    parser=argparse.ArgumentParser();parser.add_argument('bag',type=Path)
    parser.add_argument('--sample-period',type=float,default=5.)
    args=parser.parse_args();bag=args.bag;data=bag/'data'
    session=json.loads((bag/'session.json').read_text())
    if session.get('state')=='recording':raise RuntimeError('Review only a finished recording')
    meta=yaml.safe_load((data/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    begin=meta['starting_time']['nanoseconds_since_epoch'];duration=meta['duration']['nanoseconds']*1e-9
    out=bag/'diagnostics/recording_review';out.mkdir(parents=True,exist_ok=True)
    times=defaultdict(list);sizes=defaultdict(int);locations=defaultdict(list);connections=[];checks={}
    wanted={'/tf','/tf_static','/nvblox_robot/joint_states','/nvblox_lio/odom_lidar',
            '/camera/d435i/depth/image_rect_raw','/LIDAR/POINTS_NX'}
    for index,name in enumerate(meta['relative_file_paths']):
        c=sqlite3.connect('file:'+str(data/name)+'?mode=ro',uri=True);connections.append(c)
        c.execute('PRAGMA query_only=ON')
        checks[name]=[v[0] for v in c.execute('PRAGMA quick_check')]
        topics=dict(c.execute('SELECT id,name FROM topics'))
        for row_id,tid,ns,size in c.execute('SELECT id,topic_id,timestamp,length(data) FROM messages ORDER BY timestamp'):
            topic=topics[tid];times[topic].append(ns);sizes[topic]+=size
            if topic in wanted:locations[topic].append((ns,index,row_id))
        print('checked',name,flush=True)
    for rows in locations.values():rows.sort()
    stats={topic:{**intervals(values,begin),'payload_bytes':sizes[topic],
                  'average_hz_over_bag':len(values)/duration} for topic,values in times.items()}
    expected={v['topic_metadata']['name']:v['message_count'] for v in meta['topics_with_message_count']}
    if {k:len(v) for k,v in times.items()}!=expected:raise RuntimeError('Metadata and SQLite counts differ')
    def payload(row,cls):
        _,i,identifier=row
        raw=connections[i].execute('SELECT data FROM messages WHERE id=?',(identifier,)).fetchone()[0]
        return deserialize_message(raw,cls)
    transforms={};static=[]
    for topic in ('/tf','/tf_static'):
        for row in locations[topic]:
            msg=payload(row,TFMessage)
            for t in msg.transforms:
                key=t.header.frame_id+' -> '+t.child_frame_id
                v=t.transform.translation;q=t.transform.rotation
                value=[v.x,v.y,v.z,q.x,q.y,q.z,q.w]
                if topic=='/tf_static':
                    static.append(dict(parent=t.header.frame_id,child=t.child_frame_id,value=value));continue
                item=transforms.setdefault(key,dict(times=[],header_times=[],changes=0,last=None))
                item['times'].append(row[0]);item['header_times'].append(stamp(t))
                if item['last'] is not None and not np.allclose(item['last'],value,rtol=0.,atol=1e-6):item['changes']+=1
                item['last']=value
    tf_summary={key:dict(receive=intervals(v['times'],begin),header=intervals(v['header_times'],begin),
                        changing_transforms=v['changes'],last_transform=v['last']) for key,v in transforms.items()}
    joint_values=defaultdict(list);joint_times=[]
    for row in locations['/nvblox_robot/joint_states']:
        msg=payload(row,JointState);joint_times.append(stamp(msg))
        for name,value in zip(msg.name,msg.position):joint_values[name].append(value)
    joints={name:dict(min=float(np.min(v)),max=float(np.max(v)),range=float(np.ptp(v))) for name,v in joint_values.items()}
    trajectory=[]
    for row in locations['/nvblox_lio/odom_lidar']:
        msg=payload(row,Odometry);v=msg.pose.pose.position;q=msg.pose.pose.orientation
        trajectory.append([(row[0]-begin)*1e-9,stamp(msg)*1e-9,v.x,v.y,v.z,q.x,q.y,q.z,q.w])
    pose=np.asarray(trajectory);xyz=pose[:,2:5]
    delta=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
    odom=dict(samples=len(pose),position_min=xyz.min(axis=0).tolist(),position_max=xyz.max(axis=0).tolist(),
              start_position=xyz[0].tolist(),end_position=xyz[-1].tolist(),
              endpoint_separation_m=float(np.linalg.norm(xyz[-1]-xyz[0])),
              sampled_path_length_m=float(delta.sum()),max_adjacent_displacement_m=float(delta.max()),
              header=intervals((pose[:,1]*1e9).astype(np.int64),begin))
    with (out/'trajectory.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['bag_elapsed','stamp','x','y','z','qx','qy','qz','qw']);writer.writerows(trajectory)
    sensors={};depth_arrays=[]
    for topic,cls in [('/camera/d435i/depth/image_rect_raw',Image),('/LIDAR/POINTS_NX',PointCloud2)]:
        rows=locations[topic];ts=np.asarray([r[0] for r in rows],dtype=np.int64)
        chosen=set()
        for seconds in np.arange(0,duration,args.sample_period):
            target=begin+int(seconds*1e9);j=int(np.searchsorted(ts,target))
            candidates=[i for i in (j-1,j) if 0<=i<len(ts)]
            if candidates:chosen.add(min(candidates,key=lambda i:abs(int(ts[i])-target)))
        samples=[]
        for j in sorted(chosen):
            row=rows[j];msg=payload(row,cls)
            item=dict(elapsed_s=(row[0]-begin)*1e-9,header_elapsed_s=(stamp(msg)-begin)*1e-9,frame=msg.header.frame_id)
            if cls is Image:
                if msg.encoding!='16UC1':raise ValueError('Unsupported depth encoding: '+msg.encoding)
                depth=np.ndarray((msg.height,msg.width),dtype='>u2' if msg.is_bigendian else '<u2',
                    buffer=msg.data,strides=(msg.step,2))
                valid=(depth>0)&(depth<65535)
                item.update(width=msg.width,height=msg.height,valid_fraction=float(valid.mean()),
                    lower_half_valid_fraction=float(valid[msg.height//2:].mean()))
                depth_arrays.append((item['elapsed_s'],depth[::2,::2].copy()))
            else:
                fields={f.name:f for f in msg.fields};f=fields['ring']
                if f.datatype!=4 or f.count!=1:raise ValueError('Unsupported ring layout')
                ring=np.ndarray((msg.height,msg.width),dtype='>u2' if msg.is_bigendian else '<u2',
                    buffer=msg.data,offset=f.offset,strides=(msg.row_step,msg.point_step))
                item.update(points=msg.width*msg.height,front_points=int(np.count_nonzero(ring<96)),
                    rear_points=int(np.count_nonzero((ring>=96)&(ring<192))),unknown_rings=int(np.count_nonzero(ring>=192)))
            samples.append(item)
        sensors[topic]=dict(scope='sampled payloads only; full message timing above',sample_period_s=args.sample_period,samples=samples)
    record=(bag/'record.log').read_text(errors='replace')
    # rosbag prints cumulative counters again at each split and at shutdown.
    # Keep the maximum reported count, never sum repeated reports.
    totals=[int(x) for x in re.findall(r'Total lost:\s*(\d+)',record)]
    lost={}
    for block in re.findall(r'Cache buffers lost messages per topic:(.*?)Total lost:',record,re.S):
        for topic,count in re.findall(r'(/[^\s:]+):\s*(\d+)',block):lost[topic]=max(lost.get(topic,0),int(count))
    report=dict(bag=bag.name,state=session.get('state'),duration_s=duration,
        database_bytes=sum((data/n).stat().st_size for n in meta['relative_file_paths']),
        sqlite_quick_check=checks,metadata_counts_match=True,topics=stats,tf=tf_summary,static_tf=static,
        joints=joints,joint_header=intervals(joint_times,begin),odometry=odom,sensor_payload_samples=sensors,
        recorder_cache_lost_total=max(totals,default=0),recorder_cache_lost_per_topic=lost,
        recorded_terrain_settings=session.get('settings'),
        limitations=['Receive gaps alone do not identify sensor hardware faults.',
                     'Payload quality is sampled; SQLite quick_check is not a per-message semantic checksum.',
                     'Endpoint separation is not loop-closure error without knowing the actual end location.'])
    (out/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(13,9),layout='constrained')
    axes[0].plot(pose[:,0],pose[:,4]);axes[0].set(ylabel='Body world Z (m)',title=bag.name)
    for i,(topic,label) in enumerate([('/LIDAR/POINTS_NX','LiDAR'),('/camera/d435i/depth/image_rect_raw','Depth'),
                                    ('/nvblox_node/mesh','Mesh'),('/nvblox_robot/joint_states','Joints')]):
        seconds=(np.asarray(times[topic])-begin)*1e-9
        bins=np.arange(0,duration+1,1.);counts,_=np.histogram(seconds,bins)
        axes[1].plot(bins[:-1],counts,label=label)
    axes[1].set(ylabel='Recorded messages / second');axes[1].legend(ncol=4)
    sample=sensors['/camera/d435i/depth/image_rect_raw']['samples']
    axes[2].plot([x['elapsed_s'] for x in sample],[x['valid_fraction'] for x in sample],label='Whole depth image')
    axes[2].plot([x['elapsed_s'] for x in sample],[x['lower_half_valid_fraction'] for x in sample],label='Lower half')
    axes[2].set(xlabel='Bag receive elapsed (s)',ylabel='Valid depth fraction (sampled)',ylim=(0,1));axes[2].legend()
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(out/'timeline.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(12,7),layout='constrained')
    for ax,target in zip(axes.flat,[30,125,160,205,240,360]):
        t,d=min(depth_arrays,key=lambda x:abs(x[0]-target))
        masked=np.ma.masked_where((d==0)|(d==65535),d*.001)
        ax.imshow(masked,vmin=0,vmax=5,cmap='viridis');ax.set_title(f'{t:.1f} s, depth 0-5 m');ax.axis('off')
    fig.savefig(out/'depth_samples.png',dpi=140);plt.close(fig)
    for c in connections:c.close()
    print(json.dumps(dict(report=str(out/'review.json'),duration_s=duration,odometry=odom,
        cache_lost=report['recorder_cache_lost_total'],joints=len(joints),tf_edges=len(tf_summary)),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
