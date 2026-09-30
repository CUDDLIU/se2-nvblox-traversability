"""Explicit live recording + RViz acceptance, scoped to nvblox-owned services."""
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import time

from bag_manager import BagManager
from recording_policy import cache_losses
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

ROOT=Path(__file__).resolve().parent.parent
OUT=ROOT/'bag_tools/validation/20260929_recording_tf'


def audit(path):
    edges={};ages={};counts={}
    for file in sorted((path/'data').glob('*.db3')):
        with sqlite3.connect(file.resolve().as_uri()+'?mode=ro',uri=True) as db:
            topics=dict(db.execute('SELECT id,name FROM topics'))
            for tid,n in db.execute('SELECT topic_id,count(*) FROM messages GROUP BY topic_id'):
                counts[topics[tid]]=counts.get(topics[tid],0)+n
            ids=[i for i,name in topics.items() if name=='/tf']
            for tid in ids:
                for receive,raw in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=?',(tid,)):
                    for transform in deserialize_message(raw,TFMessage).transforms:
                        key=transform.header.frame_id+' -> '+transform.child_frame_id
                        stamp=transform.header.stamp.sec*10**9+transform.header.stamp.nanosec
                        edges.setdefault(key,[]).append(stamp);ages.setdefault(key,[]).append((receive-stamp)*1e-9)
    gaps={}
    for key,times in edges.items():
        times.sort();delay=sorted(ages[key])
        gaps[key]=dict(count=len(times),max_header_gap_s=max(((b-a)*1e-9 for a,b in zip(times,times[1:])),default=0),
                       max_arrival_age_s=max(delay),median_arrival_age_s=delay[len(delay)//2])
    body=gaps.get('nvblox_odom -> base_link_dog',{})
    private=[v for k,v in gaps.items() if ' -> nvblox_robot/' in k]
    loss=cache_losses(path/'record.log')
    result=dict(bag=path.name,cache_lost=loss,counts=counts,tf=gaps,
        result='PASS' if loss==0 and body.get('count',0)>8000 and body['max_header_gap_s']<.3
        and len(private)>=16 and max(v['max_header_gap_s'] for v in private)<.5 else 'FAIL')
    (OUT/'live_record_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    return result


def main():
    manager=BagManager();path=None
    try:
        key=manager.record('录制修复验收（静止测试）',new_map=True)
        path=manager.bags/key
        for i in range(12):
            time.sleep(5);print('LIVE',i,manager.tick()['message'],flush=True)
        manager.finish_record();result=audit(path)
        manager.play(key,rebuild=False)
        time.sleep(8)
        subprocess.run(['gnome-screenshot','-f',str(OUT/'fixed_replay_rviz.png')],timeout=10)
        (OUT/'gui_replay_session.txt').write_text(str(manager.session))
        print('REPLAY',manager.tick(),flush=True)
        manager.stop_play();manager.restore_live()
        if result['result']!='PASS':raise RuntimeError('Live recording continuity check failed')
    finally:
        manager.close()


if __name__=='__main__':main()
