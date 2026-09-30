"""Archive nvblox telemetry during a bag recording; no robot connection."""
import argparse
import json
import signal
import time
import rclpy
from std_msgs.msg import String
from motion_status import TOPICS, atomic_json


def main():
    from pathlib import Path
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('nvblox_motion_capture')
    stream=(args.output/'motion_status.jsonl').open('a',buffering=1)
    status=dict(frames=0,invalid_packets=0,gaps_over_300ms=0,max_interval_s=0.,
                control_outputs='none; ROS subscription only',time_basis='original local receive time')
    first=last=None;running=True
    def receive(msg):
        nonlocal first,last
        try:
            data=json.loads(msg.data);received=data['receive_time_ns']
            assert isinstance(received,int)
            assert data['PatrolDevice']['Type']==1002 and data['PatrolDevice']['Command']==4
        except (ValueError,KeyError,TypeError,AssertionError):
            status['invalid_packets']+=1;return
        stream.write(msg.data+'\n')
        if last is not None:
            dt=(received-last)/1e9;status['max_interval_s']=max(status['max_interval_s'],dt)
            status['gaps_over_300ms']+=int(dt>.3)
        first=received if first is None else first;last=received;status['frames']+=1
    def health():
        now=time.time_ns();status.update(updated_time_ns=now,running=running,
            last_receive_time_ns=last,age_s=(now-last)/1e9 if last else None,
            receive_hz=(status['frames']-1)*1e9/(last-first) if first and last>first else 0.)
        atomic_json(args.output/'health.json',status)
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    node.create_subscription(String,TOPICS[0],receive,100)
    node.create_timer(1.,health)
    try:
        health()
        while running:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        running=False;health();stream.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
