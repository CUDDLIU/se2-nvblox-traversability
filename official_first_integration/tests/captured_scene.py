#!/usr/bin/env python3
"""Stationary captured-input latency replay. Isolated ROS domain, no sender."""
import argparse,hashlib,json,math,os,signal,subprocess,time,sys
from array import array
from pathlib import Path
import numpy as np
assert os.environ.get('ROS_DOMAIN_ID')=='86' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path as RosPath
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Float64MultiArray,String

def main():
    root=Path(__file__).resolve().parents[1]
    ap=argparse.ArgumentParser();ap.add_argument('--native',action='store_true');ap.add_argument('snapshot',type=Path);ap.add_argument('--binary',type=Path,default=root/'build/first_node');ap.add_argument('--output',type=Path,required=True);ap.add_argument('--seconds',type=float,default=15.);ap.add_argument('--trace-new',action='store_true');ap.add_argument('--expect-window',type=float);ap.add_argument('--move-speed',type=float,default=0.)
    args=ap.parse_args();doc=json.loads((args.snapshot/'snapshot.json').read_text());raw=np.load(args.snapshot/'snapshot.npz')['raw'];body=doc['body'];height=doc['motion']['PatrolDevice']['Items']['MotionStatus']['Height']
    initial_body=list(body);travel=np.array([7.4,-7.4])-np.array(body[:2]);travel/=np.linalg.norm(travel)
    data=np.zeros(len(raw),dtype=np.dtype(dict(names=['x','y','z','timestamp','ring'],formats=['<f4','<f4','<f4','<f8','<u2'],offsets=[0,4,8,12,20],itemsize=24)))
    for i,name in enumerate(('x','y','z')):data[name]=raw[:,i]
    data['ring']=raw[:,4];relative=raw[:,3]-raw[:,3].max()
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('captured_first_test')
    pubs={name:node.create_publisher(kind,topic,qos_profile_sensor_data) for name,kind,topic in (
        ('odom',Odometry,'/nvblox_lio/odom'),('tf',PoseStamped,'/se2_scan/map_transform'),('robot',Float64MultiArray,'/first/robot_state'),('cloud',PointCloud2,'/LIDAR/POINTS_NX'))}
    route=node.create_publisher(RosPath,'/se2_global/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    occupancy=[]
    def capture_occupancy(m):
        if not m.width:return
        points=np.ndarray((m.width,3),dtype='<f4',buffer=bytes(m.data),strides=(m.point_step,4)).copy()
        occupancy[:]=[points]
    node.create_subscription(PointCloud2,'/first/occupancy',capture_occupancy,qos_profile_sensor_data)
    rows=[];node.create_subscription(String,'/first/status',lambda m:rows.append(json.loads(m.data)),10)
    log=(args.output/'node.log').open('w')
    command=[str(args.binary), '--ros-args','-p','library:='+str(root/'vendor/paths/first'),'-p','terrain:='+str(root/'maps/fefa46ce1e322691.terrain'),*(['-p','hit_trace:='+str(args.output/'hits.jsonl'),'-p','hit_trace_new_only:=true'] if args.trace_new else [])]
    if args.native:command=[sys.executable,str(root/'native/adapter.py'),'--output',str(args.output/'native')]
    proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    start=time.monotonic();ready=None;last_scan=0;sent=False;measured=None
    try:
        while time.monotonic()-start<args.seconds+35:
            t=time.monotonic();now=node.get_clock().now();stamp=now.to_msg()
            if measured is not None and args.move_speed:
                body[0],body[1]=np.array(initial_body[:2])+travel*args.move_speed*(t-measured)
            tf=PoseStamped();tf.header.frame_id='se2_scan_map';tf.header.stamp=stamp;tf.pose.orientation.w=1.;pubs['tf'].publish(tf)
            o=Odometry();o.header.frame_id='nvblox_odom';o.child_frame_id='base_link_dog';o.header.stamp=stamp
            o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=body[:3];o.pose.pose.orientation.z=math.sin(body[3]/2);o.pose.pose.orientation.w=math.cos(body[3]/2);pubs['odom'].publish(o)
            pubs['robot'].publish(Float64MultiArray(data=[height,float(0x3002),17.,.01,.01,0.]))
            if t-last_scan>=.1:
                # Preserve the measured 100 ms scan span and model 40 ms delivery.
                data['timestamp']=relative+now.nanoseconds/1e9-.04
                m=PointCloud2();m.header.frame_id='base_link_dog';lo=float(data['timestamp'].min());m.header.stamp.sec=int(lo);m.header.stamp.nanosec=int((lo-int(lo))*1e9)
                m.height=1;m.width=len(data);m.point_step=24;m.row_step=len(data)*24;m.is_dense=True
                m.fields=[PointField(name=n,offset=k,datatype=d,count=1) for n,k,d in [('x',0,7),('y',4,7),('z',8,7),('timestamp',12,8),('ring',20,4)]];m.data=array('B',data.tobytes());pubs['cloud'].publish(m);last_scan=t
            rclpy.spin_once(node,timeout_sec=.004)
            if proc.poll() is not None:raise RuntimeError((args.output/'node.log').read_text())
            if ready is None and rows and route.get_subscription_count():ready=t
            if ready is not None and t-ready>2 and not sent:
                path=RosPath();path.header.frame_id='se2_scan_map';path.header.stamp=stamp
                for x,y in (body[:2],(7.4,-7.4)):
                    p=PoseStamped();p.header=path.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=body[2]-height;p.pose.orientation.z=math.sin(body[3]/2);p.pose.orientation.w=math.cos(body[3]/2);path.poses.append(p)
                route.publish(path);sent=True;measured=t;rows.clear()
            if measured is not None and t-measured>=args.seconds:break
        assert measured is not None,'discovery timeout'
        (args.output/'planner.json').write_text(json.dumps(rows))
        active=[r for r in rows if r['state'] not in ('idle','waiting_inputs')]
        assert active,'no planning samples'
        assert sum(r['perception_ms']>0 for r in active)>=20,'point-cloud replay did not initialize'
        if args.trace_new:
            trace=[json.loads(line) for line in (args.output/'hits.jsonl').read_text().splitlines()]
            assert trace and all(row['new_only'] for row in trace),'new-cell trace not enabled'
            assert any(row['hits'] for row in trace),'trace missed initial observations'
            assert all(hit[14]==1 for row in trace for hit in row['hits']),'rejected point reported as occupied'
        report=dict(synthetic_motion_mps=args.move_speed,samples=len(active),holds={k:sum(r['hold_reason']==k for r in active) for k in sorted({r['hold_reason'] for r in active})})
        if occupancy:
            points=occupancy[-1];extent=np.max(np.abs(points[:,:2]-(0. if args.native else np.array(body[:2]))),axis=0)
            keys=np.rint((points/.05)-.5).astype('<i4');keys=keys[np.lexsort((keys[:,2],keys[:,1],keys[:,0]))]
            report['occupied_sha256']=hashlib.sha256(keys.tobytes()).hexdigest()
            report['occupied_points']=len(points);report['occupied_max_abs_xy_m']=extent.tolist()
            report['occupied_beyond_6m']=int(np.sum(np.linalg.norm(points[:,:2]-np.array(body[:2]),axis=1)>6.))
        if args.native:
            assert occupancy,'no official filtered cloud received'
            assert np.all(np.abs(points[:,:2])<=4.001),'outside official XY filter'
            assert np.all((points[:,2]>=-.301)&(points[:,2]<=.601)),'outside official Z filter'
            report['implementation']='nos_original_binary'
        if args.expect_window and not args.native:
            assert occupancy,'no displayed occupancy received'
            width=args.expect_window
            steps=np.array([.1,.1,.5]) if width==5. else np.array([.5,.5,.5])
            origin=np.floor(np.array(body[:3])/steps)*steps-np.array([width/2,width/2,3.])
            margin=.6 if args.move_speed else 1e-5
            assert np.all(points>=origin-margin) and np.all(points<origin+np.array([width,width,6.])+margin),'occupancy outside rolling box'
            report['verified_window_xy_m']=width
        for key in ('perception_ms','compute_ms','source_cloud_age_s','cloud_pipeline_ms','perception_stages_ms'):
            if not all(key in r for r in active):continue
            report[key+'_p50_p90_p99']=np.percentile([r[key] for r in active],[50,90,99],axis=0).tolist()
        (args.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
    finally:
        os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=10);log.close();node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
