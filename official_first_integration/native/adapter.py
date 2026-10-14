#!/usr/bin/env python3
"""ASA/ROS transport and execution gates around the original NOS local planner.

The ASA path-to-local-goal adapter is explicitly integration code; collision,
cloud filtering, candidate scoring and velocity generation run in NOS binaries.
"""
import argparse,copy,json,math,queue,signal,struct,sys,threading,time
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,TwistStamped,TransformStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String,Float64MultiArray
from tf2_ros import TransformBroadcaster
from runtime import Runtime
FRAME='se2_scan_map'
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def xyz(p):return np.array([p.x,p.y,p.z])
def quat(q):return np.array([q.x,q.y,q.z,q.w])
def fill(v,values):v.x,v.y,v.z=map(float,values)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Route:
    """Map the ASA path to the official external /local_goal interface."""
    def __init__(self,msg):
        self.msg=msg;self.points=np.array([xyz(p.pose.position) for p in msg.poses]);self.progress=0.;self.ground_offset=None
        self.arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(self.points[:,:2],axis=0),axis=1))]
    def target(self,body):
        best=(float('inf'),self.progress)
        for i in range(len(self.points)-1):
            if self.arc[i+1]<self.progress-.1:continue
            if self.arc[i]>self.progress+3.:break
            position=np.asarray(body).copy()
            dimensions=2 if self.ground_offset is None else 3
            if dimensions==3:position[2]-=self.ground_offset
            a,b=self.points[i:i+2,:dimensions];d=b-a;length=self.arc[i+1]-self.arc[i]
            u=float(np.clip(np.dot(position[:dimensions]-a,d)/max(np.dot(d,d),1e-12),0.,1.))
            distance=float(np.linalg.norm(position[:dimensions]-a-u*d))
            if distance<best[0]:best=(distance,self.arc[i]+u*length)
        if len(self.points)==1:best=(float(np.linalg.norm(body[:2]-self.points[0,:2])),0.)
        if best[0]>1.:raise ValueError('route disconnected')
        self.progress=max(self.progress,best[1]);s=min(self.arc[-1],self.progress+1.5)
        i=min(len(self.points)-1,int(np.searchsorted(self.arc,s,side='left')))
        goal=copy.deepcopy(self.msg.poses[i]);goal.header.frame_id=FRAME
        if i and self.arc[i]>self.arc[i-1]+1e-8:
            u=(s-self.arc[i-1])/(self.arc[i]-self.arc[i-1]);fill(goal.pose.position,self.points[i-1]*(1-u)+self.points[i]*u)
        return goal,float(self.arc[-1]-self.progress)

class Adapter:
    def __init__(self,node,output,runtime_factory=Runtime):
        self.node=node;self.runtime=runtime_factory(output);self.running=True;self.failure=None
        self.lock=threading.Lock();self.outbound={};self.controls=deque();self.responses=deque();self.latest={}
        self.workers=[threading.Thread(target=self.reader,daemon=True),threading.Thread(target=self.writer,daemon=True)]
        self.rotation=Rotation.identity();self.translation=np.zeros(3);self.tf_at=0.;self.odom_at=0.;self.cloud_at=0.;self.robot_at=0.;self.filtered_at=0.;self.perception_ms=0.
        self.cloud_received=0;self.cloud_accepted=0;self.cloud_rejected={};self.cloud_receive_age=0.
        self.last_odom=0.;self.raw_previous=None;self.last_tf=None;self.body=None;self.route=None;self.finished=False;self.revision=0;self.goal_sent=False;self.goal_accepted=False;self.route_changed=0.;self.accepted_at=0.;self.params=False;self.params_sent=0.;self.native_status=(0,1003);self.native_status_at=0.;self.command=(0.,0.,0.,0.);self.last_report=0.;self.last_local=0.;self.last_tick=time.monotonic();self.pose_history=deque(maxlen=100);self.cloud_end={};self.last_cloud_visual=0.
        latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd=node.create_publisher(TwistStamped,'/first/cmd_vel_stamped',latest)
        self.status=node.create_publisher(String,'/first/status',10)
        self.aligned=node.create_publisher(Odometry,'/first/aligned_odom',latest)
        self.cloud_pub=node.create_publisher(PointCloud2,'/first/occupancy',latest)
        self.path_pub=node.create_publisher(RosPath,'/first/local_path',1)
        self.timing=node.create_publisher(Float64MultiArray,'/first/control_timing',100)
        self.broadcaster=TransformBroadcaster(node)
        node.create_subscription(PoseStamped,'/se2_scan/map_transform',self.transform,latest)
        node.create_subscription(Odometry,'/nvblox_lio/odom',self.odometry,latest)
        node.create_subscription(PointCloud2,'/LIDAR/POINTS_NX',self.cloud,latest)
        node.create_subscription(Float64MultiArray,'/first/robot_state',self.robot,latest)
        node.create_subscription(RosPath,'/se2_global/path',getattr(self,'plain_route',self.new_route),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_timer(1/30,self.tick)
        for t in self.workers:t.start()
    def send(self,kind,msg=None):
        data=b'' if msg is None else serialize_message(msg)
        with self.lock:
            if kind in (5,6,7):self.controls.append((kind,data,self.revision))
            else:self.outbound[kind]=data
    def writer(self):
        try:
            while self.running:
                with self.lock:
                    control=self.controls.popleft() if self.controls else None
                    batch=self.outbound;self.outbound={}
                    if control:self.responses.append((control[0]+105,control[2]))
                # Fresh measurements precede a goal request.
                for k,b in batch.items():self.runtime.send(k,b)
                if control:self.runtime.send(*control[:2])
                time.sleep(.002)
        except Exception as e:self.failure=str(e)
    def reader(self):
        try:
            while self.running:
                try:k,b=self.runtime.receive()
                except TimeoutError:continue
                with self.lock:
                    if k in (110,111,112):
                        if not self.responses or self.responses[0][0]!=k:raise RuntimeError('unexpected service response')
                        _,revision=self.responses.popleft();self.latest[k]=(b,revision,time.time())
                    else:self.latest[k]=(b,self.revision,time.time())
        except Exception as e:
            if self.running:self.failure=str(e)
    def transform(self,m):
        try:
            q=quat(m.pose.orientation);p=xyz(m.pose.position);age=time.time()-stamp(m)
            if m.header.frame_id!=FRAME or not -.1<=age<1.5 or not np.isfinite(np.r_[p,q]).all() or np.linalg.norm(q)<.5:raise ValueError('alignment')
            r=Rotation.from_quat(q)
            anchor=self.raw_previous[1] if self.raw_previous is not None else np.zeros(3)
            correction=r.apply(anchor)+p-(self.rotation.apply(anchor)+self.translation)
            if self.last_tf is not None and (np.linalg.norm(correction)>.15 or (r*self.rotation.inv()).magnitude()>.1):
                self.clear_route('localization_jump')
            self.rotation=r;self.translation=p;self.tf_at=stamp(m);self.last_tf=m
        except ValueError:
            if self.tf_at and self.route is not None:self.clear_route('localization_invalid')
            self.tf_at=0.
    def odometry(self,m):
        t=time.time()
        if t-self.last_odom<1/35:return
        q=quat(m.pose.pose.orientation);p=xyz(m.pose.pose.position)
        if m.header.frame_id!='nvblox_odom' or m.child_frame_id!='base_link_dog' or not -.05<=t-stamp(m)<.2 or t-self.tf_at>=1.5 or not np.isfinite(np.r_[p,q]).all() or np.linalg.norm(q)<.5:return
        if self.raw_previous is not None:
            old_stamp,old_position=self.raw_previous
            if stamp(m)<old_stamp-.05 or np.linalg.norm(p-old_position)>.3:
                self.clear_route('odometry_jump');self.tf_at=0.;self.odom_at=0.;self.raw_previous=None;return
            if stamp(m)<=old_stamp:return
        self.raw_previous=(stamp(m),p.copy())
        r=self.rotation*Rotation.from_quat(q);p=self.rotation.apply(p)+self.translation
        self.body=p;self.odom_at=stamp(m);self.last_odom=t
        m.header.frame_id=FRAME;fill(m.pose.pose.position,p);m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,r.as_quat())
        self.aligned.publish(m);self.send(1,m);self.pose_history.append((stamp(m),p,r))
        tf=TransformStamped();tf.header=m.header;tf.child_frame_id='first_native/base_link';fill(tf.transform.translation,p);tf.transform.rotation=m.pose.pose.orientation;self.broadcaster.sendTransform(tf)
    def cloud(self,m):
        age=time.time()-stamp(m)
        self.cloud_received+=1;self.cloud_receive_age=age
        def reject(reason):self.cloud_rejected[reason]=self.cloud_rejected.get(reason,0)+1
        if m.header.frame_id not in ('lidar_link','base_link_dog'):return reject('frame')
        if not -.05<=age<.35:return reject('header_age')
        if any(n not in {f.name for f in m.fields} for n in ('x','y','z')):return reject('fields')
        # The merged NX cloud is already in the body coordinate convention used
        # by NOS /LIDAR/POINTS; preserve every point and acquisition timestamp.
        end=stamp(m)
        fields={f.name:f for f in m.fields}
        if 'timestamp' in fields:
            f=fields['timestamp']
            if f.datatype!=8 or f.count!=1:return reject('timestamp_type')
            try:
                times=np.ndarray((m.height,m.width),dtype='>f8' if m.is_bigendian else '<f8',buffer=m.data,offset=f.offset,strides=(m.row_step,m.point_step))
                valid=times[np.isfinite(times)]
                if not valid.size:return reject('empty_timestamps')
                lo,hi=float(valid.min()),float(valid.max())
                if lo<stamp(m)-.01 or hi-lo>.2 or hi>time.time()+.05:return reject('timestamp_range')
                end=hi
            except (ValueError,TypeError):return reject('timestamp_layout')
        # Match the previous controller's scan-end freshness convention. The
        # original PointCloud2 and its start stamp still go to NOS unchanged.
        self.cloud_at=end;self.cloud_end[(m.header.stamp.sec,m.header.stamp.nanosec)]=end
        self.cloud_accepted+=1
        while len(self.cloud_end)>100:self.cloud_end.pop(next(iter(self.cloud_end)))
        self.send(2,m)
    def robot(self,m):
        a=m.data
        self.robot_at=0.
        if len(a) not in (6,7) or not all(math.isfinite(x) for x in a):return
        if not .2<a[0]<1. or a[3]>=1.2 or a[4]>=.35 or int(a[5])!=0:return
        self.robot_at=a[6] if len(a)==7 else time.time()-a[4]
    def clear_route(self,reason):
        self.revision+=1;self.finished=False;self.route=None;self.goal_sent=False;self.goal_accepted=False;self.command=(0.,0.,0.,0.);self.route_changed=time.time();self.send(6)
    def new_route(self,m):
        self.clear_route('new_route')
        if m.header.frame_id!=FRAME or not m.poses:return
        for p in m.poses:
            if not np.isfinite(np.r_[xyz(p.pose.position),quat(p.pose.orientation)]).all() or np.linalg.norm(quat(p.pose.orientation))<.5:return
        self.route=Route(m);self.send(3,m)
    def tick(self):
        now=time.time();period=(time.monotonic()-self.last_tick)*1000;self.last_tick=time.monotonic()
        with self.lock:items=self.latest;self.latest={}
        for k,(b,rev,at) in items.items():
            if k==101:self.command=struct.unpack('<dddd',b)
            elif k==102:self.native_status=struct.unpack('<ii',b);self.native_status_at=at
            elif k==112 and rev==self.revision:
                self.params=struct.unpack('<i',b)[0]==1
                if not self.params:self.failure='planner parameter transaction rejected'
            elif k==110 and rev==self.revision:
                self.goal_accepted=struct.unpack('<i',b)[0]==1;self.accepted_at=at
                if not self.goal_accepted:self.failure='official goal service rejected goal'
            elif k==103:
                m=deserialize_message(b,PointCloud2)
                if m.header.frame_id!='first_source/gravity':m.header.frame_id='first_native/base_link'
                self.filtered_at=self.cloud_end.get((m.header.stamp.sec,m.header.stamp.nanosec),stamp(m))
                if now-self.last_cloud_visual>=.19:self.cloud_pub.publish(m);self.last_cloud_visual=now
            elif k==107:self.broadcaster.sendTransform(deserialize_message(b,TransformStamped))
            elif k==106:self.perception_ms=struct.unpack('<ddd',b)[2]
            elif k==104:
                m=deserialize_message(b,RosPath);m.header.frame_id='first_native/base_link' if m.header.frame_id in ('base_link','base_link_dog') else m.header.frame_id if m.header.frame_id=='first_source/gravity' else FRAME
                if not hasattr(self,'native_path_header'):
                    self.native_path_header={'frame':m.header.frame_id,'stamp':stamp(m),'received':now};(self.runtime.output/'path_header.json').write_text(json.dumps(self.native_path_header))
                for p in m.poses:p.header=m.header
                self.path_pub.publish(m)
        if not self.params and now-self.params_sent>1. and self.native_status[1]!=1003:
            self.send(7);self.params_sent=now
        ages=dict(odom_age_s=now-self.odom_at,cloud_age_s=now-self.cloud_at,robot_age_s=now-self.robot_at,localization_age_s=now-self.tf_at,source_cloud_age_s=now-self.filtered_at)
        hold=self.failure or ('stale_localization' if ages['localization_age_s']>=1.5 else 'stale_odometry' if ages['odom_age_s']>=.2 else 'stale_cloud' if ages['cloud_age_s']>=.35 or ages['source_cloud_age_s']>=.35 else 'stale_robot_state' if ages['robot_age_s']>=.35 else None)
        if hold is None and hasattr(self,'execution_hold'):hold=self.execution_hold(now)
        remaining=0.;target=None
        if self.route is not None and hold is None:
            try:
                target,remaining=self.route.target(self.body);target.header.stamp=self.node.get_clock().now().to_msg()
                if hasattr(self.runtime,'set_route_remaining'):self.runtime.set_route_remaining(remaining)
                # NOS rejects consecutive local-goal stamps less than 50 ms apart.
                if now-self.last_local>=.09:self.send(4,target);self.last_local=now
            except ValueError as e:hold=str(e)
            if target is not None and self.params and not self.goal_sent and now-self.route_changed>=.2:
                goal=copy.deepcopy(self.route.msg.poses[-1]);goal.header=target.header;self.send(5,goal);self.goal_sent=True
        if hold is None:
            hold='idle' if self.route is None else 'native_initializing' if not self.params else 'waiting_goal_ack' if not self.goal_accepted else 'stale_native_command' if not 0<=now-self.command[0]<.2 or self.command[0]<self.accepted_at+.1 else 'none'
        if self.goal_accepted and self.native_status[0]==999 and self.native_status_at>=self.accepted_at+.1 and now-self.native_status_at<.35 and now-self.accepted_at>.3 and self.body is not None and self.route is not None and np.linalg.norm(self.body[:2]-self.route.points[-1,:2])<.15 and (self.route.ground_offset is None or abs(self.body[2]-self.route.ground_offset-self.route.points[-1,2])<.2):
            self.route=None;self.goal_accepted=False;self.finished=True;hold='arrived'
        if self.route is None:hold='arrived' if self.finished else 'idle'
        v=self.command[1:] if hold=='none' else (0.,0.,0.)
        limits=getattr(self,'velocity_limits',(1.5,.6,1.))
        if not all(math.isfinite(x) for x in v) or any(abs(x)>limit+.0001 for x,limit in zip(v,limits)):
            self.failure='invalid_official_command';v=(0.,0.,0.);hold=self.failure
        m=TwistStamped();m.header.stamp=self.node.get_clock().now().to_msg();m.header.frame_id='base_link_dog';m.twist.linear.x,m.twist.linear.y,m.twist.angular.z=v;self.cmd.publish(m)
        self.timing.publish(Float64MultiArray(data=[now,period,0.,0.,float(hold=='none')]))
        if now-self.last_report>.1:
            self.last_report=now
            doc=dict(implementation=getattr(self.runtime,'implementation','nos_original_binary'),state='tracking' if hold=='none' else hold,hold_reason=hold,command_valid=hold=='none',native_status=self.native_status,perception_ms=self.perception_ms,selected_velocity=v,route_progress_m=self.route.progress if self.route else 0.,route_remaining_m=remaining,**ages)
            if hasattr(self.runtime,'diagnostics'):doc.update(self.runtime.diagnostics)
            doc.update(raw_cloud_received=self.cloud_received,raw_cloud_accepted=self.cloud_accepted,raw_cloud_rejected=self.cloud_rejected.copy(),raw_cloud_receive_age_s=self.cloud_receive_age)
            if hasattr(self,'task_diagnostics'):doc.update(self.task_diagnostics())
            self.status.publish(String(data=json.dumps(doc)))
    def close(self):
        self.running=False;self.runtime.close()
        for t in self.workers:t.join(2)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('first_native_adapter');adapter=None;running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        adapter=Adapter(node,args.output)
        while running and rclpy.ok():rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        if adapter:adapter.close()
        node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
