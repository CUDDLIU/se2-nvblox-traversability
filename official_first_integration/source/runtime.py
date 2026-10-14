"""In-process ROS transport to the reconstructed planner. No vendor process loads.

Freshness/route revisions are integration guards; recovered planner states retain
their NOS codes. Runs at 10 Hz with a coalesced cloud queue, never at scan backlog.
"""
import array,copy,json,math,queue,struct,threading,time
from pathlib import Path
import numpy as np
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped,TransformStamped
from sensor_msgs.msg import PointCloud2,PointField
from core import Core
from stair_perception import StairPerception
from task_profile import DEFAULT,validate_parameters
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def xyz_cloud(m):
    if m.point_step<12 or m.row_step<m.width*m.point_step or len(m.data)<m.row_step*m.height:raise ValueError('cloud layout')
    result=[]
    for name in ('x','y','z'):
        f=next(f for f in m.fields if f.name==name)
        if f.datatype!=7 or f.count!=1 or f.offset+4>m.point_step:raise ValueError('cloud field')
        if m.width*m.height==0:
            result.append(np.empty(0,np.float32));continue
        result.append(np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',buffer=m.data,offset=f.offset,strides=(m.row_step,m.point_step)).reshape(-1))
    return np.array(result,dtype=np.float32).T.copy()
def cloud_message(points,header):
    m=PointCloud2();m.header=copy.deepcopy(header);m.header.frame_id='base_link_dog';m.height=1;m.width=len(points);m.is_dense=True;m.point_step=12;m.row_step=12*len(points)
    # Typed uint8 storage bypasses generated Python per-byte validation. The
    # payload bytes are identical; validating hundreds of thousands of Python
    # integers here can cost more than the native filter on Jetson.
    m.fields=[PointField(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate(('x','y','z'))];m.data=array.array('B',points.astype('<f4',copy=False).tobytes());return m
class Continuity:
    """updateLocalGoal + checkLocalGoalContinuityAndConsume (0xb8e40/0xb9058)."""
    def __init__(self):self.last=None;self.updated=False;self.missing=0;self.bad_intervals=0
    def receive(self,t):
        if self.last is not None:
            self.bad_intervals=self.bad_intervals+1 if not .05-1e-6<=t-self.last<=.2+1e-6 else 0
        self.last=t;self.updated=True
    def consume(self):
        self.missing=0 if self.updated else self.missing+1;self.updated=False
        return self.last is not None and self.missing<10 and self.bad_intervals<10
class Runtime:
    implementation='nos_source_reconstruction'
    def __init__(self,output,mode='dwa'):
        if mode not in ('dwa','line'):raise ValueError('unsupported planner mode')
        self.stairs=StairPerception();self.terrain="flat"
        self.mode=mode;self.route_remaining=None;self.ground_offset=None;self.parameters=dict(DEFAULT,directLine_mode=mode=='line')
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.core=Core();self.lock=threading.Lock();self.pending={};self.controls=queue.Queue();self.results=queue.Queue();self.running=True
        self.odom=None;self.local=None;self.final=None;self.route=None;self.cloud_at=0.;self.cloud_sequence=0;self.goal_sequence=0;self.continuity=Continuity();self.diagnostics={};self.error=None;self.flat_timing={}
        self.trace=(self.output/'state_transitions.jsonl').open('w');self.previous_state=None
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
    def emit(self,kind,value):self.results.put((kind,value))
    def set_route_remaining(self,value):
        with self.lock:self.route_remaining=float(value)
    def send(self,kind,data=b''):
        if self.error:raise RuntimeError(self.error)
        if kind in (5,6,7):self.controls.put((kind,data))
        else:
            with self.lock:self.pending[kind]=data
    def receive(self):
        if self.error:raise RuntimeError(self.error)
        try:return self.results.get(timeout=.5)
        except queue.Empty:raise TimeoutError
    def relative(self,pose):
        body=self.odom.pose.pose;a=yaw(body.orientation);dx=pose.position.x-body.position.x;dy=pose.position.y-body.position.y
        return (math.cos(a)*dx+math.sin(a)*dy,-math.sin(a)*dx+math.cos(a)*dy,math.remainder(yaw(pose.orientation)-a,2*math.pi))
    def cycle(self):
        cycle_started=time.monotonic()
        with self.lock:pending=self.pending;self.pending={}
        for kind,b in pending.items():
            if kind==1:self.odom=deserialize_message(b,Odometry)
            elif kind==3:self.route=deserialize_message(b,RosPath)
            elif kind==4:
                self.local=deserialize_message(b,PoseStamped);self.continuity.receive(stamp(self.local))
            elif kind==2 and self.terrain=="flat":
                m=deserialize_message(b,PointCloud2);t=time.monotonic();points=self.core.cloud(xyz_cloud(m));self.cloud_at=stamp(m);self.cloud_sequence+=1
                self.flat_timing=self.core.cloud_timing()
                # Adapter already validates absolute per-point acquisition times.
                f=next((f for f in m.fields if f.name=='timestamp'),None)
                if f is not None:
                    if f.datatype!=8 or f.count!=1 or f.offset+8>m.point_step:raise ValueError('timestamp layout')
                    times=np.ndarray((m.height,m.width),dtype='>f8' if m.is_bigendian else '<f8',buffer=m.data,offset=f.offset,strides=(m.row_step,m.point_step))
                    finite=times[np.isfinite(times)]
                    if not finite.size or finite.min()<stamp(m)-.01 or finite.max()-finite.min()>.2 or finite.max()>time.time()+.05:raise ValueError('invalid scan time')
                    self.cloud_at=float(finite.max())
                self.emit(103,serialize_message(cloud_message(points,m.header)));self.emit(106,struct.pack('<ddd',stamp(m),stamp(m),(time.monotonic()-t)*1000))
        while True:
            try:kind,b=self.controls.get_nowait()
            except queue.Empty:break
            if kind==6:
                self.final=None;self.core.cancel();self.cloud_at=0.
                self.stairs.enable(self.terrain!='flat');self.core.obstacles(np.empty((0,3),np.float32))
                self.emit(111,struct.pack('<i',1))
            elif kind==7:
                try:
                    doc=json.loads(b.decode()) if b else self.parameters
                    terrain=doc.get('terrain','flat')
                    if terrain not in ('flat','stair_up','stair_down'):raise ValueError('invalid terrain')
                    parameters=validate_parameters(doc.get('parameters',doc))
                    self.terrain=terrain;self.cloud_at=0.;self.stairs.enable(terrain!='flat')
                    self.core.obstacles(np.empty((0,3),np.float32))
                    self.core.configure(parameters);self.parameters=parameters;self.mode='line' if parameters['directLine_mode'] else 'dwa'
                    self.final=None;self.emit(112,struct.pack('<i',1))
                except (ValueError,TypeError,KeyError):
                    self.final=None;self.core.cancel();self.emit(112,struct.pack('<i',0))
            elif kind==5:
                self.final=deserialize_message(b,PoseStamped);self.core.goal();self.goal_sequence=self.cloud_sequence;self.emit(110,struct.pack('<i',1))
        if self.terrain!='flat':
            perception=self.stairs.update(xyz_cloud)
            if perception is not None:
                header,points,rotation,elapsed=perception
                self.core.obstacles(points);self.cloud_at=header.stamp.sec+header.stamp.nanosec*1e-9;self.cloud_sequence+=1
                m=cloud_message(points,header);m.header.frame_id='first_source/gravity'
                self.emit(103,serialize_message(m));self.emit(106,struct.pack('<ddd',self.cloud_at,self.cloud_at,elapsed))
                tf=TransformStamped();tf.header=copy.deepcopy(header);tf.header.frame_id='first_native/base_link';tf.child_frame_id='first_source/gravity'
                tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,(-rotation[0],-rotation[1],-rotation[2],rotation[3]))
                self.emit(107,serialize_message(tf))
        now=time.time();valid_local=self.continuity.consume();status=999;warning=0;v=(0.,0.,0.);state='idle';path_id=-1;planning_ms=0.
        if self.final is not None:
            state='goal_received';status=1000
            if self.odom is None or not -.05<=now-stamp(self.odom)<.2:state='stale_odometry';warning=1001
            elif not -.05<=now-self.cloud_at<.35:state='stale_cloud';warning=1002
            elif self.mode=='dwa' and not valid_local:state='local_goal_discontinuous';warning=1004
            elif self.route is None or not self.route.poses:state='waiting_global_path'
            else:
                terminal=(self.route_remaining is None or self.route_remaining<=.65) and (self.ground_offset is None or abs(self.odom.pose.pose.position.z-self.ground_offset-self.final.pose.position.z)<.2)
                planning_started=time.monotonic()
                result=self.core.line(self.relative(self.final.pose),backward=self.parameters['backward_mode']) if self.mode=='line' else self.core.plan(self.relative(self.local.pose),self.relative(self.final.pose),terminal=terminal)
                planning_ms=(time.monotonic()-planning_started)*1000
                v=(result.x,result.y,result.w);status=result.status;path_id=result.id
                warning=status if status in (1002,1101,1102,2004,2005) else 2005 if status==2015 else 0
                state={999:'arrived',1001:'tracking',1002:'rotation_blocked',1003:'trajectory_blocked',1101:'no_feasible_path',1102:'goal_in_obstacle',2001:'line_turning',2002:'line_approaching',2003:'line_goal_adjustment',2004:'line_rotation_wait',2005:'line_obstacle_wait',2015:'line_obstacle_yaw_adjustment'}.get(status,'unknown')
                if status==999:self.final=None
                # Display the command's actual Euler sweep, including final limits.
                path=RosPath();path.header=copy.deepcopy(self.odom.header);path.header.frame_id='base_link_dog' if self.terrain=='flat' else 'first_source/gravity';x=y=a=0.
                for _ in range(10):
                    x+=.1*(math.cos(a)*v[0]-math.sin(a)*v[1]);y+=.1*(math.sin(a)*v[0]+math.cos(a)*v[1]);a+=.1*v[2]
                    p=PoseStamped();p.header=path.header;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);path.poses.append(p)
                self.emit(104,serialize_message(path))
        self.diagnostics=dict(planner_state=state,planner_mode=self.mode,planner_parameters=self.parameters,path_id=path_id,local_goal_missing=self.continuity.missing,local_goal_bad_intervals=self.continuity.bad_intervals,official_system_equivalence_verified=False,stair_execution_connected=True,physical_stair_tested=False,perception_mode='raw_nx' if self.terrain=='flat' else 'aos_height',terrain=self.terrain,**self.stairs.diagnostics)
        self.diagnostics.update(flat_filter=self.flat_timing,planning_ms=planning_ms,cycle_ms=(time.monotonic()-cycle_started)*1000)
        if (state,status,warning)!=self.previous_state:
            self.previous_state=(state,status,warning);self.trace.write(json.dumps(dict(stamp=now,state=state,status=status,warning=warning,velocity=v))+'\n');self.trace.flush()
        self.emit(102,struct.pack('<ii',status,warning));self.emit(101,struct.pack('<dddd',time.time(),*v))
    def loop(self):
        try:
            self.emit(100,b'')
            while self.running:
                start=time.monotonic();self.cycle();time.sleep(max(0.,.1-(time.monotonic()-start)))
        except Exception as e:self.error=repr(e)
    def close(self):
        self.running=False;self.thread.join(5)
        if self.thread.is_alive():raise RuntimeError('source worker did not stop')
        self.stairs.close();self.core.close();self.trace.close()
