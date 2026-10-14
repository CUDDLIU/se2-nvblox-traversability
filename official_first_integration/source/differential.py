"""Read-only reference comparison in the native private network namespace.

Never imports robot.py or publishes a ROS actuator command. The vendor programs
are reference oracles only; the implementation under test is libnos_source.so.
"""
import importlib.util,json,math,os,struct,sys,threading,time
from collections import Counter
from pathlib import Path
import numpy as np
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2,PointField
from core import Core
BASE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('reference_runtime',BASE/'native/runtime.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
runtime=module.Runtime(out/'reference');core=Core();events=[];stop=False
def receive():
    while not stop:
        try:
            k,b=runtime.receive()
            if k==101:value=struct.unpack('<dddd',b)[1:]
            elif k in (102,110,111,112):value=struct.unpack('<'+'i'*(len(b)//4),b)
            elif k==103:
                m=deserialize_message(b,PointCloud2)
                value=np.column_stack([np.ndarray((m.height,m.width),dtype='<f4',buffer=m.data,offset=next(f.offset for f in m.fields if f.name==name),strides=(m.row_step,m.point_step)).reshape(-1) for name in ('x','y','z')])
            else:continue
            events.append((time.monotonic(),k,value))
        except TimeoutError:continue
        except Exception as e:
            if not stop:events.append((time.monotonic(),'error',repr(e)))
            break
thread=threading.Thread(target=receive,daemon=True);thread.start()
def stamp(m):
    t=time.time_ns();m.header.stamp.sec=t//10**9;m.header.stamp.nanosec=t%10**9
def pose(x,y,a=0):
    p=PoseStamped();p.header.frame_id='map';p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);return p
def block(x0,x1,y0,y1):
    return np.array([[x,y,z] for x in np.arange(x0,x1,.02) for y in np.arange(y0,y1,.02) for z in (.06,.08,.1,.12)],np.float32)
odom=Odometry();odom.header.frame_id='map';odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1.
cloud=PointCloud2();cloud.header.frame_id='lidar_link';cloud.height=1;cloud.is_dense=True;cloud.point_step=12
cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
anchor=block(-2.,-1.9,1.5,1.7)
scenes=[('clear',(1.5,0,0),anchor),('left',(1.3,.7,0),anchor),('right',(1.3,-.7,0),anchor),('front',(1.5,0,0),np.vstack((anchor,block(.9,1.1,-.5,.5)))),('side',(1.3,.7,0),np.vstack((anchor,block(.5,1.5,.65,.85)))),('blocked',(1.5,0,0),np.vstack((anchor,block(.46,.6,-.65,.65),block(-.6,-.46,-.65,.65),block(-.45,.45,.27,.4),block(-.45,.45,-.4,-.27))))]
report=[]
line=os.environ.get('NOS_COMPARE_MODE')=='line'
if not line:scenes.extend([('goal_blocked',(.8,0,0),np.vstack((anchor,block(.75,.95,-.15,.15)))),('goal_fine',(.3,0,0),anchor),('goal_yaw',(.01,0,.5),anchor)])
if line:
    scenes=[('turn',(3,1,0),anchor),('approach',(3,0,0),anchor),('close',(.4,0,0),anchor),('yaw',(.01,0,.5),anchor),('arrived',(.01,0,0),anchor),('obstacle',(3,0,0),np.vstack((anchor,block(.5,.7,-.3,.3))))]
try:
    for index,(name,target,points) in enumerate(scenes):
        final=target if line or name.startswith('goal_') else (8,0,0)
        expected_cloud=core.cloud(points);core.goal();expected=core.plan(target,final) if not line else core.line(target)
        cloud.width=len(points);cloud.row_step=len(points)*12;cloud.data=points.astype('<f4').tobytes()
        local=pose(*target);goal=pose(*final);route=RosPath();route.header.frame_id='map';route.poses=[pose(0,0),goal]
        start=time.monotonic();sent=False;duration=15 if index==0 else 4
        while time.monotonic()-start<duration:
            for kind,message in ((1,odom),(2,cloud),(3,route),(4,local)):
                stamp(message);runtime.send(kind,serialize_message(message))
            if not sent and time.monotonic()-start>(10 if index==0 else .3):
                runtime.send(6);runtime.send(7,b'\1' if line else b'');stamp(goal);runtime.send(5,serialize_message(goal));sent=True
            if line:expected=core.line(target)
            time.sleep(.1)
        recent=[e for e in events if e[0]>time.monotonic()-1.2]
        commands=Counter(tuple(round(x,5) for x in v) for _,k,v in recent if k==101)
        clouds=[v for _,k,v in recent if k==103]
        native_cloud=clouds[-1] if clouds else np.empty((0,3))
        def ordered(a):return a[np.lexsort(a.T[::-1])] if len(a) else a
        cloud_equal=expected_cloud.shape==native_cloud.shape and np.allclose(ordered(expected_cloud),ordered(native_cloud),atol=1e-6,rtol=0)
        row=dict(scene=name,source_velocity=[expected.x,expected.y,expected.w],source_status=expected.status,path_id=expected.id,native_commands=[dict(velocity=k,count=v) for k,v in commands.items()],native_statuses=sorted(set(tuple(v) for _,k,v in recent if k==102)),source_cloud_count=len(expected_cloud),native_cloud_count=len(native_cloud),cloud_equal=bool(cloud_equal))
        report.append(row);print(json.dumps(row),flush=True)
finally:
    stop=True;runtime.close();thread.join(3);core.close()
    (out/'comparison.json').write_text(json.dumps(report,indent=2))
