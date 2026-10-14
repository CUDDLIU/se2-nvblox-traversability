"""Offline native-binary interface probe. Does not import or launch robot.py."""
import json,math,struct,sys,threading,time
from pathlib import Path
import numpy as np
from runtime import Runtime
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2,PointField
out=Path(sys.argv[1]);r=Runtime(out);received=[];stop=False

def receive():
    while not stop:
        try:
            k,b=r.receive()
            if k==101:v=struct.unpack('<dddd',b)
            elif k in (102,110,111,112):v=struct.unpack('<'+'i'*(len(b)//4),b)
            elif k==103:
                m=deserialize_message(b,PointCloud2);v=[m.width,m.height,m.header.frame_id]
            elif k==104:
                m=deserialize_message(b,RosPath);v=[len(m.poses),m.header.frame_id]
            else:v=[len(b)]
            received.append([time.time(),k,v])
        except TimeoutError:continue
        except Exception as e:
            if not stop:received.append([time.time(),'error',str(e)])
            break
thread=threading.Thread(target=receive,daemon=True);thread.start()
def stamp(m):
    t=time.time_ns();m.header.stamp.sec=t//10**9;m.header.stamp.nanosec=t%10**9
odom=Odometry();odom.header.frame_id='map';odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1.
cloud=PointCloud2();cloud.header.frame_id='lidar_link';cloud.height=1;cloud.is_dense=True;cloud.point_step=12
cloud.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
pts=np.array([[x,y,z] for x in np.arange(1.9,2.11,.02) for y in np.arange(1.5,1.71,.02) for z in (.1,.12,.14)],dtype='<f4')
cloud.width=len(pts);cloud.row_step=len(pts)*12;cloud.data=pts.tobytes()
goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=3.;goal.pose.orientation.w=1.
try:
    start=time.monotonic();sent=False
    while time.monotonic()-start<20:
        stamp(odom);r.send(1,serialize_message(odom));stamp(cloud);r.send(2,serialize_message(cloud))
        stamp(goal);r.send(4,serialize_message(goal))
        if not sent and time.monotonic()-start>10:
            r.send(7);r.send(5,serialize_message(goal));sent=True
        time.sleep(.05)
finally:
    stop=True;r.close();thread.join(3)
    (out/'probe.json').write_text(json.dumps(received))
    from collections import Counter
    print(Counter(str(x[1]) for x in received));print([x for x in received if x[1] in (110,112,'error')]);print('commands', [x[2] for x in received if x[1]==101][-15:])
