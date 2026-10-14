"""Saved goal metadata bridge for the existing ASA map frame.

This is ROS integration, not an emulation of AOS task storage/map switching.
One atomic request contains a pose and its profile. No profile is inherited
by the next RViz goal. A new session always starts without a saved goal.
"""
import copy,json,math,time,sys
from pathlib import Path
from geometry_msgs.msg import PoseStamped,PointStamped
from nav_msgs.msg import Path as RosPath,Odometry
from std_msgs.msg import String,Int32,Empty
from task_profile import DEFAULT,task_parameters
from supervision import WarningTimer
from sensor_msgs.msg import PointCloud2,Imu
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'global_planner'))
from navigation_modes import validate_route


def parse_saved_goal(doc):
    if doc.get('frame_id')!='se2_scan_map':raise ValueError('saved goal requires current ASA map coordinates')
    if doc.get('PointInfo',0)!=0:raise ValueError('point actions/map switching are not reconstructed; use an ordinary point in the active map')
    for k in ('Terrain','Posture','MapID'):
        if doc.get(k,0)!=0:raise ValueError('unsupported saved task action: '+k)
    for k in ('PosX','PosY','PosZ','AngleYaw'):
        if type(doc.get(k)) not in (int,float) or not math.isfinite(doc[k]):raise ValueError('invalid '+k)
    return task_parameters(doc)


def adapter_class(base):
    class TaskAdapter(base):
        def __init__(self,*args,**kw):
            self.terrain="flat";self.body_height=.55
            self.route_modes=[];self.mode_index=-1;self.route_gait=0;self.navigation_stamp=(-1,0);self.route_inhibited=False
            self.task=None;self.task_at=0.;self.actual_gait=0;self.gait_at=0.;self.task_error=None;self.awaiting_global_reset=False
            self.profile=DEFAULT.copy();self.velocity_limits=(1.5,.6,1.);self.warning_timer=WarningTimer(int(time.monotonic()*1000))
            super().__init__(*args,**kw)
            sensor=QoSProfile(depth=8,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.node.create_subscription(PointCloud2,'/nvblox_lio/body_cloud',self.registered_cloud,sensor)
            self.node.create_subscription(Odometry,'/nvblox_lio/odom_lidar',self.runtime.stairs.odometry,sensor)
            self.node.create_subscription(Imu,'/IMU',self.runtime.stairs.imu,QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.gait_pub=self.node.create_publisher(Int32,'/first/request_gait',1)
            self.global_goal=self.node.create_publisher(PoseStamped,'/se2_global/goal_pose',10)
            self.global_cancel=self.node.create_publisher(Empty,'/se2_global/cancel',10)
            self.node.create_subscription(String,'/first/saved_goal',self.saved_goal,10)
            self.node.create_subscription(PointStamped,'/clicked_point',self.operator_point,10)
            self.node.create_subscription(Empty,'/first/cancel',self.cancel_navigation,10)
            self.node.create_subscription(String,'/se2_global/navigation_route',self.navigation_route,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def operator_point(self,m):
            # A real new click cancels the saved task before ASA computes its
            # route; an unrelated path message cannot silently change gait.
            self.route_inhibited=False;self.clear_route('operator_point')

        def cancel_navigation(self,m):
            self.route_inhibited=True;self.clear_route('explicit_cancel');self.global_cancel.publish(Empty())

        def plain_route(self,m):
            pass  # Geometry-only topic is visual/reference output.

        def send(self,kind,msg=None):
            if kind!=7:return super().send(kind,msg)
            with self.lock:self.controls.append((7,json.dumps(dict(parameters=self.profile,terrain=self.terrain)).encode(),self.revision))

        def cloud(self,m):
            if self.terrain=='flat':super().cloud(m)

        def registered_cloud(self,m):
            self.runtime.stairs.cloud(m)
            at=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if self.terrain!='flat' and m.header.frame_id=='base_link_dog' and -.05<=time.time()-at<.35:self.cloud_at=at

        def clear_route(self,reason):
            self.terrain='flat';self.cloud_at=0.;self.filtered_at=0.
            self.runtime.ground_offset=None
            self.route_modes=[];self.mode_index=-1;self.route_gait=0
            self.task=None;self.task_error=None;self.awaiting_global_reset=False
            self.profile=DEFAULT.copy();self.velocity_limits=(1.5,.6,1.);self.warning_timer=WarningTimer(int(time.monotonic()*1000))
            self.params=False;self.params_sent=0.
            if hasattr(self,'gait_pub'):self.gait_pub.publish(Int32(data=0))
            super().clear_route(reason)

        def navigation_route(self,m):
            try:
                doc=validate_route(json.loads(m.data));stamp=tuple(doc['stamp'])
                if stamp<=self.navigation_stamp:return
                self.navigation_stamp=stamp
                if self.route_inhibited:return
                path=RosPath();path.header.frame_id=doc['frame_id']
                path.header.stamp.sec,path.header.stamp.nanosec=stamp
                for x,y,z,a in doc['poses']:
                    p=PoseStamped();p.header=path.header
                    p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.position.z=float(z)
                    p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);path.poses.append(p)
                if self.task is not None:
                    # Explicit saved tasks retain their explicitly supplied
                    # profile. Ordinary ASA goals use the segment annotations.
                    self.new_route(path);return
                super().new_route(path)
                if self.route is not None:
                    self.route_modes=doc['modes']
                    if any(m['terrain']!='flat' for m in self.route_modes):
                        self.route.ground_offset=self.body_height;self.runtime.ground_offset=self.body_height
                    self.activate_mode(0)
            except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as exc:
                self.clear_route('invalid_navigation_route');self.task_error=str(exc)

        def activate_mode(self,index):
            item=self.route_modes[index];profile=task_parameters(item)
            self.mode_index=index;self.route_gait=item['Gait']
            terrain=item['terrain']
            if self.profile==profile and self.terrain==terrain and self.params:return
            self.terrain=terrain;self.cloud_at=0.;self.filtered_at=0.
            # A parameter change resets the native goal. Invalidate old ACKs
            # and commands, then reissue the existing final goal after ACK.
            self.revision+=1;self.goal_sent=False;self.goal_accepted=False
            self.command=(0.,0.,0.,0.);self.route_changed=time.time()
            self.profile=profile;self.velocity_limits=tuple(profile[k] for k in ('maxSpeedX','maxSpeedY','maxTheta'))
            self.params=False;self.send(6);self.send(7);self.params_sent=time.time()

        def saved_goal(self,m):
            self.route_inhibited=False
            self.clear_route('saved_goal')
            try:
                doc=json.loads(m.data)
                self.profile=parse_saved_goal(doc)
                self.terrain="stair_up" if doc["Gait"]==0x3003 else "flat"
                if self.body is None or time.time()-self.tf_at>=1.5:raise ValueError('localization unavailable for saved goal')
                self.task=dict(doc);self.task_at=time.time()
                self.velocity_limits=tuple(self.profile[k] for k in ('maxSpeedX','maxSpeedY','maxTheta'))
                goal=PoseStamped();goal.header.frame_id='se2_scan_map';goal.header.stamp=self.node.get_clock().now().to_msg()
                goal.pose.position.x=float(doc['PosX']);goal.pose.position.y=float(doc['PosY']);goal.pose.position.z=float(doc['PosZ'])
                goal.pose.orientation.z=math.sin(doc['AngleYaw']/2);goal.pose.orientation.w=math.cos(doc['AngleYaw']/2)
                self.send(7);self.params_sent=time.time()
                if self.profile['directLine_mode']:
                    path=RosPath();path.header=goal.header
                    start=copy.deepcopy(goal);start.pose.position.x=float(self.body[0]);start.pose.position.y=float(self.body[1]);start.pose.position.z=float(self.body[2])
                    path.poses=[start,goal];self.accept_task_route(path)
                else:
                    self.awaiting_global_reset=True;self.global_goal.publish(goal)
            except (ValueError,TypeError,KeyError,AttributeError) as exc:
                self.clear_route('invalid_saved_goal');self.task_error=str(exc)

        def accept_task_route(self,m):
            # Do not call clear_route here: it would discard this atomic profile.
            # Route type comes from the same loaded transport module as base.
            Route=base.new_route.__globals__['Route']
            self.task_error=None;self.awaiting_global_reset=False;self.route=Route(m);self.send(3,m)

        def new_route(self,m):
            # The plain Path is for RViz/reference backends. ASA control uses
            # the atomic geometry+mode envelope, never a separately latched tag.
            if self.task is None:return
            if self.task is not None:
                # Only a path computed after this AUTO request, ending at its
                # requested pose, may acquire that task's gait/parameters.
                stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                if stamp<self.task_at:return
                if not m.poses and self.awaiting_global_reset:
                    self.awaiting_global_reset=False;return
                if m.poses and m.header.frame_id=='se2_scan_map' and not self.profile['directLine_mode']:
                    p=m.poses[-1].pose.position
                    if math.sqrt((p.x-self.task['PosX'])**2+(p.y-self.task['PosY'])**2+(p.z-self.task['PosZ'])**2)<.15:
                        if all(math.isfinite(v) for q in m.poses for v in (q.pose.position.x,q.pose.position.y,q.pose.position.z,q.pose.orientation.x,q.pose.orientation.y,q.pose.orientation.z,q.pose.orientation.w)) and all(sum(v*v for v in (q.pose.orientation.x,q.pose.orientation.y,q.pose.orientation.z,q.pose.orientation.w))>=.25 for q in m.poses):
                            self.accept_task_route(m);return
                if m.poses:
                    self.task_error='unexpected_saved_task_route';self.goal_accepted=False;self.goal_sent=False;self.command=(0.,0.,0.,0.);self.route=None;self.send(6);return
                # Empty path after accepting a route is cancellation/failure.
            super().new_route(m)

        def robot(self,m):
            self.gait_at=0.
            super().robot(m)
            if self.robot_at:
                self.body_height=float(m.data[0])
                if self.route is not None and self.route.ground_offset is not None:
                    self.route.ground_offset=self.body_height;self.runtime.ground_offset=self.body_height
                self.actual_gait=int(m.data[1]);self.gait_at=time.time()-max(m.data[3],m.data[4])

        def execution_hold(self,now):
            if self.warning_timer.error:return 'navigation_fault_'+hex(self.warning_timer.error)
            if self.task_error:return self.task_error
            target=self.task['Gait'] if self.task is not None else self.route_gait
            if not target:return None
            self.gait_pub.publish(Int32(data=target))
            if now-self.gait_at>=1.2 or self.actual_gait!=target:return 'waiting_task_gait'
            return None

        def tick(self):
            if self.route is not None and self.route_modes and self.body is not None and time.time()-self.odom_at<.2:
                # Projection is the existing route progress calculation, not a
                # second collision/support check. Progress is monotonic.
                try:
                    self.route.target(self.body)
                    index=self.mode_index
                    while index+1<len(self.route_modes) and self.route.progress>=self.route_modes[index]['end_m']:
                        index+=1
                    if index!=self.mode_index:self.activate_mode(index)
                except ValueError:pass  # Common adapter reports disconnection.
            if self.route is not None and self.goal_accepted:
                self.warning_timer.update(int(time.monotonic()*1000),self.native_status[1],localized=time.time()-self.tf_at<1.5,command_fresh=time.time()-self.command[0]<.2)
            if self.warning_timer.error:self.gait_pub.publish(Int32(data=0))
            # Gait requests must continue during the pending-gait telemetry
            # hold, where the common adapter won't call execution_hold.
            target=self.task['Gait'] if self.task is not None else self.route_gait
            if target and not self.finished and not self.warning_timer.error:
                self.gait_pub.publish(Int32(data=target))
            super().tick()
            if self.finished and target:
                self.task=None;self.route_gait=0;self.route_modes=[];self.mode_index=-1;self.gait_pub.publish(Int32(data=0))

        def task_diagnostics(self):
            return dict(task_gait=self.task['Gait'] if self.task else None,
                        actual_gait=self.actual_gait,task_error=self.task_error,
                        asa_mode=self.route_modes[self.mode_index] if self.route_modes else None,
                        asa_mode_index=self.mode_index,requested_gait=self.task['Gait'] if self.task else self.route_gait,
                        task_map_switch_supported=False,navigation_error=self.warning_timer.error)
    return TaskAdapter
