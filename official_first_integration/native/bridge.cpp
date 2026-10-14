// Transport-only adapter. Planning and filtering execute in the untouched NOS binaries.
#include <drdds/core/drdds_core.h>
#include <drdds/core/drdds_manager.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <atomic>
#include <memory>
#include <cstring>
#include <mutex>
#include <iostream>
#include <stdexcept>
#include <thread>
using Payload=eprosima::fastrtps::rtps::SerializedPayload_t;
static int fd=-1;
static std::mutex output,cloud_mutex;
static std::shared_ptr<sensor_msgs::msg::PointCloud2> pending_cloud;
static std::atomic<bool> cloud_inflight{false};
static builtin_interfaces::msg::Time submitted_time;
static double submitted_at=0.;
static bool io(void* p,size_t n,bool write){
 auto c=static_cast<char*>(p);
 while(n){auto k=write?send(fd,c,n,MSG_NOSIGNAL):recv(fd,c,n,0);if(k<=0)return false;c+=k;n-=k;}return true;
}
static void packet(uint32_t kind,const void* p,uint32_t n){
 std::lock_guard<std::mutex> g(output);uint32_t head[]{n,kind};
 if(!io(head,sizeof(head),true)||!io(const_cast<void*>(p),n,true))_exit(3);
}
template<class T> static typename T::type decode(const std::vector<uint8_t>& data){
 Payload p(data.size());memcpy(p.data,data.data(),data.size());p.length=data.size();
 T type;typename T::type m;if(!type.deserialize(&p,&m))throw std::runtime_error("CDR decode");return m;
}
template<class T> static void emit(uint32_t kind,const typename T::type* msg){
 T type;auto copy=*msg;Payload p(type.getSerializedSizeProvider(&copy)());
 if(!type.serialize(&copy,&p))return;packet(kind,p.data,p.length);
}
static double seconds(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
int main(int argc,char**argv){
 if(argc!=2)return 2;
 // Network and IPC namespaces are mandatory; refuse host execution.
 char own[128]{};const char* host=getenv("NOS_HOST_NETNS");
 if(readlink("/proc/self/ns/net",own,127)<0||!host||strcmp(own,host)==0){
  std::cerr<<"native bridge requires a separate network namespace"<<std::endl;return 2;
 }
 fd=socket(AF_UNIX,SOCK_STREAM,0);sockaddr_un a{};a.sun_family=AF_UNIX;
 if(strlen(argv[1])>=sizeof(a.sun_path))return 2;strcpy(a.sun_path,argv[1]);
 if(connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a)))return 2;
 DrDDSManager::Init(0,"eth0");
 ChannelOdometry odom("/ODOM");ChannelPointCloud2 cloud("/LIDAR/POINTS",0,true);
 ChannelPath path("/path_Astar");ChannelPoseStamped local("/local_goal");ChannelMotionInfo motion("/MOTION_INFO");
 ChannelNavCmd cmd([](const drdds::msg::NavCmd* m){double v[]{seconds(),m->data().x_vel(),m->data().y_vel(),m->data().yaw_vel()};packet(101,v,sizeof(v));},"/NAV_CMD");
 ChannelPlannerStatus status([](const drdds::msg::PlannerStatus* m){int32_t v[]{m->data().status(),m->data().warning_state()};packet(102,v,sizeof(v));},"/PLANNER_STATUS");
 ChannelPointCloud2 filtered([](const sensor_msgs::msg::PointCloud2* m){
  sensor_msgs::msg::PointCloud2 copy=*m;double timing[3];
  {std::lock_guard<std::mutex> g(cloud_mutex);if(!cloud_inflight)return;
   // NOS pcl_pass_grid 0xc6c78-0xc6d08 accidentally derives nanoseconds from
   // integer seconds, always producing zero. With exactly one frame in flight,
   // restore its acquisition stamp only on the ROS output. Native geometry and
   // the messages delivered to localPlanner remain completely untouched.
   timing[0]=m->header().stamp().sec()+m->header().stamp().nanosec()*1e-9;
   timing[1]=submitted_time.sec()+submitted_time.nanosec()*1e-9;timing[2]=(seconds()-submitted_at)*1000.;
   copy.header().stamp()=submitted_time;
  }
  emit<sensor_msgs::msg::PointCloud2PubSubType>(103,&copy);packet(106,timing,sizeof(timing));cloud_inflight=false;
 },"/NAV_POINTS",0,true);
 ChannelPath selected([](const nav_msgs::msg::Path* m){emit<nav_msgs::msg::PathPubSubType>(104,m);},"/local_path");
 ChannelPoseStamped target([](const geometry_msgs::msg::PoseStamped* m){emit<geometry_msgs::msg::PoseStampedPubSubType>(105,m);},"/local_goal_baselink");
 ChannelClientSetParam params("/set_service","eth0");
 ChannelClientPoseStampedToInt32 goal("/GOAL_PLANNER","eth0");
 ChannelClientStdSrvInt32 cancel("/CANCEL_NAV_PLANNER","eth0");
 std::thread cloud_worker([&]{for(;;){
  std::shared_ptr<sensor_msgs::msg::PointCloud2> m;
  if(!cloud_inflight&&cloud.GetMatchedCount()>0){std::lock_guard<std::mutex> g(cloud_mutex);m.swap(pending_cloud);}
  if(m){{std::lock_guard<std::mutex> g(cloud_mutex);submitted_time=m->header().stamp();submitted_at=seconds();cloud_inflight=true;}cloud.Write(m.get());}
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
 }});cloud_worker.detach();
 packet(100,nullptr,0);
 try{for(;;){uint32_t h[2];if(!io(h,8,false))break;if(h[0]>16000000)throw std::runtime_error("packet too large");std::vector<uint8_t> b(h[0]);if(!io(b.data(),b.size(),false))break;
 switch(h[1]){
 case 1:{auto m=decode<nav_msgs::msg::OdometryPubSubType>(b);odom.Write(&m);auto t=m.twist().twist();drdds::msg::MotionInfo info;info.data().vel_x(t.linear().x());info.data().vel_y(t.linear().y());info.data().vel_yaw(t.angular().z());motion.Write(&info);break;}
 case 2:{auto m=std::make_shared<sensor_msgs::msg::PointCloud2>(decode<sensor_msgs::msg::PointCloud2PubSubType>(b));std::lock_guard<std::mutex> g(cloud_mutex);pending_cloud=std::move(m);break;}
 case 3:{auto m=decode<nav_msgs::msg::PathPubSubType>(b);path.Write(&m);break;}
 case 4:{auto m=decode<geometry_msgs::msg::PoseStampedPubSubType>(b);local.Write(&m);break;}
 case 5:{drdds::srv::PoseStampedToInt32_Request req;req.pose(decode<geometry_msgs::msg::PoseStampedPubSubType>(b));drdds::srv::PoseStampedToInt32_Response res;int32_t v=goal.Call(req,res,1000)&&res.result();packet(110,&v,4);break;}
 case 6:{drdds::srv::StdSrvInt32_Request req;req.command(1);drdds::srv::StdSrvInt32_Response res;int32_t v=cancel.Call(req,res,1000)?res.result():-1;packet(111,&v,4);break;}
 case 7:{drdds::srv::SetParam_Request req;drdds::srv::SetParam_Response res;
 req.directLine_mode(!b.empty()&&b[0]==1);req.maxSpeedX(1.5);req.maxSpeedY(.6);req.maxTheta(1.0);req.barking_deceleration(1.0);req.xy_tolerance(.1);req.yaw_tolerance(.1);req.stop_distance(.7);req.judge_close(0);req.backward_mode(0);req.dl_min_yaw(.7);req.dl_min_x(.25);req.dl_min_y(.25);req.dl_line_yaw_min(.2);
 int32_t v=params.Call(req,res,1000)&&res.is_success();packet(112,&v,4);break;}
 default:throw std::runtime_error("unknown packet");}
 }}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;_exit(2);}
 // Process teardown closes all channels; no command can leave the isolated network.
 _exit(0);
}
