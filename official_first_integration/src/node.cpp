#include "first.hpp"
#include "world.hpp"
#include "near_filter.hpp"
#include "self_shadow.hpp"
#include "route.hpp"
#include "terminal.hpp"
#include "posture.hpp"
#include "pose_math.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <cstring>
#include <iomanip>
#include <filesystem>
using Clock=std::chrono::steady_clock;
using Cloud=sensor_msgs::msg::PointCloud2;
using Odom=nav_msgs::msg::Odometry;
using Path=nav_msgs::msg::Path;
static double steady(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
static double stamp(const builtin_interfaces::msg::Time&t){return t.sec+t.nanosec*1e-9;}
static double yaw(Eigen::Quaterniond q){auto r=q.toRotationMatrix();return std::atan2(r(1,0),r(0,0));}
static Eigen::Vector3d xyz(const geometry_msgs::msg::Point&p){return {p.x,p.y,p.z};}
class Node:public rclcpp::Node {
 struct Snapshot {std::shared_ptr<const first::Evidence> evidence=std::make_shared<first::Evidence>();std::vector<first::Pose> route;first::Pose body;double height=0,odom_at=0,cloud_at=0,cloud_stamp=0,alignment_at=0,motion_at=0,perception_ms=0;int gait=0,motion_state=0;bool posture_ready=false;std::array<double,4> perception_stages{},cloud_pipeline{};size_t shadow_rejected=0;uint64_t generation=0,route_revision=0;};
 struct Result {bool continuity=false;first::Selection selected;std::vector<first::Pose> path;std::string reason="waiting_inputs",phase="follow",near_body="[]";double at=0,compute_ms=0,source_stamp=0,route_progress=0,route_remaining=0;std::array<double,4> plan_stages{};uint64_t generation=0;};
 public:
 Node():rclcpp::Node("official_first"),library_(declare_parameter<std::string>("library","")){
  auto reference=declare_parameter<std::string>("terrain","");
  if(!reference.empty())terrain_.load(reference); // Optional height reference, never a support certificate.
  frame_=declare_parameter<std::string>("world_frame","se2_scan_map");
  auto trace=declare_parameter<std::string>("hit_trace","");
  hit_trace_path_=trace;
  if(!trace.empty()){hit_trace_.open(trace);if(!hit_trace_)throw std::runtime_error("cannot open hit trace");}
  auto latest=rclcpp::SensorDataQoS().keep_last(1);
  command_=create_publisher<geometry_msgs::msg::TwistStamped>("/first/cmd_vel_stamped",latest);
  path_pub_=create_publisher<Path>("/first/local_path",1);status_=create_publisher<std_msgs::msg::String>("/first/status",10);
  aligned_=create_publisher<Odom>("/first/aligned_odom",latest);voxels_=create_publisher<Cloud>("/first/occupancy",latest);
  planning_cloud_=create_publisher<Cloud>("/first/planning_obstacles",latest);
  gait_pub_=create_publisher<std_msgs::msg::Int32>("/first/request_gait",1);
  timing_=create_publisher<std_msgs::msg::Float64MultiArray>("/first/control_timing",100);
  transform_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("/se2_scan/map_transform",latest,[this](geometry_msgs::msg::PoseStamped::SharedPtr m){
   auto q=m->pose.orientation;Eigen::Quaterniond r(q.w,q.x,q.y,q.z);auto t=xyz(m->pose.position);double age=now().seconds()-stamp(m->header.stamp);
   std::lock_guard<std::mutex>g(lock_);
   if(m->header.frame_id!=frame_||!t.allFinite()||!r.coeffs().allFinite()||r.norm()<.5||age<-.1||age>1.5){state_.alignment_at=0;++state_.generation;return;}
   r.normalize();if((translation_-t).norm()>.02||rotation_.angularDistance(r)>.01){state_.evidence=std::make_shared<first::Evidence>();state_.cloud_at=0;++state_.generation;}
   translation_=t;rotation_=r;state_.alignment_at=steady();
  });
  odom_sub_=create_subscription<Odom>("/nvblox_lio/odom",rclcpp::SensorDataQoS().keep_last(200),[this](Odom::SharedPtr m){
   auto p=m->pose.pose.position;auto q=m->pose.pose.orientation;se2_deskew::Pose raw{stamp(m->header.stamp),{p.x,p.y,p.z},{q.w,q.x,q.y,q.z}};
   if(m->header.frame_id!="nvblox_odom"||m->child_frame_id!="base_link_dog"||!raw.p.allFinite()||!raw.q.coeffs().allFinite()||raw.q.norm()<.5||std::abs(now().seconds()-raw.stamp)>.35)return;
   raw.q.normalize();std::lock_guard<std::mutex>g(lock_);
   if(!poses_.empty()&&(raw.stamp<poses_.back().stamp-.05||(raw.p-poses_.back().p).norm()>.3)){poses_.clear();state_.evidence=std::make_shared<first::Evidence>();state_.cloud_at=0;state_.alignment_at=0;++state_.generation;}
   if(!poses_.empty()&&raw.stamp<=poses_.back().stamp)return;
   poses_.push_back(raw);while(poses_.size()>2&&poses_[1].stamp<raw.stamp-2.)poses_.pop_front();
   if(steady()-state_.alignment_at>1.5)return;
   auto w=rotation_*raw.p+translation_;auto rq=rotation_*raw.q;
   state_.body={w.x(),w.y(),w.z(),yaw(rq)};state_.odom_at=steady();
   m->header.frame_id=frame_;m->pose.pose.position.x=w.x();m->pose.pose.position.y=w.y();m->pose.pose.position.z=w.z();
   m->pose.pose.orientation.w=rq.w();m->pose.pose.orientation.x=rq.x();m->pose.pose.orientation.y=rq.y();m->pose.pose.orientation.z=rq.z();aligned_->publish(*m);
  });
  cloud_sub_=create_subscription<Cloud>("/LIDAR/POINTS_NX",latest,[this](Cloud::SharedPtr m){std::lock_guard<std::mutex>g(pending_lock_);pending_=m;pending_received_=steady();pending_ros_received_=now().seconds();});
  route_sub_=create_subscription<Path>("/se2_global/path",rclcpp::QoS(1).reliable().transient_local(),[this](Path::SharedPtr m){
   std::lock_guard<std::mutex>g(lock_);state_.route.clear();++state_.generation;++state_.route_revision;
   if(m->header.frame_id!=frame_)return;
   for(auto&p:m->poses){auto q=p.pose.orientation;auto v=p.pose.position;Eigen::Quaterniond r(q.w,q.x,q.y,q.z);
    if(!std::isfinite(v.x+v.y+v.z)||!r.coeffs().allFinite()||r.norm()<.5){state_.route.clear();return;}state_.route.push_back({v.x,v.y,v.z,yaw(r.normalized())});}
  });
  motion_sub_=create_subscription<std_msgs::msg::Float64MultiArray>("/first/robot_state",latest,[this](std_msgs::msg::Float64MultiArray::SharedPtr m){
   if(m->data.size()!=6&&m->data.size()!=7)return;
   std::lock_guard<std::mutex>g(lock_);auto&a=m->data;
   if(!std::all_of(a.begin(),a.end(),[](double v){return std::isfinite(v);})||a[0]<.2||a[0]>1.||a[3]>=1.2||a[4]>=.35){state_.motion_at=0;return;}
   double acquired=a.size()==7?a[6]:now().seconds()-a[4];
   double age=now().seconds()-acquired;
   if(age<-.1||age>=.35){state_.motion_at=0;return;}
   bool was_ready=state_.posture_ready;
   state_.posture_ready=posture_.observe(int(a[1]),int(a[5]),a[0],acquired);
   if(was_ready!=state_.posture_ready){++state_.generation;state_.cloud_at=0;}
   state_.height=a[0];state_.gait=int(a[1]);state_.motion_state=int(a[2]);state_.motion_at=steady()-std::max(0.,age);
  });
  perception_=std::thread([this]{while(running_){drain();std::this_thread::sleep_for(std::chrono::milliseconds(5));}});
  planner_=std::thread([this]{plan_loop();});control_=std::thread([this]{control_loop();});
 }
 ~Node(){running_=false;if(perception_.joinable())perception_.join();if(planner_.joinable())planner_.join();if(control_.joinable())control_.join();}
 private:
 template<class T>static T read(const uint8_t*p){T out;std::memcpy(&out,p,sizeof(out));return out;}
 static int field(const Cloud&c,const std::string&name,int type,int size){for(auto&f:c.fields)if(f.name==name&&f.datatype==type&&f.count==1&&f.offset+size<=c.point_step)return f.offset;throw std::runtime_error("bad field "+name);}
 void discard(const Cloud::SharedPtr&m){std::lock_guard<std::mutex>g(pending_lock_);if(pending_==m)pending_.reset();}
 void drain(){
  Cloud::SharedPtr m;double received=0,ros_received=0;{std::lock_guard<std::mutex>g(pending_lock_);m=pending_;received=pending_received_;ros_received=pending_ros_received_;}
  if(!m)return;
  double perception_start=steady();
  try{
   if(m->header.frame_id!="lidar_link"&&m->header.frame_id!="base_link_dog")throw std::runtime_error("cloud frame");
   if(m->is_bigendian||uint64_t(m->width)*m->point_step>m->row_step||uint64_t(m->height)*m->row_step>m->data.size())throw std::runtime_error("cloud layout");
   int fx=field(*m,"x",7,4),fy=field(*m,"y",7,4),fz=field(*m,"z",7,4),ft=field(*m,"timestamp",8,8),fr=field(*m,"ring",4,2);
   int fi=-1;for(auto&f:m->fields)if(f.name=="intensity"&&f.datatype==7&&f.count==1&&f.offset+4<=m->point_step)fi=f.offset;
   std::deque<se2_deskew::Pose> poses;Eigen::Quaterniond rotation;Eigen::Vector3d translation;uint64_t gen;double not_before,height,motion_age;int gait;
   {std::lock_guard<std::mutex>g(lock_);if(steady()-state_.alignment_at>1.5||steady()-state_.motion_at>.35||!state_.posture_ready)return;poses=poses_;rotation=rotation_;translation=translation_;gen=state_.generation;not_before=posture_.not_before;height=state_.height;gait=state_.gait;motion_age=steady()-state_.motion_at;}
   double lo=stamp(m->header.stamp),hi=lo;
   for(size_t i=0;i<m->height;++i)for(size_t j=0;j<m->width;++j){double t=read<double>(m->data.data()+i*m->row_step+j*m->point_step+ft);if(std::isfinite(t)&&std::abs(t-stamp(m->header.stamp))<=.25){lo=std::min(lo,t);hi=std::max(hi,t);}}
   if(lo<not_before){discard(m);return;}
   if(now().seconds()-hi>.35)throw std::runtime_error("old cloud");
   if(poses.empty()||poses.back().stamp<hi)return;
   if(poses.front().stamp>lo)throw std::runtime_error("unbracketed cloud");
   discard(m);int n=std::max(1,int(std::ceil((hi-lo)/.0005)));double dt=(hi-lo)/n;
   std::vector<Eigen::Isometry3d> transforms;
   for(int i=0;i<=n;++i){auto p=se2_deskew::interpolate(poses,std::min(hi,lo+i*dt));Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.linear()=(rotation*p.q).toRotationMatrix();t.translation()=rotation*p.p+translation;transforms.push_back(t);}
   std::vector<std::pair<first::Point,first::Point>> raw_rays,rays;raw_rays.reserve(size_t(m->width)*m->height);rays.reserve(size_t(m->width)*m->height);
   struct Return {Eigen::Vector3d raw,body;double stamp;int ring;float intensity;};std::vector<Return> returns;returns.reserve(size_t(m->width)*m->height);
   size_t sensor_returns[2]={0,0};
   size_t shadow_rejected=0;
   bool tracing=hit_trace_.is_open();std::ostringstream hits;hits<<std::setprecision(15);bool first_hit=true;
   for(size_t row=0;row<m->height;++row)for(size_t col=0;col<m->width;++col){auto p=m->data.data()+row*m->row_step+col*m->point_step;
    Eigen::Vector3d v(read<float>(p+fx),read<float>(p+fy),read<float>(p+fz));double t=read<double>(p+ft);int ring=read<uint16_t>(p+fr);if(ring>=192)throw std::runtime_error("unverified ring");
    if(!v.allFinite()||!std::isfinite(t)||t<lo||t>hi)continue;
    if(std::abs(v.x())<.41&&std::abs(v.y())<.253&&v.z()>-.15&&v.z()<.35)continue;
    if(first::chassis_bottom_shadow({v.x(),v.y(),v.z()},ring)){++shadow_rejected;continue;}
    int i=dt>0?std::clamp(int(std::llround((t-lo)/dt)),0,n):0;auto w=transforms[i]*v;auto o=transforms[i]*Eigen::Vector3d(ring<96?.32028:-.32028,0,-.013);
    first::Point hit{w.x(),w.y(),w.z()},origin{o.x(),o.y(),o.z()};
    raw_rays.emplace_back(origin,hit);returns.push_back({v,transforms[i].translation(),t,ring,fi>=0?read<float>(p+fi):-1.f});++sensor_returns[ring/96];
   }
   double decoded_at=steady();
   auto keep=first::near_support(raw_rays,near_scratch_);size_t rejected=0;
   for(int sensor=0;sensor<2;++sensor)seen_[sensor].reset(sensor_returns[sensor]);
   for(size_t j=0;j<raw_rays.size();++j){
    auto [origin,hit]=raw_rays[j];auto &ret=returns[j];auto v=ret.raw;
    if(!keep[j])++rejected;
    // Rejected endpoints contribute neither occupied cells nor clearing rays.
    if(keep[j]&&seen_[ret.ring/96].insert(first::Evidence::key(hit)).second)rays.emplace_back(origin,hit);
    if(tracing&&std::abs(v.x())<1.2&&std::abs(v.y())<.6&&v.z()>-.8&&v.z()<.05){
     // Preserve one representative per voxel/sensor/outcome, including rejects.
     auto k=first::Evidence::key(hit);
     if(trace_seen_[keep[j]?ret.ring/96:2+ret.ring/96].insert(k).second){
      if(!first_hit)hits<<",";
      first_hit=false;auto b=ret.body;
      hits<<"["<<ret.stamp<<","<<ret.ring<<","<<v.x()<<","<<v.y()<<","<<v.z()<<","<<hit.x<<","<<hit.y<<","<<hit.z<<","<<k.x<<","<<k.y<<","<<k.z<<","<<b.x()<<","<<b.y()<<","<<b.z()<<","<<int(keep[j])<<","<<(std::isfinite(ret.intensity)?ret.intensity:-1.f)<<"]";
     }
    }
   }
   for(auto &seen:trace_seen_)seen.clear();
   if(rays.empty())throw std::runtime_error("empty cloud");
   Cloud visual;visual.header.frame_id=frame_;visual.header.stamp=m->header.stamp;
   std::vector<float> occupied;std::shared_ptr<const first::Evidence> previous;first::Point body;
   {std::lock_guard<std::mutex>g(lock_);if(gen!=state_.generation)return;previous=state_.evidence;body={state_.body.x,state_.body.y,state_.body.z};}
   double filtered_at=steady();
   auto evidence=std::make_shared<first::Evidence>(*previous);evidence->ingest(rays,body,steady());
   double ingested_at=steady();
   for(auto p:evidence->occupied()){occupied.insert(occupied.end(),{float(p.x),float(p.y),float(p.z)});}
   {std::lock_guard<std::mutex>g(lock_);if(gen!=state_.generation)return;state_.evidence=evidence;state_.cloud_at=steady();state_.cloud_stamp=hi;state_.perception_ms=(steady()-perception_start)*1000;}
   if(tracing){
    hit_trace_<<std::setprecision(15)<<"{\"schema\":2,\"stamp\":"<<hi<<",\"height\":"<<height<<",\"motion_age_s\":"<<motion_age<<",\"gait\":"<<gait<<",\"raw_returns\":"<<raw_rays.size()<<",\"shadow_rejected\":"<<shadow_rejected<<",\"rejected_near\":"<<rejected<<",\"hits\":["<<hits.str()<<"]}\n";hit_trace_.flush();
    if(!hit_trace_){hit_trace_.close();RCLCPP_WARN(get_logger(),"Near-body hit trace write failed");}
    else if(hit_trace_.tellp()>std::streampos(64*1024*1024)){
     hit_trace_.close();std::error_code ec;
     std::filesystem::remove(hit_trace_path_+".2",ec);
     std::filesystem::rename(hit_trace_path_+".1",hit_trace_path_+".2",ec);
     std::filesystem::rename(hit_trace_path_,hit_trace_path_+".1",ec);
     if(!ec)hit_trace_.open(hit_trace_path_,std::ios::trunc);
    }
    // Keep the bounded rolling trace through transient blocks: the first
    // rejected plan need not be the later persistent obstruction.
   }
   sensor_msgs::PointCloud2Modifier mod(visual);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(occupied.size()/3);
   sensor_msgs::PointCloud2Iterator<float> x(visual,"x"),y(visual,"y"),z(visual,"z");for(size_t i=0;i<occupied.size();i+=3,++x,++y,++z){*x=occupied[i];*y=occupied[i+1];*z=occupied[i+2];}voxels_->publish(visual);
   {std::lock_guard<std::mutex>g(lock_);if(gen==state_.generation){state_.perception_stages={(decoded_at-perception_start)*1000,(filtered_at-decoded_at)*1000,(ingested_at-filtered_at)*1000,(steady()-ingested_at)*1000};state_.shadow_rejected=shadow_rejected;state_.cloud_pipeline={(hi-lo)*1000,(ros_received-hi)*1000,(perception_start-received)*1000,(ingested_at-perception_start)*1000};}}
  }catch(const std::exception&e){discard(m);std::lock_guard<std::mutex>g(lock_);state_.cloud_at=0;RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"%s",e.what());}
 }
 std::vector<first::Point> obstacle_points(const Snapshot&s,first::Pose base,double reference_offset,int component)const{
  std::vector<first::Point> out;
  for(auto p:s.evidence->occupied()){
   if(p.z<base.z-.25||p.z>base.z+1.4)continue;
   auto k=first::Evidence::key(p);k.z=0;
   auto support=terrain_.at_voxel_center(k,base.z-reference_offset,component,.8);
   if(p.z<=(support?support->z+reference_offset:base.z)+.10)continue;
   out.push_back(p);
  }
  return out;
 }
 void show_obstacles(const std::vector<first::Point>&points){
  double t=steady();if(t-last_obstacle_visual_<.1)return;last_obstacle_visual_=t;
  Cloud msg;msg.header.frame_id=frame_;msg.header.stamp=now();
  sensor_msgs::PointCloud2Modifier mod(msg);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float>x(msg,"x"),y(msg,"y"),z(msg,"z");
  for(auto p:points){*x=p.x;*y=p.y;*z=p.z;++x;++y;++z;}planning_cloud_->publish(msg);
 }
 Result plan(const Snapshot&s){
  Result r;r.generation=s.generation;r.source_stamp=s.cloud_stamp;double t=steady();
  if(s.route.empty()){
   if(t-last_obstacle_visual_>=.1){
    if(t-s.cloud_at<.35&&t-s.motion_at<.35&&t-s.alignment_at<1.5&&s.posture_ready){
     auto base=s.body;base.z-=s.height;auto ground=terrain_.at(base.x,base.y,base.z,-1,.16);
     show_obstacles(obstacle_points(s,base,ground?base.z-ground->z:0.,ground?ground->component:-1));
    }else show_obstacles({});
   }
   r.reason="idle";return r;
  }
  if(t-s.cloud_at>.35||t-s.motion_at>.35||t-s.alignment_at>1.5||!s.posture_ready)show_obstacles({});
  if(t-s.odom_at>.2){r.reason="stale_odometry";return r;}
  if(t-s.motion_at>.35){r.reason="stale_robot_state";return r;}
  if(t-s.alignment_at>1.5){r.reason="stale_localization";return r;}
  auto base=s.body;base.z-=s.height;
  // Missing saved ground is not evidence of a physical hole. Retain available
  // heights only as reference; odometry minus reported height is the fallback.
  auto ground=terrain_.at(base.x,base.y,base.z,-1,.16);
  // The live body/height pair is the ground datum. Saved surfaces can differ
  // vertically after registration; preserve their relative relief, never move
  // the measured ground to an old Mesh (which turns low returns into obstacles).
  double reference_offset=ground?base.z-ground->z:0.;
  int component=ground?ground->component:-1;
  // Localization/posture invalidates commands and cloud evidence, not progress
  // along an unchanged route in the fixed map frame.
  if(route_generation_!=s.route_revision){route_.reset(s.route);route_generation_=s.route_revision;}
  first::RouteTarget rt;try{rt=route_.target(base);}catch(const std::exception&){r.reason="route_disconnected";return r;}
  r.route_progress=rt.progress;r.route_remaining=rt.remaining;
  auto end=s.route.back();double final_d=std::hypot(end.x-base.x,end.y-base.y);
  if(rt.remaining<=.5&&final_d<=.10&&std::abs(end.z-base.z)<.15&&std::abs(first::wrap(end.yaw-base.yaw))<.1){r.reason="arrived";return r;}
  // Lead includes the request deadline and braking at the normal 1.5 m/s cap.
  double switch_lead=std::max(2.5,1.5*3.+1.5*1.5/2.+.5);
  bool stairs=rt.stairs_ahead<=switch_lead,on_stairs=rt.on_stairs;double stair_distance=rt.stairs_ahead;
  std_msgs::msg::Int32 request;request.data=(stairs||on_stairs)?first::Gait::stairs:first::Gait::flat;gait_pub_->publish(request);
  bool gait_ready=s.gait==first::Gait::stairs;
  if(s.gait!=first::Gait::flat&&!gait_ready){r.reason="waiting_gait";return r;}
  if(!s.posture_ready){r.reason="posture_settling";return r;}
  if(t-s.cloud_at>.35){r.reason="stale_cloud";return r;}
  if((on_stairs||stair_distance<.65)&&!gait_ready){r.reason="waiting_stair_confirmation";return r;}
  auto target_pose=rt.goal;double dx=target_pose.x-base.x,dy=target_pose.y-base.y,c=std::cos(base.yaw),si=std::sin(base.yaw);
  first::Pose goal{c*dx+si*dy,-si*dx+c*dy,0,first::wrap(target_pose.yaw-base.yaw)};
  double classify_at=steady();
  auto obstacles=obstacle_points(s,base,reference_offset,component);show_obstacles(obstacles);
  double classified_at=steady();
  std::vector<int> cells;
  for(auto p:obstacles){double x=p.x-base.x,y=p.y-base.y;cells.push_back(first::Library::cell(c*x+si*y,-si*x+c*y));}
  std::ostringstream near;near<<"[";size_t near_count=0;
  for(auto p:obstacles)if(first::intersects(base,p,.9)&&near_count<8){if(near_count++)near<<",";double dx=p.x-base.x,dy=p.y-base.y;near<<"["<<c*dx+si*dy<<","<<-si*dx+c*dy<<","<<p.z-base.z<<"]";}
  near<<"]";r.near_body=near.str();
  auto penalties=library_.penalties(cells);first::Limits limits=(stairs||on_stairs||s.gait==first::Gait::stairs)?first::Limits{.4,.3,.4,.4}:first::Limits{};
  first::ObstacleIndex obstacle_index(obstacles);
  double indexed_at=steady();
  std::vector<first::Pose> candidate;
  auto check=[&](const first::Primitive&p){
   // Validate the whole nominal second AND enough additional braking distance.
   double speed=std::hypot(p.velocity.x,p.velocity.y);candidate=first::sweep(p,base,std::max(0.,speed/2.+.2-1.));double prev=base.z;
   for(auto&pose:candidate){
    auto reference=terrain_.at(pose.x,pose.y,prev-reference_offset,component);
    pose.z=reference?reference->z+reference_offset:prev;
    // No saved-map center/footprint support veto. Stair entry is gated above
    // from the ASA route and actual confirmed gait, independently of Mesh holes.
    if(obstacle_index.collides(pose,.9))return false;
    prev=pose.z;
   }return true;
  };
  first::Velocity previous;bool continuity=false;
  {std::lock_guard<std::mutex>g(lock_);continuity=last_sent_valid_&&last_sent_generation_==s.generation&&steady()-last_sent_at_<.12;
   previous=last_sent_;}
  r.continuity=continuity;r.selected=first::select(library_,goal,penalties,limits,check,continuity?&previous:nullptr);
  if(r.selected.id>=0){
   double ex=end.x-base.x,ey=end.y-base.y;
   auto cmd=first::terminal(r.selected.command,{c*ex+si*ey,-si*ex+c*ey,end.z-base.z,first::wrap(end.yaw-base.yaw)},rt.remaining,limits);
   if(cmd.phase==first::Phase::Arrived){r.selected.command={};r.reason="arrived";return r;}
   r.phase=cmd.phase==first::Phase::Approach?"approach":cmd.phase==first::Phase::Align?"align":"follow";
   // CmdCalculate changes the selected primitive near the final goal. Validate
   // the actual resulting command; never use the old candidate's certificate.
   bool valid=cmd.phase==first::Phase::Follow||check(first::command_primitive(cmd.velocity));
   if(valid){r.selected.command=cmd.velocity;r.path=candidate;r.reason="tracking";}
   else {r.selected.command={};r.reason="terminal_blocked";}
  }else r.reason="blocked";
  r.plan_stages={(classify_at-t)*1000,(classified_at-classify_at)*1000,(indexed_at-classified_at)*1000,(steady()-indexed_at)*1000};
  return r;
 }
 void plan_loop(){while(running_){auto next=Clock::now()+std::chrono::milliseconds(33);Snapshot s;{std::lock_guard<std::mutex>g(lock_);s=state_;}auto t=steady();auto r=plan(s);r.at=t;r.compute_ms=(steady()-t)*1000;{std::lock_guard<std::mutex>g(lock_);if(r.generation==state_.generation)result_=std::move(r);}std::this_thread::sleep_until(next);}}
 void control_loop(){auto next=Clock::now();double last=steady(),report=0;while(running_){next+=std::chrono::microseconds(33333);double t=steady();Result r;bool valid;double odom_age,cloud_age,robot_age,localization_age,perception_ms,result_age,source_age;std::array<double,4> perception_stages{},cloud_pipeline{};size_t shadow_rejected=0;std::string hold;
  {std::lock_guard<std::mutex>g(lock_);r=result_;odom_age=t-state_.odom_at;cloud_age=t-state_.cloud_at;robot_age=t-state_.motion_at;localization_age=t-state_.alignment_at;perception_ms=state_.perception_ms;perception_stages=state_.perception_stages;cloud_pipeline=state_.cloud_pipeline;shadow_rejected=state_.shadow_rejected;result_age=t-r.at;source_age=now().seconds()-r.source_stamp;hold=r.generation!=state_.generation?"generation_changed":r.reason!="tracking"?r.reason:result_age>=.12?"expired_plan":odom_age>=.2?"stale_odometry":cloud_age>=.35?"stale_cloud_update":source_age>=.35?"stale_cloud_acquisition":robot_age>=.35?"stale_robot_state":localization_age>=1.5?"stale_localization":"none";valid=hold=="none";}
  geometry_msgs::msg::TwistStamped cmd;cmd.header.stamp=now();cmd.header.frame_id="base_link_dog";if(valid){cmd.twist.linear.x=r.selected.command.x;cmd.twist.linear.y=r.selected.command.y;cmd.twist.angular.z=r.selected.command.w;}command_->publish(cmd);
  {std::lock_guard<std::mutex>g(lock_);last_sent_={cmd.twist.linear.x,cmd.twist.linear.y,cmd.twist.angular.z};last_sent_at_=t;last_sent_generation_=r.generation;last_sent_valid_=valid&&r.phase=="follow";}
  std_msgs::msg::Float64MultiArray timing;timing.data={t,(t-last)*1000,r.compute_ms,now().seconds()-r.source_stamp,valid?1.:0.};timing_->publish(timing);last=t;
  if(t-report>.1){std_msgs::msg::String status;std::ostringstream out;out<<"{\"state\":\""<<r.reason<<"\",\"phase\":\""<<r.phase<<"\",\"odom_age_s\":"<<odom_age<<",\"cloud_age_s\":"<<cloud_age<<",\"robot_age_s\":"<<robot_age<<",\"localization_age_s\":"<<localization_age<<",\"perception_ms\":"<<perception_ms<<",\"perception_stages_ms\":["<<perception_stages[0]<<","<<perception_stages[1]<<","<<perception_stages[2]<<","<<perception_stages[3]<<"],\"cloud_pipeline_ms\":["<<cloud_pipeline[0]<<","<<cloud_pipeline[1]<<","<<cloud_pipeline[2]<<","<<cloud_pipeline[3]<<"],\"self_shadow_rejected\":"<<shadow_rejected<<",\"command_valid\":"<<(valid?"true":"false")<<",\"hold_reason\":\""<<hold<<"\",\"plan_age_s\":"<<result_age<<",\"source_cloud_age_s\":"<<source_age<<",\"path_id\":"<<r.selected.id<<",\"route_progress_m\":"<<r.route_progress<<",\"route_remaining_m\":"<<r.route_remaining<<",\"selected_velocity\":["<<r.selected.command.x<<","<<r.selected.command.y<<","<<r.selected.command.w<<"],\"continuity_active\":"<<(r.continuity?"true":"false")<<",\"compute_ms\":"<<r.compute_ms<<",\"plan_stages_ms\":["<<r.plan_stages[0]<<","<<r.plan_stages[1]<<","<<r.plan_stages[2]<<","<<r.plan_stages[3]<<"],\"near_body_obstacles_xyz_m\":"<<r.near_body<<",\"hard_rejected\":"<<r.selected.hard_rejected<<",\"geometry_rejected\":"<<r.selected.terrain_rejected<<"}";status.data=out.str();status_->publish(status);
   Path p;p.header.frame_id=frame_;p.header.stamp=now();for(auto q:r.path){geometry_msgs::msg::PoseStamped m;m.header=p.header;m.pose.position.x=q.x;m.pose.position.y=q.y;m.pose.position.z=q.z+.04;m.pose.orientation.w=std::cos(q.yaw/2);m.pose.orientation.z=std::sin(q.yaw/2);p.poses.push_back(m);}path_pub_->publish(p);report=t;}
  if(Clock::now()>next+std::chrono::milliseconds(33))next=Clock::now();
  std::this_thread::sleep_until(next);
 }}
 first::Velocity last_sent_;double last_sent_at_=0;uint64_t last_sent_generation_=0;bool last_sent_valid_=false;
 first::Route route_;uint64_t route_generation_=uint64_t(-1);
 first::NearSupportScratch near_scratch_;first::ScanKeys<uint8_t> seen_[2];
 first::Library library_;first::Terrain terrain_;first::PostureGate posture_;Snapshot state_;Result result_;std::mutex lock_,pending_lock_;std::atomic<bool>running_{true};std::thread planner_,control_,perception_;
 double last_obstacle_visual_=0;std::ofstream hit_trace_;std::string hit_trace_path_;std::unordered_set<first::Key,first::Hash> trace_seen_[4];std::string frame_;Eigen::Quaterniond rotation_=Eigen::Quaterniond::Identity();Eigen::Vector3d translation_=Eigen::Vector3d::Zero();std::deque<se2_deskew::Pose>poses_;Cloud::SharedPtr pending_;double pending_received_=0,pending_ros_received_=0;
 rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_;rclcpp::Publisher<Path>::SharedPtr path_pub_;rclcpp::Publisher<Odom>::SharedPtr aligned_;rclcpp::Publisher<Cloud>::SharedPtr voxels_,planning_cloud_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_;rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gait_pub_;
 rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr transform_sub_;rclcpp::Subscription<Odom>::SharedPtr odom_sub_;rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;rclcpp::Subscription<Path>::SharedPtr route_sub_;rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motion_sub_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Node>());}catch(const std::exception&e){fprintf(stderr,"FIRST: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
