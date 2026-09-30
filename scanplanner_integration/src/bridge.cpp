// Independent perception / control process. The optimizer never runs here.
#include "pose_math.hpp"
#include <plan_env/grid_map.h>
#include <bspline_opt/uniform_bspline.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <cstring>

using Cloud=sensor_msgs::msg::PointCloud2;
using Odom=nav_msgs::msg::Odometry;
using Path=nav_msgs::msg::Path;
using Spline=scan_planner::UniformBspline;
using Clock=std::chrono::steady_clock;
using se2_deskew::Pose;
static double steady(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
static double stamp(const builtin_interfaces::msg::Time& t){return t.sec+t.nanosec*1e-9;}
static double wrap(double a){return std::remainder(a,2*M_PI);}
static Eigen::Vector3d xyz(const geometry_msgs::msg::Point&p){return {p.x,p.y,p.z};}
class Bridge:public rclcpp::Node {
public:
 Bridge():Node("scan_bridge") {
  frame_=declare_parameter<std::string>("world_frame","nvblox_odom");
  use_alignment_=declare_parameter<bool>("use_map_alignment",false);
  hz_=declare_parameter<double>("control_hz",30.);
  max_x_=declare_parameter<double>("max_forward_speed",.50);
  max_y_=declare_parameter<double>("max_lateral_speed",.25);
  max_yaw_=declare_parameter<double>("max_yaw_speed",.60);
  lookahead_=declare_parameter<double>("control_lookahead_s",.50);
  if(!std::isfinite(max_x_)||!std::isfinite(max_y_)||!std::isfinite(max_yaw_)||max_x_<=0||max_x_>.50||max_y_<=0||max_y_>.25||max_yaw_<=0||max_yaw_>.60)throw std::runtime_error("invalid navigation velocity limits");
  // Keep the target inside the one-second swept-trajectory collision check.
  if(!std::isfinite(lookahead_)||lookahead_<.05||lookahead_>.75)throw std::runtime_error("control lookahead must be in [0.05, 0.75] seconds");
  if(hz_<20 || hz_>100)throw std::runtime_error("control_hz must be in [20,100]");
  map_.initMap(this);
  auto latest=rclcpp::SensorDataQoS().keep_last(1);
  alignment_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("map_transform",latest,[this](geometry_msgs::msg::PoseStamped::SharedPtr m){
    const auto&q=m->pose.orientation;Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);auto translation=xyz(m->pose.position);
    double age=now().seconds()-stamp(m->header.stamp);
    if(m->header.frame_id!=frame_||!translation.allFinite()||!rotation.coeffs().allFinite()||rotation.norm()<.5||age<-.1||age>1.5){alignment_time_=0;blocked_=true;return;}
    map_rotation_=rotation.normalized();map_translation_=translation;alignment_time_=steady();
  });
  cloud_sub_=create_subscription<Cloud>("raw_cloud",latest,[this](Cloud::SharedPtr m){
    if(pending_)++replaced_;
    pending_=m;ingress_=steady();drain();});
  odom_sub_=create_subscription<Odom>("raw_body_pose",rclcpp::SensorDataQoS().keep_last(200),[this](Odom::SharedPtr m){odom(m);});
  ref_sub_=create_subscription<Path>("reference",rclcpp::QoS(1).reliable().transient_local(),[this](Path::SharedPtr m){
    std::lock_guard<std::mutex> l(lock_);
    if(m->header.frame_id!=frame_)m->poses.clear();
    reference_=m; blocked_=true; have_traj_=false; turn_ok_=false;
  });
  traj_sub_=create_subscription<scan_planner_msgs::msg::Bspline>("verified_bspline",1,[this](scan_planner_msgs::msg::Bspline::SharedPtr m){trajectory(m);});
  turn_sub_=create_subscription<geometry_msgs::msg::PoseStamped>("turn_permit",1,[this](geometry_msgs::msg::PoseStamped::SharedPtr m){
    std::lock_guard<std::mutex> l(lock_);
    turn_ok_=reference_ && !reference_->poses.empty() && m->header.frame_id==frame_ &&
      m->header.stamp==reference_->header.stamp &&
      (xyz(m->pose.position)-xyz(reference_->poses.back().pose.position)).norm()<1e-6 &&
      std::abs(wrap(tf2::getYaw(m->pose.orientation)-tf2::getYaw(reference_->poses.back().pose.orientation)))<1e-6;
  });
  alive_sub_=create_subscription<std_msgs::msg::Bool>("route_alive",1,[this](std_msgs::msg::Bool::SharedPtr m){route_alive_=m->data?steady():0.;});
  cloud_pub_=create_publisher<Cloud>("world_cloud",latest);
  pose_pub_=create_publisher<Odom>("body_pose",latest);
  aligned_pub_=create_publisher<Odom>("aligned_odom",latest);
  cmd_pub_=create_publisher<geometry_msgs::msg::Twist>("cmd_vel",latest);
  stamped_pub_=create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped",latest);
  metric_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("timing",100);
  blocked_pub_=create_publisher<std_msgs::msg::Bool>("blocked",1);
  status_pub_=create_publisher<std_msgs::msg::String>("control_status",1);
  local_pub_=create_publisher<Path>("local_path",1);
  timer_=create_wall_timer(std::chrono::milliseconds(5),[this]{drain();});
  safety_=create_wall_timer(std::chrono::milliseconds(10),[this]{check();});
  control_metric_=create_publisher<std_msgs::msg::Float64MultiArray>("control_timing",rclcpp::SensorDataQoS().keep_last(100));
  control_=std::thread([this]{controlLoop();});
 }
 ~Bridge(){run_=false;if(control_.joinable())control_.join();}
private:
 void odom(Odom::SharedPtr m){
  if(m->header.frame_id!=(use_alignment_?"nvblox_odom":frame_) || m->child_frame_id!="base_link_dog")return;
  if(use_alignment_){
    if(steady()-alignment_time_>1.5)return;
    auto& q=m->pose.pose.orientation;Eigen::Quaterniond body(q.w,q.x,q.y,q.z);
    Eigen::Vector3d p=map_rotation_*xyz(m->pose.pose.position)+map_translation_;body=map_rotation_*body;
    m->pose.pose.position.x=p.x();m->pose.pose.position.y=p.y();m->pose.pose.position.z=p.z();
    q.w=body.w();q.x=body.x();q.y=body.y();q.z=body.z();m->header.frame_id=frame_;
  }
  const auto&q=m->pose.pose.orientation;
  Pose p{stamp(m->header.stamp),xyz(m->pose.pose.position),{q.w,q.x,q.y,q.z}};
  if(!p.p.allFinite()||!p.q.coeffs().allFinite()||p.q.norm()<.5)return;
  p.q.normalize();
  if(!poses_.empty() && p.stamp<poses_.back().stamp-.5){poses_.clear();pending_.reset();blocked_=true;}
  if(!poses_.empty() && p.stamp<=poses_.back().stamp)return;
  poses_.push_back(p);while(poses_.size()>2 && poses_[1].stamp<p.stamp-2)poses_.pop_front();
  {
   std::lock_guard<std::mutex> l(lock_);position_=p.p; yaw_=tf2::getYaw(q);odom_time_=steady();odom_ros_=p.stamp;
  }
  // nav_msgs/Odometry twist is in child coordinates; SCAN expects world velocity.
  Eigen::Vector3d vel(m->twist.twist.linear.x,m->twist.twist.linear.y,m->twist.twist.linear.z);
  // Project body odometry currently supplies world velocity: derive from measured
  // body positions instead, avoiding any assumption about the old twist producer.
  if(poses_.size()>1){const auto&a=poses_[poses_.size()-2];vel=(p.p-a.p)/std::max(.001,p.stamp-a.stamp);}
  if(!vel.allFinite()||vel.norm()>4)vel.setZero();
  m->twist.twist.linear.x=vel.x();m->twist.twist.linear.y=vel.y();m->twist.twist.linear.z=vel.z();
  pose_pub_->publish(*m);if(use_alignment_)aligned_pub_->publish(*m);drain();
 }
 static int field(const Cloud& c,const char* name,int type,int bytes){
  for(const auto&f:c.fields)if(f.name==name&&f.datatype==type&&f.count==1&&f.offset+bytes<=c.point_step)return f.offset;
  throw std::runtime_error(std::string("missing field ")+name);
 }
 template<class T>static T read(const uint8_t*p){T v;std::memcpy(&v,p,sizeof(v));return v;}
 void drain(){
  if(!pending_)return;
  auto c=pending_;double began=ingress_;
  try{
   if(c->header.frame_id!="lidar_link"&&c->header.frame_id!="base_link_dog")throw std::runtime_error("unverified raw cloud frame");
   if(c->is_bigendian||uint64_t(c->width)*c->point_step>c->row_step||uint64_t(c->height)*c->row_step>c->data.size())throw std::runtime_error("bad cloud layout");
   int fx=field(*c,"x",7,4),fy=field(*c,"y",7,4),fz=field(*c,"z",7,4),ft=field(*c,"timestamp",8,8);
   double lo=stamp(c->header.stamp),hi=lo;
   for(size_t row=0;row<c->height;++row)for(size_t col=0;col<c->width;++col){
    double t=read<double>(c->data.data()+row*c->row_step+col*c->point_step+ft);
    if(std::isfinite(t)&&std::abs(t-stamp(c->header.stamp))<=.25){lo=std::min(lo,t);hi=std::max(hi,t);}
   }
   if(poses_.empty()||poses_.back().stamp<hi){if(steady()-began<.15)return;throw std::runtime_error("pose does not bracket scan");}
   if(poses_.front().stamp>lo)throw std::runtime_error("scan older than pose history");
   auto base=se2_deskew::interpolate(poses_,stamp(c->header.stamp));
   int n=std::max(1,int(std::ceil((hi-lo)/.0005)));double dt=(hi-lo)/n;
   std::vector<Eigen::Isometry3f> transforms;transforms.reserve(n+1);
   for(int i=0;i<=n;++i){auto p=se2_deskew::interpolate(poses_,std::min(hi,lo+i*dt));Eigen::Isometry3f t=Eigen::Isometry3f::Identity();t.linear()=p.q.toRotationMatrix().cast<float>();t.translation()=p.p.cast<float>();transforms.push_back(t);}
   std::vector<Eigen::Vector3f> points;points.reserve(c->width*c->height);
   std::unordered_set<int64_t> voxels;voxels.reserve(c->width*c->height);
   for(size_t row=0;row<c->height;++row)for(size_t col=0;col<c->width;++col){
    const uint8_t*p=c->data.data()+row*c->row_step+col*c->point_step;
    Eigen::Vector3f v(read<float>(p+fx),read<float>(p+fy),read<float>(p+fz));double t=read<double>(p+ft);
    if(!v.allFinite()||!std::isfinite(t)||t<lo||t>hi||v.squaredNorm()>16.f)continue;
    // Measured merged NX cloud is already body aligned; no guessed lidar rotation.
    if(std::abs(v.x())<.41f&&std::abs(v.y())<.253f&&v.z()>-.15f&&v.z()<.35f)continue;
    int i=dt>0?std::clamp(int(std::llround((t-lo)/dt)),0,n):0;
    Eigen::Vector3f w=transforms[i]*v;
    // Keep one return per 4 cm world voxel, never average obstacle surfaces away.
    Eigen::Vector3i k=(w/.04f).array().floor().cast<int>();
    int64_t key=(int64_t(k.x()+1048576)<<42)^(int64_t(k.y()+1048576)<<21)^int64_t(k.z()+1048576);
    if(voxels.insert(key).second)points.push_back(w);
   }
   pending_.reset();
   if(points.empty())throw std::runtime_error("empty cloud");
   auto world=std::make_shared<Cloud>();world->header=c->header;world->header.frame_id=frame_;
   sensor_msgs::PointCloud2Modifier mod(*world);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(points.size());
   sensor_msgs::PointCloud2Iterator<float>x(*world,"x"),y(*world,"y"),z(*world,"z");
   for(const auto&p:points){*x=p.x();*y=p.y();*z=p.z();++x;++y;++z;}
   auto pose=std::make_shared<Odom>();pose->header=world->header;pose->child_frame_id="base_link_dog";
   pose->pose.pose.position.x=base.p.x();pose->pose.pose.position.y=base.p.y();pose->pose.pose.position.z=base.p.z();
   pose->pose.pose.orientation.w=base.q.w();pose->pose.pose.orientation.x=base.q.x();pose->pose.pose.orientation.y=base.q.y();pose->pose.pose.orientation.z=base.q.z();
   double transformed=steady();map_.ingest(world,pose);double mapped=steady();
   cloud_time_=steady();cloud_ros_=stamp(c->header.stamp);check();
   std_msgs::msg::Float64MultiArray m;
   // schema: source stamp, ingress steady, decision steady, preprocess ms,
   // map ms, decision ms, total ms, sensor age ms, input points, kept, replaced, dropped
   double end=steady();m.data={stamp(c->header.stamp),began,end,(transformed-began)*1000,(mapped-transformed)*1000,(end-mapped)*1000,(end-began)*1000,(now().seconds()-stamp(c->header.stamp))*1000,double(c->width*c->height),double(points.size()),double(replaced_),double(dropped_),double(blocked_),double(active_path_)};
   metric_pub_->publish(m);cloud_pub_->publish(*world);
  }catch(const std::exception&e){pending_.reset();++dropped_;blocked_=true;RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"cloud rejected: %s",e.what());}
 }
 void trajectory(scan_planner_msgs::msg::Bspline::SharedPtr m){
  if(m->order!=3||m->pos_pts.size()<4||m->knots.size()!=m->pos_pts.size()+4)return;
  Eigen::MatrixXd p(3,m->pos_pts.size());Eigen::VectorXd k(m->knots.size());
  for(size_t i=0;i<m->pos_pts.size();++i)p.col(i)=xyz(m->pos_pts[i]);
  for(size_t i=0;i<m->knots.size();++i){k(i)=m->knots[i];if(!std::isfinite(k(i))||(i&&k(i)<k(i-1)))return;}
  if(!p.allFinite()||k(k.size()-4)<=k(3))return;
  Spline s(p,3,.1);s.setKnot(k);
  {std::lock_guard<std::mutex> l(lock_);traj_=s;velocity_=s.getDerivative();duration_=s.getTimeSum();traj_time_=steady();progress_=0;have_traj_=true;blocked_=true;}
  check();
 }
 double desiredYaw(const Eigen::Vector3d& pos,const Path&r)const{
  double best=1e99,y=yaw_;
  for(size_t i=0;i<r.poses.size();++i){auto a=xyz(r.poses[i].pose.position);double d=(a-pos).squaredNorm();
   if(d<best){best=d;y=tf2::getYaw(r.poses[i].pose.orientation);}
   if(i+1<r.poses.size()){
    auto b=xyz(r.poses[i+1].pose.position);auto v=b-a;double n=v.squaredNorm();if(n<1e-8)continue;
    double u=std::clamp((pos-a).dot(v)/n,0.,1.);d=(pos-a-u*v).squaredNorm();
    if(d<best){best=d;double ya=tf2::getYaw(r.poses[i].pose.orientation),yb=tf2::getYaw(r.poses[i+1].pose.orientation);y=ya+u*wrap(yb-ya);}
   }
  }return y;
 }
 bool occupied(const Eigen::Vector3d&p,double yaw){
  // Complete physical rectangle, including corners, against SCAN's raw map.
  // Leg/step support is checked by the static SE(2) mesh validator. This prism
  // is the torso, from body -0.10 m to +0.40 m, with voxel intersection padding.
  double c=std::cos(yaw),s=std::sin(yaw),r=map_.getResolution();
  Eigen::Vector3i low,high;map_.posToIndex(p-Eigen::Vector3d(.6,.6,.1+r),low);map_.posToIndex(p+Eigen::Vector3d(.6,.6,.4+r),high);
  for(int x=low.x();x<=high.x();++x)for(int y=low.y();y<=high.y();++y){
   Eigen::Vector3d w;map_.indexToPos(Eigen::Vector3i(x,y,0),w);double dx=w.x()-p.x(),dy=w.y()-p.y();
   double pad=.5*r*(std::abs(c)+std::abs(s));if(std::abs(c*dx+s*dy)>.41+pad||std::abs(-s*dx+c*dy)>.253+pad)continue;
   for(int z=low.z();z<=high.z();++z)if(map_.getOccupancy(Eigen::Vector3i(x,y,z))!=0)return true;
  }return false;
 }
 void check(){
  active_path_=false;
  std::vector<std::pair<Eigen::Vector3d,double>> samples;
  {
   std::lock_guard<std::mutex> l(lock_);
   if(!reference_||reference_->poses.empty()||steady()-odom_time_>.2||steady()-cloud_time_>.22){blocked_=true;return;}
   active_path_=(have_traj_ && steady()-traj_time_<1.)||turn_ok_;
   samples.push_back({position_,yaw_});
   if(have_traj_){
    double best=1e99,tbest=progress_;for(double t=std::max(0.,progress_-.1);t<=std::min(duration_,progress_+.8);t+=.025){double d=(traj_.evaluateDeBoorT(t)-position_).squaredNorm();if(d<best){best=d;tbest=t;}}
    progress_=tbest;
    for(double t=tbest;t<=std::min(duration_,tbest+1.);t+=.08){auto p=traj_.evaluateDeBoorT(t);samples.push_back({p,desiredYaw(p,*reference_)});}
   }else if(turn_ok_){double dy=wrap(tf2::getYaw(reference_->poses.back().pose.orientation)-yaw_);for(int i=1;i<=10;++i)samples.push_back({position_,yaw_+dy*i/10});}
   else{blocked_=true;return;}
  }
  bool blocked=false;
  for(size_t i=0;i<samples.size() && !blocked;++i){
    const auto&a=samples[i?i-1:0];const auto&b=samples[i];
    double dy=wrap(b.second-a.second);
    int n=std::max(1,std::max(int(std::ceil((b.first-a.first).norm()/.04)),int(std::ceil(std::abs(dy)/.06))));
    // Also cover the measured-pose -> spline connector and changing yaw, not
    // only spline samples. The voxel intersection padding covers substeps.
    for(int j=0;j<=n;++j){double u=double(j)/n;
      if(occupied(a.first+u*(b.first-a.first),a.second+u*dy)){blocked=true;break;}
    }
  }
  blocked_=blocked;decision_time_=steady();std_msgs::msg::Bool b;b.data=blocked;blocked_pub_->publish(b);
 }
 void controlLoop(){
  auto deadline=Clock::now();double last=steady();int count=0;
  while(run_&&rclcpp::ok()){
   deadline+=std::chrono::microseconds(int64_t(1e6/hz_));
   geometry_msgs::msg::Twist cmd;std::string reason="waiting";double t=steady();
   {
    std::unique_lock<std::mutex> l(lock_,std::try_to_lock);
    if(!l.owns_lock()){reason="state_busy";}else{
    double ros=now().seconds();
    bool fresh=t-route_alive_<.25&&t-odom_time_<.2&&t-cloud_time_<.22&&t-decision_time_<.1 &&
      (!use_alignment_||t-alignment_time_<1.5) &&
      ros-odom_ros_ < .3 && ros-odom_ros_ > -.1 && ros-cloud_ros_ < .35 && ros-cloud_ros_ > -.1;
    if(reference_&&!reference_->poses.empty()&&fresh&&!blocked_){
     double desired=desiredYaw(position_,*reference_);Eigen::Vector3d vel=Eigen::Vector3d::Zero();
     if(have_traj_&&t-traj_time_<1.){
      double at=std::min(duration_,progress_+lookahead_);auto p=traj_.evaluateDeBoorT(at);vel=velocity_.evaluateDeBoorT(at)+.8*(p-position_);desired=desiredYaw(p,*reference_);reason="tracking";
     }else if(turn_ok_){desired=tf2::getYaw(reference_->poses.back().pose.orientation);reason="turn";}else reason="stale_trajectory";
     if(reason=="tracking"||reason=="turn"){
      double dy=wrap(desired-yaw_);cmd.angular.z=std::clamp(1.5*dy,-max_yaw_,max_yaw_);
      if(std::abs(dy)<.35){double c=std::cos(yaw_),s=std::sin(yaw_);cmd.linear.x=std::clamp(c*vel.x()+s*vel.y(),-max_x_,max_x_);cmd.linear.y=std::clamp(-s*vel.x()+c*vel.y(),-max_y_,max_y_);}
     }
    }else reason=!fresh?"stale_sensor_or_decision":blocked_?"blocked":"no_route";
    }
   }
   double prepared=steady();
   geometry_msgs::msg::TwistStamped stamped;stamped.header.stamp=now();stamped.header.frame_id="base_link_dog";stamped.twist=cmd;
   stamped_pub_->publish(stamped);
   cmd_pub_->publish(cmd);
   double published=steady();
   std_msgs::msg::Float64MultiArray metric;metric.data={published,(published-last)*1000,cmd.linear.x,cmd.linear.y,cmd.angular.z,(prepared-t)*1000,(published-prepared)*1000};control_metric_->publish(metric);last=published;
   if(++count%30==0){std_msgs::msg::String m;m.data=reason;status_pub_->publish(m);}
   std::this_thread::sleep_until(deadline);if(Clock::now()>deadline+std::chrono::milliseconds(100))deadline=Clock::now();
  }
  if(rclcpp::ok()){geometry_msgs::msg::Twist stop;cmd_pub_->publish(stop);geometry_msgs::msg::TwistStamped s;s.header.stamp=now();s.header.frame_id="base_link_dog";stamped_pub_->publish(s);}
 }
 GridMap map_;std::string frame_;double hz_,max_x_,max_y_,max_yaw_,lookahead_;std::deque<Pose>poses_;Cloud::SharedPtr pending_;double ingress_=0;
 bool use_alignment_;Eigen::Quaterniond map_rotation_=Eigen::Quaterniond::Identity();Eigen::Vector3d map_translation_=Eigen::Vector3d::Zero();std::atomic<double>alignment_time_{0};
 std::mutex lock_;Eigen::Vector3d position_=Eigen::Vector3d::Zero();double yaw_=0,odom_time_=0,odom_ros_=0;
 Path::SharedPtr reference_;Spline traj_,velocity_;double duration_=0,progress_=0,traj_time_=0;bool have_traj_=false;
 std::atomic<bool>blocked_{true},run_{true},turn_ok_{false},active_path_{false};std::atomic<double>cloud_time_{0},cloud_ros_{0},decision_time_{0},route_alive_{0};
 size_t replaced_=0,dropped_=0;std::thread control_;
 rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;rclcpp::Subscription<Odom>::SharedPtr odom_sub_;rclcpp::Subscription<Path>::SharedPtr ref_sub_;
 rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr traj_sub_;rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr turn_sub_;rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alive_sub_;
 rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;rclcpp::Publisher<Odom>::SharedPtr pose_pub_;rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
 rclcpp::Publisher<Odom>::SharedPtr aligned_pub_;rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr alignment_sub_;
 rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_pub_;
 rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metric_pub_,control_metric_;
 rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr blocked_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;rclcpp::Publisher<Path>::SharedPtr local_pub_;
 rclcpp::TimerBase::SharedPtr timer_,safety_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);{auto node=std::make_shared<Bridge>();rclcpp::spin(node);}if(rclcpp::ok())rclcpp::shutdown();}
