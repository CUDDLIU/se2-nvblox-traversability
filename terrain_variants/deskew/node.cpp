// Replay-only point-time compensation in the verified body-aligned cloud frame.
// No TF, extrinsic, raw recording, or robot command is modified.
#include "pose_math.hpp"
#include "scan_clock.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <cstring>
#include <limits>
#include <vector>

using se2_deskew::Pose;
using Cloud=sensor_msgs::msg::PointCloud2;
class Deskew:public rclcpp::Node {
 public:
  Deskew():Node("terrain_lidar_deskew") {
    restore_clock_=declare_parameter<bool>("restore_scan_time",false);
    pub_=create_publisher<Cloud>("/terrain_variants/lidar_deskewed",rclcpp::SensorDataQoS().keep_last(4));
    cloud_=create_subscription<Cloud>("/LIDAR/POINTS_NX",rclcpp::SensorDataQoS().keep_last(4),
      [this](Cloud::SharedPtr c){
        ++received_;
        if(pending_.size()>=4){pending_.pop_front();++dropped_;}
        pending_.push_back({std::move(c),std::chrono::steady_clock::now()});drain();
      });
    odom_=create_subscription<nav_msgs::msg::Odometry>("/nvblox_lio/odom",rclcpp::SensorDataQoS().keep_last(300),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr o){
        if(o->header.frame_id!="nvblox_odom" || o->child_frame_id!="base_link_dog"){
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"ignore unverified odometry frames");return;
        }
        const auto& p=o->pose.pose.position;const auto& q=o->pose.pose.orientation;
        Pose v{stamp(o->header.stamp),{p.x,p.y,p.z},{q.w,q.x,q.y,q.z}};
        if(!v.p.allFinite() || !v.q.coeffs().allFinite() || v.q.norm()<.5)return;
        if(!poses_.empty() && v.stamp<poses_.back().stamp-.5){poses_.clear();pending_.clear();}
        if(!poses_.empty() && v.stamp<=poses_.back().stamp)return;
        v.q.normalize();poses_.push_back(v);
        while(poses_.size()>2 && poses_[1].stamp<v.stamp-2.)poses_.pop_front();
        drain();
      });
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{drain();});
    report_=create_wall_timer(std::chrono::seconds(5),[this]{
      RCLCPP_INFO(get_logger(),"frames=%lu dropped=%lu invalid_points=%lu pending=%zu mean_compute_ms=%.3f max_compute_ms=%.3f max_correction_m=%.4f clock_restored=%lu clock_plain=%lu clock_uncertain=%lu",
        frames_,dropped_,invalid_,pending_.size(),frames_?total_ms_/frames_:0.,max_ms_,max_correction_,clock_restored_,clock_plain_,clock_uncertain_);
    });
  }
  ~Deskew() override {
    RCLCPP_INFO(get_logger(),"DESKEW_FINAL received=%lu frames=%lu dropped=%lu invalid_points=%lu pending=%zu mean_compute_ms=%.3f max_compute_ms=%.3f max_correction_m=%.4f clock_restored=%lu clock_plain=%lu clock_uncertain=%lu",
      received_,frames_,dropped_,invalid_,pending_.size(),frames_?total_ms_/frames_:0.,max_ms_,max_correction_,clock_restored_,clock_plain_,clock_uncertain_);
  }
 private:
  static double stamp(const builtin_interfaces::msg::Time& t){return t.sec+t.nanosec*1e-9;}
  struct Pending {Cloud::SharedPtr cloud;std::chrono::steady_clock::time_point received;};
  static int field(const Cloud& c,const char* name,int datatype,size_t width){
    for(const auto& f:c.fields)if(f.name==name && f.datatype==datatype && f.count==1 && uint64_t(f.offset)+width<=c.point_step)return f.offset;
    throw std::runtime_error(std::string("unsupported point field ")+name);
  }
  template<class T>static T read(const uint8_t* p){T v;std::memcpy(&v,p,sizeof(v));return v;}
  void drain(){
    while(!pending_.empty()){
      const auto start=std::chrono::steady_clock::now();
      auto& c=*pending_.front().cloud;
      try {
        if(c.header.frame_id!="lidar_link" && c.header.frame_id!="base_link_dog")throw std::runtime_error("unverified cloud frame");
        if(c.is_bigendian || uint64_t(c.width)*c.point_step>c.row_step ||
           (c.height && uint64_t(c.height-1)*c.row_step+uint64_t(c.width)*c.point_step>c.data.size()))throw std::runtime_error("unsupported cloud layout");
        const int x=field(c,"x",7,4),y=field(c,"y",7,4),z=field(c,"z",7,4),t=field(c,"timestamp",8,8);
        const int ring=restore_clock_?field(c,"ring",4,2):0;
        se2_deskew::ScanClock scan_clock;
        const double reference=stamp(c.header.stamp);double lo=reference,hi=reference;
        for(size_t row=0;row<c.height;++row)for(size_t col=0;col<c.width;++col){
          const uint8_t* p=c.data.data()+row*c.row_step+col*c.point_step;
          double pt=read<double>(p+t);
          if(std::isfinite(pt) && std::abs(pt-reference)<=.25){
            lo=std::min(lo,pt);hi=std::max(hi,pt);
            if(restore_clock_)scan_clock.observe(read<uint16_t>(p+ring),read<float>(p+y),read<float>(p+z),pt);
          }
        }
        const int clock_evidence=restore_clock_?scan_clock.factor():1;
        const int clock_factor=clock_evidence==2?2:1;
        if(clock_factor==2)lo=std::min(reference,se2_deskew::restore_scan_time(lo,hi,2));
        if(poses_.empty() || poses_.back().stamp<hi){
          if(std::chrono::duration<double>(start-pending_.front().received).count()<.4)return;
          throw std::runtime_error("point timestamps not bracketed by recorded odometry");
        }
        if(poses_.front().stamp>lo)throw std::runtime_error("old point timestamp before buffered odometry");
        const Pose base=se2_deskew::interpolate(poses_,reference);
        // Uniform 0.5 ms pose table bounds time quantization to 0.25 ms;
        // interpolation uses recorded poses, without extrapolation.
        const int intervals=std::max(1,static_cast<int>(std::ceil((hi-lo)/.0005)));
        const double dt=(hi-lo)/intervals;
        std::vector<Eigen::Isometry3f> transforms;transforms.reserve(intervals+1);
        for(int i=0;i<=intervals;++i)transforms.push_back(se2_deskew::relative(base,
          se2_deskew::interpolate(poses_,std::min(hi,lo+i*dt))).cast<float>());
        const float nan=std::numeric_limits<float>::quiet_NaN();
        for(size_t row=0;row<c.height;++row)for(size_t col=0;col<c.width;++col){
          uint8_t* p=c.data.data()+row*c.row_step+col*c.point_step;
          double pt=read<double>(p+t);
          if(clock_factor==2)pt=se2_deskew::restore_scan_time(pt,hi,2);
          Eigen::Vector3f xyz(read<float>(p+x),read<float>(p+y),read<float>(p+z));
          if(!std::isfinite(pt) || pt<lo || pt>hi || !xyz.allFinite()){
            std::memcpy(p+x,&nan,4);std::memcpy(p+y,&nan,4);std::memcpy(p+z,&nan,4);++invalid_;continue;
          }
          int i=dt>0?std::clamp(static_cast<int>(std::llround((pt-lo)/dt)),0,intervals):0;
          Eigen::Vector3f corrected=transforms[i]*xyz;
          max_correction_=std::max(max_correction_,static_cast<double>((corrected-xyz).norm()));
          float cx=corrected.x(),cy=corrected.y(),cz=corrected.z();
          std::memcpy(p+x,&cx,4);std::memcpy(p+y,&cy,4);std::memcpy(p+z,&cz,4);
          // Downstream fields now consistently describe one reference time.
          std::memcpy(p+t,&reference,8);
        }
        c.is_dense=false;pub_->publish(c);++frames_;
        if(restore_clock_){
          if(clock_evidence==2)++clock_restored_;
          else if(clock_evidence==1)++clock_plain_;
          else ++clock_uncertain_;
        }
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        total_ms_+=ms;max_ms_=std::max(max_ms_,ms);
      }catch(const std::exception& e){
        ++dropped_;RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"skip deskew frame: %s",e.what());
      }
      pending_.pop_front();
    }
  }
  std::deque<Pose> poses_;std::deque<Pending> pending_;
  rclcpp::Publisher<Cloud>::SharedPtr pub_;rclcpp::Subscription<Cloud>::SharedPtr cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::TimerBase::SharedPtr timer_,report_;
  unsigned long received_=0,frames_=0,dropped_=0,invalid_=0;double total_ms_=0,max_ms_=0,max_correction_=0;
  bool restore_clock_=false;
  unsigned long clock_restored_=0,clock_plain_=0,clock_uncertain_=0;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Deskew>());rclcpp::shutdown();}
