#include "pose_math.hpp"
#include "scan_clock.hpp"
#include <iostream>
using namespace se2_deskew;
int main() {
  auto check=[](bool ok){if(!ok)throw std::runtime_error("deskew math regression");};
  const Eigen::Quaterniond identity=Eigen::Quaterniond::Identity();
  std::deque<Pose> poses{{100.,{0,0,0},identity},{100.1,{.1,0,0},identity}};
  auto half=interpolate(poses,100.05);
  check((half.p-Eigen::Vector3d(.05,0,0)).norm()<1e-10);
  check((relative(poses.front(),half)*Eigen::Vector3d(.95,2,3)-Eigen::Vector3d(1,2,3)).norm()<1e-10);
  poses.back().q=Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));
  poses.back().p=Eigen::Vector3d::Zero();half=interpolate(poses,100.05);
  Eigen::Vector3d fixed(1,0,0),measured=half.q.conjugate()*fixed;
  check((relative(poses.front(),half)*measured-fixed).norm()<1e-10);
  check((relative(half,poses.front())*fixed-measured).norm()<1e-10);
  check(interpolate(poses,100.).stamp==100. && interpolate(poses,100.1).stamp==100.1);
  bool refused=false;try{interpolate(poses,99.9);}catch(const std::out_of_range&){refused=true;}
  check(refused);
  for(int factor:{1,2}){
    ScanClock detector;
    for(int i=0;i<1800;++i){
      double real=100.+i/9000.;double angle=i*2*M_PI/900;
      double observed=100.2+(real-100.2)/factor;
      detector.observe(24,2*std::sin(angle),2*std::cos(angle)-.013,observed);
      check(std::abs(restore_scan_time(observed,100.2,factor)-real)<1e-10);
    }
    check(detector.factor()==factor);
  }
  ScanClock insufficient;check(insufficient.factor()==0);
  ScanClock wrong_rate;
  for(int i=0;i<200;++i){double angle=i*2*M_PI/900;
    wrong_rate.observe(24,std::sin(angle),std::cos(angle)-.013,100.+i/12000.);}
  check(wrong_rate.factor()==0);
  std::cout<<"deskew pose/range and scan-clock detection/inversion checks passed\n";
}
