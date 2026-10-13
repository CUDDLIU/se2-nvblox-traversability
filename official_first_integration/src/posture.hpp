#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <utility>
namespace first {
// Accept stable standing previews as well as stable navigation postures.
// Preserve existing map evidence. A pending request alone does not interrupt
// an already stable navigation gait.
class PostureGate {
 public:
 bool ready=false;
 double not_before=0;
 bool observe(int gait,int pending,double height,double stamp){
  if(!std::isfinite(height+stamp)||stamp<=last_stamp_)return ready;
  bool navigation=gait==0x3002||gait==0x3003;
  bool supported=navigation||(gait==0x1001&&pending==0);
  bool changed=gait!=last_gait_;
  bool lowering=pending!=0&&last_stamp_>0&&std::abs(height-last_height_)>.02;
  last_stamp_=stamp;last_height_=height;last_gait_=gait;
  if(!supported||changed||lowering){ready=false;samples_.clear();}
  if(!supported)return false;
  samples_.emplace_back(stamp,height);
  while(samples_.size()>1&&samples_[1].first<stamp-.35)samples_.pop_front();
  if(!ready&&stamp-samples_.front().first>=.30){
   auto bounds=std::minmax_element(samples_.begin(),samples_.end(),[](auto a,auto b){return a.second<b.second;});
   if(bounds.second->second-bounds.first->second<=.015){ready=true;not_before=stamp;}
  }
  return ready;
 }
 private:
 double last_stamp_=0,last_height_=0;int last_gait_=0;
 std::deque<std::pair<double,double>> samples_;
};
}
