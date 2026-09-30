#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace se2_deskew {
// Audit only the known M20 10 Hz spindle pattern. The archived upstream guard
// compresses two scan periods into one, keeping the latest point time fixed.
// A factor of zero means insufficient/ambiguous evidence, never permission to
// invent timestamps. Callers preserve original timing and count that case.
class ScanClock {
 public:
  void observe(unsigned ring,double y,double z,double time){
    unsigned local=ring%96;
    if(ring>=192 || (local!=24 && local!=48 && local!=72) ||
       !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(time))return;
    auto& last=last_[ring/96*3+local/24-1];
    double angle=std::atan2(y,z+.013);
    if(last.valid){
      double delta=std::abs(std::remainder(angle-last.angle,2*M_PI));
      double dt=time-last.time;
      if(delta>.001 && delta<.02 && dt>1e-6 && dt<.002)
        ratios_.push_back(delta/dt/(20*M_PI));
    }
    last={true,angle,time};
  }
  int factor(){
    if(ratios_.size()<60)return 0;
    const size_t n=ratios_.size();
    auto quantile=[&](size_t i){std::nth_element(ratios_.begin(),ratios_.begin()+i,ratios_.end());return ratios_[i];};
    double lo=quantile(n/10),median=quantile(n/2),hi=quantile(n*9/10);
    if(hi-lo>.15)return 0;
    if(std::abs(median-1)<.05)return 1;
    if(std::abs(median-2)<.05)return 2;
    return 0;
  }
 private:
  struct Last {bool valid=false;double angle=0,time=0;};
  std::array<Last,6> last_{};
  std::vector<double> ratios_;
};
inline double restore_scan_time(double observed,double end,int factor){
  return end+(observed-end)*factor;
}
}  // namespace se2_deskew
