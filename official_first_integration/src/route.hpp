#pragma once
#include "first.hpp"
namespace first {
struct RouteTarget {Pose goal;double progress=0,remaining=0,stairs_ahead=INFINITY;bool on_stairs=false;};
class Route {
 std::vector<Pose> points_;std::vector<double> arc_;std::vector<std::pair<double,double>> height_bands_;double progress_=0;
 public:
 void reset(const std::vector<Pose>&points){
  points_=points;arc_.assign(points.size(),0);progress_=0;height_bands_.clear();
  for(size_t i=1;i<points.size();++i)arc_[i]=arc_[i-1]+std::hypot(points[i].x-points[i-1].x,points[i].y-points[i-1].y);
  // Gait classification only: reject isolated mesh-height spikes using a
  // metric median, independent of waypoint density and duplicate yaw samples.
  // Leave route geometry, collision checks and height/floor guards unchanged.
  auto height_at=[&](double s){
   if(s<=0)return points.front().z;
   if(s>=arc_.back())return points.back().z;
   size_t j=std::upper_bound(arc_.begin(),arc_.end(),s)-arc_.begin();
   double u=(s-arc_[j-1])/(arc_[j]-arc_[j-1]);
   return points[j-1].z+u*(points[j].z-points[j-1].z);
  };
  std::vector<double> heights;heights.reserve(points.size());
  for(double s:arc_){std::array<double,5> h;for(int j=0;j<5;++j)h[j]=height_at(s+(j-2)*.05);std::sort(h.begin(),h.end());heights.push_back(h[2]);}
  for(size_t i=0;i+1<points.size();++i){
   if(std::abs(heights[i+1]-heights[i])>.06||
      (arc_[i+1]-arc_[i]<1e-6&&std::abs(points[i+1].z-points[i].z)>.06))height_bands_.emplace_back(arc_[i],arc_[i+1]);
   // Dense ASA samples may subdivide a 15 cm riser into changes < 6 cm.
   // Detect accumulated rise/fall as well; sample spacing must not disable gait.
   for(size_t j=i+1;j<points.size()&&arc_[j]-arc_[i]<=.5+1e-6;++j)
    if(std::abs(heights[j]-heights[i])>=.10){height_bands_.emplace_back(arc_[i],arc_[j]);break;}
  }
 }
 RouteTarget target(Pose body,double lookahead=2.5){
  if(points_.empty())throw std::runtime_error("empty route");
  double best=INFINITY,s=progress_;size_t segment=0;
  // Ordered arc window prevents jumping to a nearby later loop or another floor.
  for(size_t i=1;i<points_.size();++i){if(arc_[i]<progress_-.2||arc_[i-1]>progress_+3.)continue;
   auto a=points_[i-1],b=points_[i];double dx=b.x-a.x,dy=b.y-a.y,ds=arc_[i]-arc_[i-1];double u=ds>1e-9?std::clamp(((body.x-a.x)*dx+(body.y-a.y)*dy)/(ds*ds),0.,1.):0;
   double z=a.z+u*(b.z-a.z);if(std::abs(z-body.z)>.25)continue;
   double distance=std::hypot(body.x-a.x-u*dx,body.y-a.y-u*dy)+3*std::abs(body.z-z);
   if(distance<best){best=distance;s=arc_[i-1]+u*ds;segment=i;}
  }
  if(points_.size()==1){best=std::hypot(body.x-points_[0].x,body.y-points_[0].y)+3*std::abs(body.z-points_[0].z);}
  if(best>1.)throw std::runtime_error("route disconnected from measured body");
  progress_=std::max(progress_,s);double end=std::min(arc_.back(),progress_+lookahead);
  RouteTarget out;out.progress=progress_;out.remaining=arc_.back()-progress_;
  // Keep large geometric bends within the target window without feeding every
  // dense ASA sample as a stopping waypoint. Duplicate yaw samples don't stop it.
  for(size_t i=std::max(size_t(1),segment);i+1<points_.size();++i){if(arc_[i]>end)break;if(arc_[i]<progress_+.25)continue;
   auto a=points_[i-1],b=points_[i],c=points_[i+1];double ax=b.x-a.x,ay=b.y-a.y,bx=c.x-b.x,by=c.y-b.y;
   if(std::hypot(ax,ay)>.01&&std::hypot(bx,by)>.01&&std::abs(wrap(std::atan2(by,bx)-std::atan2(ay,ax)))>pi/3){end=arc_[i];break;}
  }
  for(auto [start,finish]:height_bands_){
   if(progress_>=start-.5&&progress_<=finish+.5)out.on_stairs=true;
   if(finish>=progress_)out.stairs_ahead=std::min(out.stairs_ahead,std::max(0.,start-progress_));
  }
  out.goal=points_.back();
  for(size_t i=1;i<points_.size();++i)if(arc_[i]>=end&&arc_[i]>arc_[i-1]+1e-9){double u=(end-arc_[i-1])/(arc_[i]-arc_[i-1]);auto a=points_[i-1],b=points_[i];out.goal={a.x+u*(b.x-a.x),a.y+u*(b.y-a.y),a.z+u*(b.z-a.z),a.yaw+u*wrap(b.yaw-a.yaw)};break;}
  return out;
 }
};
}
