#pragma once
#include "first.hpp"
namespace first {
struct RouteTarget {Pose goal;double progress=0,remaining=0,stairs_ahead=INFINITY,cross_track=0,target_arc=0,lookahead=0,preview_angle=0,heading=0;bool on_stairs=false,rejoining=false;};
class Route {
 std::vector<Pose> points_;std::vector<double> arc_;std::vector<std::pair<double,double>> height_bands_;double progress_=0;bool rejoining_=false;
 public:
 double progress()const{return progress_;}
 Pose at(double s)const{
  if(s<=0)return points_.front();
  for(size_t i=1;i<points_.size();++i)if(arc_[i]>=s&&arc_[i]>arc_[i-1]+1e-9){
   double u=(s-arc_[i-1])/(arc_[i]-arc_[i-1]);auto a=points_[i-1],b=points_[i];
   return {a.x+u*(b.x-a.x),a.y+u*(b.y-a.y),a.z+u*(b.z-a.z),a.yaw+u*wrap(b.yaw-a.yaw)};
  }return points_.back();
 }
 void reset(const std::vector<Pose>&points){
  points_=points;arc_.assign(points.size(),0);progress_=0;rejoining_=false;height_bands_.clear();
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
 RouteTarget target(Pose body,double speed=0){
  if(points_.empty())throw std::runtime_error("empty route");
  double best=INFINITY,s=progress_;
  // Ordered arc window prevents jumping to a nearby later loop or another floor.
  for(size_t i=1;i<points_.size();++i){if(arc_[i]<progress_-.2||arc_[i-1]>progress_+3.)continue;
   auto a=points_[i-1],b=points_[i];double dx=b.x-a.x,dy=b.y-a.y,ds=arc_[i]-arc_[i-1];double u=ds>1e-9?std::clamp(((body.x-a.x)*dx+(body.y-a.y)*dy)/(ds*ds),0.,1.):0;
   double z=a.z+u*(b.z-a.z);if(std::abs(z-body.z)>.25)continue;
   double distance=std::hypot(body.x-a.x-u*dx,body.y-a.y-u*dy)+3*std::abs(body.z-z);
   if(distance<best){best=distance;s=arc_[i-1]+u*ds;}
  }
  if(points_.size()==1){best=std::hypot(body.x-points_[0].x,body.y-points_[0].y)+3*std::abs(body.z-points_[0].z);}
  if(best>1.)throw std::runtime_error("route disconnected from measured body");
  progress_=std::max(progress_,s);
  double nominal=std::clamp(1.+std::clamp(speed,0.,1.5),1.,2.5),angle=0.;
  // Measure bends over 40 cm chords. Individual 5 cm mesh zigzags must not
  // pin a straight corridor to minimum preview after every transient hold.
  double horizon=std::min(arc_.back(),progress_+nominal);
  for(double s=progress_;s+.15<horizon;s+=.2){
   auto a=at(s),b=at(std::min(horizon,s+.4));
   if(std::hypot(b.x-a.x,b.y-a.y)<.10)continue;
   angle=std::max(angle,std::abs(wrap(std::atan2(b.y-a.y,b.x-a.x)-body.yaw)));
  }
  double lookahead=std::clamp(nominal/(1.+1.5*angle),.45,2.5);
  double end=std::min(arc_.back(),progress_+lookahead);
  RouteTarget out;out.lookahead=lookahead;out.preview_angle=angle;out.progress=progress_;out.remaining=arc_.back()-progress_;

  auto anchor=at(progress_);out.cross_track=std::hypot(body.x-anchor.x,body.y-anchor.y);
  // Bound geometric shortcutting by distance, not the angle between adjacent
  // samples. Repeated yaw poses and several small turns cannot hide a doorway.
  auto within=[&](double candidate){
   auto goal=at(candidate);double dx=goal.x-anchor.x,dy=goal.y-anchor.y,d2=dx*dx+dy*dy;
   for(size_t i=1;i<points_.size();++i){if(arc_[i]<=progress_||arc_[i]>=candidate)continue;
    auto p=points_[i];double u=d2>1e-12?std::clamp(((p.x-anchor.x)*dx+(p.y-anchor.y)*dy)/d2,0.,1.):0.;
    if(std::hypot(p.x-anchor.x-u*dx,p.y-anchor.y-u*dy)>.10)return false;
   }
   // The robot may already be off the centerline. A shortcut acceptable from
   // the projected anchor can still cut a wall from the actual body position.
   // Require its lateral error to converge instead of increasing at a bend.
   for(double u:{.25,.5,.75}){
    double x=body.x+u*(goal.x-body.x),y=body.y+u*(goal.y-body.y),distance=INFINITY;
    for(size_t i=1;i<points_.size();++i){if(arc_[i]<progress_-.25||arc_[i-1]>candidate)continue;
     auto a=points_[i-1],b=points_[i];double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy;
     double v=d2>1e-12?std::clamp(((x-a.x)*dx+(y-a.y)*dy)/d2,0.,1.):0.;
     distance=std::min(distance,std::hypot(x-a.x-v*dx,y-a.y-v*dy));
    }
    if(distance>.10+std::max(0.,out.cross_track-.10)*(1-u)+1e-6)return false;
   }return true;
  };
  double accepted=progress_;
  for(double candidate=std::min(end,progress_+.05);candidate>accepted+1e-9;candidate=std::min(end,candidate+.05)){
   if(!within(candidate))break;
   accepted=candidate;
  }
  end=accepted;
  // Once off the route, target its nearby segment before looking around the
  // next corner. This is guidance only; live collision checks still apply.
  if(out.cross_track>.35)rejoining_=true;
  if(out.cross_track<.15)rejoining_=false;
  out.rejoining=rejoining_;
  for(auto [start,finish]:height_bands_){
   if(progress_>=start-.5&&progress_<=finish+.5)out.on_stairs=true;
   if(finish>=progress_)out.stairs_ahead=std::min(out.stairs_ahead,std::max(0.,start-progress_));
  }
  out.target_arc=end;out.goal=at(end);
  // Anticipate up to 25 cm of stored heading, bounded by the accepted chord.
  // This starts a real corner turn before passing its in-place yaw samples.
  out.heading=at(std::min(arc_.back(),std::min(end,progress_+.25)+1e-6)).yaw;
  return out;
 }
};
}
