#pragma once
#include "first.hpp"
namespace first {
// A finite transition between actual selected commands, without an acceleration
// limit. Stable tracking passes through unchanged. Holds bypass this entirely.
// Per operator request the blend itself adds no collision-check pass.
struct SwitchBlend {
 Velocity target{},origin{};double began=0,until=0;int phase=-1,path=-1;bool initialized=false,active=false;
 void reset(){*this=SwitchBlend{};}
 Velocity update(Velocity wanted,Velocity sent,int stage,int id,double now,const Limits& limits){
  bool changed=!initialized||stage!=phase||(stage%10==0&&id!=path)||
   wanted.x*target.x<0||wanted.y*target.y<0||wanted.w*target.w<0;
  if(active&&now>=until)active=false;
  if(changed&&!active){
   until=now+.22;
   origin=sent;began=now;active=true;
  }
  target=wanted;phase=stage;path=id;initialized=true;
  double u=active?std::clamp((now-began)/(until-began),0.,1.):1.;u=u*u*(3-2*u);
  Velocity out=active?Velocity{origin.x+u*(target.x-origin.x),origin.y+u*(target.y-origin.y),origin.w+u*(target.w-origin.w)}:target;
  // Gait changes may tighten limits immediately, even during a transition.
  double scale=1.;auto bound=[&](double value,double cap){if(std::abs(value)>cap)scale=std::min(scale,cap/std::abs(value));};
  bound(out.x,limits.x);bound(out.y,limits.y);bound(out.w,limits.w);bound(std::hypot(out.x,out.y),limits.planar);
  out.x*=scale;out.y*=scale;out.w*=scale;return out;
 }
};

enum class RecoveryPhase {Follow,Wait,Retreat,Recheck,Exhausted};
struct MotionProgress {
 Pose anchor{};double since=0,last=0;bool initialized=false;
 void reset(){*this=MotionProgress{};}
 bool stalled(Pose body,double now){
  if(!initialized){anchor=body;since=last=now;initialized=true;return false;}
  // Missing sensor/planner intervals do not count as failed motion time.
  if(now-last>.2)since+=now-last;
  last=now;
  if(std::hypot(body.x-anchor.x,body.y-anchor.y)>.08||std::abs(wrap(body.yaw-anchor.yaw))>.20){anchor=body;since=now;}
  return now-since>=2.5;
 }
};
// Estimate translation over a measured 200 ms baseline, not a selected command.
struct MeasuredSpeed {
 Pose anchor{};double since=0,value=0;bool initialized=false;
 void reset(){*this=MeasuredSpeed{};}
 double update(Pose body,double now){
  if(!initialized||now-since>.5){anchor=body;since=now;value=0;initialized=true;}
  else if(now-since>=.2){value=std::clamp(std::hypot(body.x-anchor.x,body.y-anchor.y)/(now-since),0.,1.5);anchor=body;since=now;}
  return value;
 }
};
struct Recovery {
 RecoveryPhase phase=RecoveryPhase::Follow;Pose origin{},goal{};double since=0,clear_since=-1,last_progress=0,total_retreat=0,retreat_distance=0;
 int attempts=0;bool attempt_moved=false;
 void reset(){*this=Recovery{};}
 const char* name()const{switch(phase){case RecoveryPhase::Follow:return "follow";case RecoveryPhase::Wait:return "blocked_wait";case RecoveryPhase::Retreat:return "retreat";case RecoveryPhase::Recheck:return "recheck";default:return "recovery_exhausted";}}
 void update(bool forward,bool enabled,Pose body,double progress,double now){
  if(phase==RecoveryPhase::Follow&&progress>last_progress+.5){attempts=0;total_retreat=0;last_progress=progress;}
  if(phase==RecoveryPhase::Follow&&!forward){phase=RecoveryPhase::Wait;since=now;}
  if(phase==RecoveryPhase::Wait){
   if(forward){phase=RecoveryPhase::Follow;return;}
   if(enabled&&now-since>=.65){
    if(attempts>=2||total_retreat>=.85){phase=RecoveryPhase::Exhausted;return;}
    attempt_moved=false;origin=body;retreat_distance=std::min(.30+.20*attempts,.85-total_retreat);
    goal={body.x-retreat_distance*std::cos(body.yaw),body.y-retreat_distance*std::sin(body.yaw),body.z,body.yaw};
    since=now;clear_since=-1;phase=RecoveryPhase::Retreat;
   }
  }
  if(phase==RecoveryPhase::Retreat){
   double moved=std::hypot(body.x-origin.x,body.y-origin.y);
   // Sensor interruption or an entirely rejected reverse library is not an
   // executed retreat. Charge the bounded budget only after measured motion.
   if(moved>=.05&&!attempt_moved){++attempts;attempt_moved=true;}
   // Operator retains early release after measured separation and sustained
   // forward availability; increasing distances are targets, not minimums.
   if(forward&&moved>=.15){if(clear_since<0)clear_since=now;}else clear_since=-1;
   if(!enabled||moved>=retreat_distance||now-since>=4.||(clear_since>=0&&now-clear_since>=.3)){
    total_retreat+=moved;phase=RecoveryPhase::Recheck;since=now;
   }
  }else if(phase==RecoveryPhase::Recheck&&now-since>=.3){
   phase=forward?RecoveryPhase::Follow:RecoveryPhase::Wait;since=now;
  }else if(phase==RecoveryPhase::Exhausted&&forward){phase=RecoveryPhase::Follow;}
 }
};
}
