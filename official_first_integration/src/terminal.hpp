#pragma once
#include "first.hpp"
namespace first {
enum class Phase {Follow,Approach,Align,Arrived};
struct TerminalCommand {Velocity velocity;Phase phase=Phase::Follow;};
// CmdCalculate 0xbd830..0xbe064: native terminal position/yaw controller.
// Bounds below are project limits, applied before validating the actual sweep.
inline TerminalCommand terminal(Velocity selected,Pose error,double remaining,const Limits& lim){
 TerminalCommand out{selected,Phase::Follow};
 double d=std::hypot(error.x,error.y);
 // Arc and floor guards are integration extensions: a loop or another floor
 // near the final XY coordinate must not trigger terminal control early.
 if(d>.5||remaining>.5||std::abs(error.z)>.15)return out;
 if(d>.1){
  out.phase=Phase::Approach;out.velocity={error.x,error.y,0};
  double k=.5;
  while(std::abs(out.velocity.x)<.25&&std::abs(out.velocity.y)<.25){
   k+=.05;out.velocity={k*error.x,k*error.y,0};
  }
 }else{
  double a=wrap(error.yaw);out.velocity={};
  if(std::abs(a)<=.1){out.phase=Phase::Arrived;return out;}
  out.phase=Phase::Align;
  out.velocity.w=std::abs(a)<.7?std::copysign(.7,a):1.5*a;
 }
 // Never boost after the planner. Preserve the native terminal direction while
 // limiting it for normal/stair gait; collision checks see this exact command.
 double scale=1.;auto bound=[&](double v,double max){if(std::abs(v)>max)scale=std::min(scale,max/std::abs(v));};
 bound(out.velocity.x,lim.x);bound(out.velocity.y,lim.y);bound(out.velocity.w,lim.w);
 bound(std::hypot(out.velocity.x,out.velocity.y),lim.planar);
 out.velocity.x*=scale;out.velocity.y*=scale;out.velocity.w*=scale;return out;
}
inline Primitive command_primitive(Velocity v){
 Primitive p;p.velocity=v;Pose q;
 for(auto&sample:p.samples){double c=std::cos(q.yaw),s=std::sin(q.yaw);q.x+=.1*(c*v.x-s*v.y);q.y+=.1*(s*v.x+c*v.y);q.yaw+=.1*v.w;sample=q;}
 p.end=q;return p;
}
}
