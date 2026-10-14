#include "navigation.hpp"
#include <iostream>
#define REQUIRE(v) do{if(!(v))throw std::runtime_error(#v);}while(0)
int main(int argc,char**argv){
 REQUIRE(argc==2);first::Library lib(argv[1]);auto empty=lib.penalties({});
 auto check=[](const auto&){return true;};
 auto rear=first::select(lib,{-.45,0,0,0},empty,{.4,.3,.4,.4},check,nullptr,first::Direction::Reverse);
 REQUIRE(rear.id>=0&&rear.command.x<0&&std::abs(rear.command.w)<1e-9);
 auto walls=empty;for(const auto&p:lib.paths)if(p.velocity.x<0)walls.hard[p.id]=1;
 REQUIRE(first::select(lib,{-.45,0,0,0},walls,{},check,nullptr,first::Direction::Reverse).id==-1);
 REQUIRE(first::select(lib,{-.45,0,0,0},empty,{},[](const auto&){return false;},nullptr,first::Direction::Reverse).id==-1);
 auto forward=first::select(lib,{2,0,0,0},empty,{},check,nullptr,first::Direction::Forward);
 REQUIRE(forward.id>=0&&forward.command.x>0);
 // A close, off-center preview must correct translation without rotating
 // away from the route; a body already over-rotated must turn back toward it.
 double heading=0.;
 auto lateral=first::select(lib,{.35,-.35,0,0},empty,{},check,nullptr,first::Direction::Forward,&heading);
 REQUIRE(lateral.id>=0&&lateral.command.y<0&&std::abs(lateral.command.w)<.11);
 heading=1.05;
 auto correction=first::select(lib,{.36,.027,0,0},empty,{},check,nullptr,first::Direction::Forward,&heading);
 REQUIRE(correction.id>=0&&correction.command.w>.5);
 auto capped=first::select(lib,{.36,.027,0,0},empty,{.4,.3,.4,.4},check,nullptr,first::Direction::Forward,&heading);
 REQUIRE(capped.id>=0&&capped.command.w<=.400001);
 REQUIRE(first::select(lib,{.36,.027,0,0},empty,{},[](const auto&){return false;},nullptr,first::Direction::Forward,&heading).id==-1);
 // A stationary primitive is not a usable route that suppresses recovery.
 auto still=empty;for(const auto&p:lib.paths)if(std::hypot(p.velocity.x,p.velocity.y)>.001||std::abs(p.velocity.w)>.001)still.hard[p.id]=1;
 REQUIRE(first::select(lib,{2,0,0,0},still,{},check,nullptr,first::Direction::Forward).id==-1);
 first::SwitchBlend blend;first::Velocity sent{.3,0,0};
 auto v=blend.update({1.5,0,0},sent,0,1,0,{});REQUIRE(v.x==.3&&blend.active);
 v=blend.update({1.5,0,0},v,0,1,.11,{});REQUIRE(std::abs(v.x-.9)<1e-9);
 v=blend.update({1.5,0,0},v,0,1,.23,{});REQUIRE(v.x==1.5&&!blend.active);
 // Stable phase/primitive is not continuously acceleration limited.
 v=blend.update({1.4,0,0},v,0,1,.24,{});REQUIRE(v.x==1.4&&!blend.active);
 v=blend.update({.3,.2,.4},v,0,2,.25,{});REQUIRE(v.x==1.4&&blend.active);
 auto before=v;v=blend.update({.2,-.2,0},v,0,3,.30,{});REQUIRE(v.x<before.x&&v.x>.2);
 v=blend.update({.2,-.2,0},v,0,3,.48,{});REQUIRE(v.x==.2&&!blend.active);
 // Phase change, tightened gait limits and reset after a hold.
 v=blend.update({0,0,.4},{1.5,0,0},2,3,.5,{.4,.3,.4,.4});REQUIRE(v.x<=.4);
 v=blend.update({0,0,.4},v,2,3,.73,{.4,.3,.4,.4});REQUIRE(v.x==0&&v.w==.4);
 blend.reset();v=blend.update({-.3,0,0},{},20,4,1,{});REQUIRE(v.x==0);
 v=blend.update({-.3,0,0},v,20,4,1.23,{});REQUIRE(v.x==-.3);
 // Changing primitive on every 30 Hz plan must not pin interpolation at u=0.
 blend.reset();v={};
 for(int i=0;i<7;++i)v=blend.update({1.,0,i%2?.1:-.1},v,0,i,i/30.,{});
 REQUIRE(v.x>.95);
 v=blend.update({1.,0,-.1},v,0,6,.24,{});REQUIRE(v.x==1.);
 first::Recovery r;first::Pose body{};
 r.update(false,false,body,0,0);r.update(false,false,body,0,2);REQUIRE(r.phase==first::RecoveryPhase::Wait&&r.attempts==0);
 r.update(false,true,body,0,2.1);REQUIRE(r.phase==first::RecoveryPhase::Retreat&&r.attempts==0&&r.goal.x==-.30);
 body.x=-.2;r.update(true,true,body,0,2.2);r.update(true,true,body,0,2.49);REQUIRE(r.phase==first::RecoveryPhase::Retreat);
 r.update(true,true,body,0,2.51);REQUIRE(r.phase==first::RecoveryPhase::Recheck);
 r.update(true,true,body,0,2.83);REQUIRE(r.phase==first::RecoveryPhase::Follow);
 r.update(false,true,body,0,3);r.update(false,true,body,0,3.66);REQUIRE(r.attempts==1&&std::abs(r.retreat_distance-.50)<1e-9);
 body.x-=.06;r.update(false,true,body,0,3.68);REQUIRE(r.attempts==2);
 r.update(false,false,body,0,3.7);REQUIRE(r.phase==first::RecoveryPhase::Recheck); // rear scan loss
 r.update(false,true,body,0,4.01);r.update(false,true,body,0,4.7);REQUIRE(r.phase==first::RecoveryPhase::Exhausted);
 r.update(true,true,body,0,4.8);r.update(true,true,body,.6,4.9);REQUIRE(r.attempts==0);
 r.reset();r.update(false,true,{},0,0);r.update(false,true,{},0,.7);r.update(false,true,{},0,4.71);
 REQUIRE(r.phase==first::RecoveryPhase::Recheck); // bounded time even if blocked behind
 REQUIRE(r.attempts==0); // no executed retreat, no exhausted movement budget
 r.reset();
 for(int i=0;i<6;++i){double t=i*2.;r.update(false,true,{},0,t);r.update(false,true,{},0,t+.7);r.update(false,false,{},0,t+.8);r.update(false,true,{},0,t+1.2);}
 REQUIRE(r.attempts==0&&r.phase!=first::RecoveryPhase::Exhausted);
 r.reset();REQUIRE(r.phase==first::RecoveryPhase::Follow&&r.attempts==0);
 // No early release before 15 cm; interrupted forward availability restarts the timer.
 r.reset();r.update(false,true,{},0,0);r.update(false,true,{},0,.7);
 r.update(true,true,{-.14,0,0,0},0,.8);r.update(true,true,{-.14,0,0,0},0,1.2);REQUIRE(r.phase==first::RecoveryPhase::Retreat);
 r.update(true,true,{-.16,0,0,0},0,1.3);r.update(false,true,{-.16,0,0,0},0,1.5);
 r.update(true,true,{-.16,0,0,0},0,1.6);r.update(true,true,{-.16,0,0,0},0,1.8);REQUIRE(r.phase==first::RecoveryPhase::Retreat);
 r.update(true,true,{-.16,0,0,0},0,1.91);REQUIRE(r.phase==first::RecoveryPhase::Recheck);
 // Later retreats have a larger target but cannot exceed remaining budget.
 r.reset();r.total_retreat=.70;r.attempts=1;r.update(false,true,{},0,0);r.update(false,true,{},0,.7);
 REQUIRE(std::abs(r.retreat_distance-.15)<1e-9);
 first::MeasuredSpeed speed;REQUIRE(speed.update({},0)==0);
 REQUIRE(std::abs(speed.update({.1,0,0,0},.2)-.5)<1e-9);
 REQUIRE(speed.update({.1,0,0,1},.4)==0);REQUIRE(speed.update({2,0,0,0},2)==0);
 first::MotionProgress progress;
 for(int i=0;i<25;++i)REQUIRE(!progress.stalled({0,0,0,i%2?.03:-.03},i*.1));
 REQUIRE(progress.stalled({0,0,0,.03},2.6)); // repeated micro-turns do not count as progress
 REQUIRE(!progress.stalled({.1,0,0,.03},2.7));
 progress.reset();for(int i=0;i<50;++i)REQUIRE(!progress.stalled({0,0,0,i*.04},i*.1)); // useful alignment
 progress.reset();REQUIRE(!progress.stalled({},0));REQUIRE(!progress.stalled({},20)); // input outage
 std::cout<<"PASS reverse selection, blocked rear, switching and recovery lifecycle\n";
}
