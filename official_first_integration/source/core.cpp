// Source reconstruction from NOS localPlanner/pcl_pass_grid, not a binary wrapper.
// Audited addresses are recorded beside the recovered operations. The integration
// transport gates are explicit project additions.
#include "first.hpp"
#include "terminal.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <map>
#include <tuple>
#include <memory>
#include <cstring>
#include <climits>
#include <unordered_map>
#include <chrono>

namespace recovered {
using first::Pose;using first::Velocity;using first::Primitive;
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
struct Cell {int x,y,z;bool operator==(const Cell& b)const{return x==b.x&&y==b.y&&z==b.z;}};
struct CellHash {size_t operator()(const Cell& c)const{return size_t(uint32_t(c.x)*73856093u)^size_t(uint32_t(c.y)*19349663u)^size_t(uint32_t(c.z)*83492791u);}};
// Exact radius-count predicate used by PCL ROR: at least four other points
// within 10 cm. A spatial hash stops after five hits (including self), avoiding
// building/querying a KD tree for every frame. Voxel centroids/order unchanged.
void radius_filter(const Cloud& input,Cloud& output){
 std::unordered_map<Cell,int,CellHash> heads;heads.reserve(input.size());
 std::vector<int> next(input.size());std::vector<Cell> cells;cells.reserve(input.size());
 for(size_t i=0;i<input.size();++i){auto p=input[i];Cell c{int(std::floor(double(p.x)/.1)),int(std::floor(double(p.y)/.1)),int(std::floor(double(p.z)/.1))};cells.push_back(c);
  auto [it,inserted]=heads.emplace(c,int(i));next[i]=inserted?-1:it->second;it->second=int(i);
 }
 output.clear();output.reserve(input.size());
 for(size_t i=0;i<input.size();++i){auto p=input[i];auto c=cells[i];int hits=0;
  auto visit=[&](Cell key){auto it=heads.find(key);if(it==heads.end())return;
   for(int j=it->second;j>=0&&hits<5;j=next[j]){auto q=input[j];float x=p.x-q.x,y=p.y-q.y,z=p.z-q.z;
    if(double(x*x+y*y+z*z)<=.1*.1)++hits;
   }
  };
  visit(c);
  for(int dx=-1;dx<=1&&hits<5;++dx)for(int dy=-1;dy<=1&&hits<5;++dy)for(int dz=-1;dz<=1&&hits<5;++dz)
   if(dx||dy||dz)visit({c.x+dx,c.y+dy,c.z+dz});
  if(hits>=5)output.push_back(p);
 }
}
struct Output {double x=0,y=0,w=0,score=0;int id=-1,status=999,hard=0,retries=0;};
struct Params {
 double maxx=1.5,maxy=.6,maxw=1.,xy=.1,yaw=.1,deceleration=1.,stop=.7,minx=.25,miny=.25,minw=.7,linew=.2;
 bool direct=false,ignore_obstacles=false,backward=false;
};
class Planner {
 public:
 Params params;
 first::Library library;Cloud::Ptr filtered{new Cloud};std::vector<pcl::PointXYZ> occupied;
 std::array<double,10> cloud_stats{};
 bool active=false,line_close=false;int line_wait=0,rotate_wait=0,line_phase=0;double previous_x=0,max_range=0;
 explicit Planner(const char* paths):library(paths){
  // setPathData 0xba55c..0xba588 derives range from every library endpoint.
  for(auto p:library.paths)max_range=std::max(max_range,std::hypot(p.end.x,p.end.y));
 }
 void reset(){active=false;line_close=false;line_wait=rotate_wait=line_phase=0;previous_x=0;}
 void goal(){reset();active=true;}
 // PCL calls recovered from pcl_pass_grid::pointCloudCallback. Preserve
 // timestamp in the ROS caller instead of reproducing the native truncation bug.
 void cloud(const float* data,int n,bool passability=false,bool reference=false){
  using Clock=std::chrono::steady_clock;auto last=Clock::now();
  auto elapsed=[&]{auto now=Clock::now();double ms=std::chrono::duration<double,std::milli>(now-last).count();last=now;return ms;};
  cloud_stats={};cloud_stats[0]=n;
  Cloud::Ptr input(new Cloud),tmp(new Cloud);input->reserve(n);
  for(int i=0;i<n;++i){const float* p=data+3*i;if(std::isfinite(p[0])&&std::isfinite(p[1])&&std::isfinite(p[2])&&
   (reference||passability||(p[0]>=-4.f&&p[0]<=4.f&&p[1]>=-4.f&&p[1]<=4.f&&p[2]>=-.3f&&p[2]<=.6f)))input->push_back({p[0],p[1],p[2]});}
  if(!passability&&reference){
   pcl::PassThrough<pcl::PointXYZ> pass;
   for(const auto& axis:std::array<std::string,3>{"x","y","z"}){
    pass.setInputCloud(input);pass.setFilterFieldName(axis);pass.setFilterLimits(axis=="z"?-.3f:-4.f,axis=="z"?.6f:4.f);pass.filter(*tmp);input.swap(tmp);
   }
  }
  cloud_stats[1]=input->size();cloud_stats[5]=elapsed();
  auto downsample=[&]{pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(input);float leaf=passability?.05f:.01f;voxel.setLeafSize(leaf,leaf,leaf);voxel.filter(*tmp);input.swap(tmp);};
  if(!passability)downsample(); // pcl_pass_grid 0xc6a1c precedes CropBox.
  pcl::CropBox<pcl::PointXYZ> crop;crop.setInputCloud(input);crop.setNegative(true);
  crop.setMin(Eigen::Vector4f(-.45f,-.25f,passability?-.4f:-.3f,1));crop.setMax(Eigen::Vector4f(.45f,.25f,passability?1.f:.6f,1));crop.filter(*tmp);input.swap(tmp);
  if(passability)downsample();
  cloud_stats[2]=input->size();cloud_stats[6]=elapsed();
  if(input->empty())filtered->clear();
  else if(reference){pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;ror.setInputCloud(input);ror.setRadiusSearch(.1);ror.setMinNeighborsInRadius(4);ror.filter(*filtered);}
  else radius_filter(*input,*filtered);
  cloud_stats[3]=filtered->size();cloud_stats[7]=elapsed();
  grid();
  cloud_stats[4]=occupied.size();cloud_stats[8]=elapsed();
  cloud_stats[9]=cloud_stats[5]+cloud_stats[6]+cloud_stats[7]+cloud_stats[8];
 }
 void grid(){
  occupied.clear();if(filtered->empty())return;
  // NOS laserCloudHandler creates a fresh 5 cm grid per cloud. Coordinate
  // origins are aligned downwards to the grid, occupancy requires two returns.
  std::map<std::tuple<int,int,int>,int> counts;
  int xmax=INT_MIN,ymax=INT_MIN,zmax=INT_MIN;
  for(auto p:*filtered){int x=int(std::floor(p.x/.05f)),y=int(std::floor(p.y/.05f)),z=int(std::floor(p.z/.05f));++counts[{x,y,z}];xmax=std::max(xmax,x);ymax=std::max(ymax,y);zmax=std::max(zmax,z);}
  for(auto [key,n]:counts)if(n>=2){auto [x,y,z]=key;pcl::PointXYZ p{(x+.5f)*.05f,(y+.5f)*.05f,(z+.5f)*.05f};
   // 0x94f08..0x94f64 excludes each maximum boundary cell.
   if(x<xmax&&y<ymax&&z<zmax)occupied.push_back(p);
  }
 }
 // Project interface between recovered AOS semantic cost cells and NOS DWA.
 // These are confirmed obstacle cells, not raw returns: neither the >=2-return
 // grid nor its maximum-Z exclusion applies. Preserve each official cost cell.
 void obstacles(const float* data,int n){
  occupied.clear();filtered->clear();
  for(int i=0;i<n;++i){const float* p=data+3*i;
   if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2]))throw std::runtime_error("nonfinite obstacle cell");
   occupied.push_back({p[0],p[1],p[2]});filtered->push_back(occupied.back());
  }
 }
 bool path(const Primitive& p)const{
  // judgePath 0xb9b08: ten library samples, rotated .84 x .50 rectangle.
  for(auto q:p.samples){double c=std::cos(q.yaw),s=std::sin(q.yaw);
   for(auto ob:occupied){double dx=ob.x-q.x,dy=ob.y-q.y;
    if(std::abs(c*dx+s*dy)<=.42&&std::abs(-s*dx+c*dy)<=.25)return false;
   }
  }return true;
 }
 bool rotation(double w)const{
  if(params.ignore_obstacles)return true;
  // judgeRotate 0xbcc54-0xbcf54: two opposed angular sectors inside the
  // circumcircle, with angular extent supplied by the caller (not a rectangle).
  double lo=std::min(0.,w),width=std::abs(w),radius=std::hypot(.42,.25);
  for(auto p:occupied){if(std::hypot(p.x,p.y)>radius)continue;
   double a=std::atan2(p.y,p.x);
   for(double shift:{0.,first::pi}){
    double delta=std::fmod(a-lo-shift+4*first::pi,2*first::pi);
    if(delta<=width)return false;
   }
  }return true;
 }
 bool goal_obstacle(Pose p)const{
  for(auto ob:occupied)if(ob.z<=.5&&std::hypot(ob.x-p.x,ob.y-p.y)<=.25)return true;
  return false;
 }
 static Velocity cap(Velocity v,double x,double y,double w){return {std::clamp(v.x,-x,x),std::clamp(v.y,-y,y),std::clamp(v.w,-w,w)};}
 Output dwa(Pose target,Pose final,bool terminal_allowed,double maxx=1.5,double maxy=.6,double maxw=1.){
  Output out;if(!active)return out;
  if(terminal_allowed&&std::abs(final.x)+std::abs(final.y)<=max_range&&goal_obstacle(final)){out.status=1102;return out;}
  first::Penalties hits;
  // updatePathPenalty 0xb9520-0xb9700: each occupied 3D cell contributes,
  // including repeated XY at distinct heights; table duplicates count too.
  for(auto ob:occupied){
   // b9540..b9578 performs the subtraction/division in float, not double.
   int i=int(std::floor((ob.x-(-1.475f))/.05f)),j=int(std::floor((ob.y-(-1.975f))/.05f));
   if(i<0||i>=90||j<0||j>=80)continue;
   int cell=i*80+j;
   for(int id:library.table[0][cell])++hits.hard[id];
   for(int l=1;l<4;++l)for(int id:library.table[l][cell])hits.soft[l-1][id]=true;
  }
  // runDWA 0xbf550-0xbf5d8: positive vx upper limit and nonzero minimum
  // primitive speeds. Native component maxima are applied to output later.
  for(auto p:library.paths){auto v=p.velocity;
   auto small=[](double a){a=std::abs(a);return a>.05&&a<.25;};
   if(float(v.x)>maxx||small(float(v.x))||small(float(v.y))||small(float(v.w)))hits.hard[p.id]=100;
  }
  std::array<double,first::count> costs;
  costs.fill(std::numeric_limits<double>::infinity());
  for(auto p:library.paths){if(hits.hard[p.id]>=1){++out.hard;continue;}
   // b9f9c..ba0d8: float endpoint distance; double cost compared with a
   // float incumbent. Sorting float costs changes near-tie selection.
   float dx=float(p.end.x-target.x),dy=float(p.end.y-target.y);
   double s=double(std::sqrt(std::fma(dx,dx,dy*dy)))+.5*std::abs(first::wrap(std::atan2(target.y,target.x)-p.end.yaw));
   s=std::fma(1.,double(hits.soft[0][p.id]),s);
   s=std::fma(.8,double(hits.soft[1][p.id]),s);
   s=std::fma(.6,double(hits.soft[2][p.id]),s);
   s=std::fma(.2,std::abs(double(float(p.velocity.y))),s);
   if(target.x>0&&float(p.velocity.x)<=.05)s+=1.;
   costs[p.id]=s;
  }
  for(;;){
   float best=9999.f;int id=-1;
   for(int i=0;i<first::count;++i)if(costs[i]<double(best)){best=float(costs[i]);id=i;}
   if(id<0)break;
   if(!path(library.paths[id])){costs[id]=std::numeric_limits<double>::infinity();if(++out.retries>100)break;continue;}
   out.id=id;out.score=best;break;
  }
  // No feasible path is retried from fresh evidence next cycle; no native
  // escalating reverse-recovery routine was found in runDWA.
  if(out.id<0){out.status=1101;return out;}
  Velocity v=library.paths[out.id].velocity;out.status=1001;
  double distance=std::hypot(final.x,final.y);
  if(terminal_allowed&&distance<=.5){
   if(distance>params.xy){
    v={final.x,final.y,0};double k=.5;
    while(std::abs(v.x)<params.minx&&std::abs(v.y)<params.miny){k+=.05;v={k*final.x,k*final.y,0};}
   }else if(std::abs(first::wrap(final.yaw))<=params.yaw){active=false;out.status=999;return out;}
   else{double a=first::wrap(final.yaw);v={0,0,std::abs(a)<params.minw?std::copysign(params.minw,a):1.5*a};if(!rotation(v.w)){out.status=1002;return out;}}
  }
  v=cap(v,maxx,maxy,maxw);
  // bdf0c..bdf20: a zero primitive with an unfinished goal is blocked.
  if(v.x==0&&v.y==0&&v.w==0){out.status=1101;return out;}
  out.x=float(v.x);out.y=float(v.y);out.w=float(v.w);return out;
 }
 double smooth_x(double desired)const{
  // computeSmoothXVel 0xc54e0. The log text mentions a difference threshold,
  // but the machine branch at c5554 tests difference > 0, not > 1.
  if(std::abs(desired)<params.minx&&std::abs(desired)>.01)desired=std::copysign(params.minx,desired);
  double result=desired;
  if(std::abs(desired)>=1.&&desired-previous_x>0){
   if(std::abs(previous_x)<1.)result=std::copysign(1.,desired);
   else result=desired>0?std::min(desired,previous_x+.05):std::max(desired,previous_x-.05);
  }
  if(result*desired<0)result=std::abs(desired)>params.minx?std::copysign(params.minx,desired):0.;
  return result;
 }
 Output line(Pose final,bool backward=false){
  Output out;if(!active)return out;
  double distance=std::hypot(final.x,final.y),bearing=first::wrap(std::atan2(final.y,final.x)-(backward?first::pi:0)),heading=first::wrap(final.yaw);
  Velocity v;auto turning=[&](double angle,int status){
   double w=std::abs(angle)<params.minw?std::copysign(params.minw,angle):1.5*angle;
   if(!rotation(w)){rotate_wait=10;out.status=2004;return;}
   if(rotate_wait>0){--rotate_wait;out.status=2004;return;}
   v.w=w;out.status=status;
  };
  // c7580 loads 0xd4288 = .05, despite the YAML's .2 parameter.
  if(std::abs(bearing)>=.05&&line_phase!=1){line_phase=2;turning(bearing,2001);}
  else{
   line_phase=1;bool obstacle=false;
   if(!params.ignore_obstacles)for(auto p:occupied){double x=backward?-p.x:p.x;if(x>=.42&&x<=.42+std::min(distance,params.stop)&&std::abs(p.y)<=.25){obstacle=true;break;}}
   if(obstacle){
    out.status=2005;line_wait=10;
    // handleApproach 0xc6b34-0xc6c58: effective default 5 deg, .35 rad/s.
    if(std::abs(bearing)>5*first::pi/180&&rotation(bearing)){v.w=std::copysign(.35,bearing);out.status=2015;}
   }else if(line_wait>0){--line_wait;out.status=2005;}
   else if(distance<=params.xy){
    line_close=true;
    if(std::abs(heading)<=params.yaw){active=false;out.status=999;}
    else turning(heading,2003);
   }else if(distance<.75||line_close){
    v={smooth_x(params.deceleration*final.x),params.deceleration*final.y,0};double k=.5;
    for(int i=0;i<190&&std::abs(v.y)<params.miny;++i){k+=.05;v.y=k*final.y;}
    out.status=2003;
    if(line_close){turning(heading,2003);if(out.status==2004)v={};}
   }else{
    v.x=smooth_x(params.deceleration*final.x);v.w=std::abs(bearing)<params.linew?std::copysign(params.linew,bearing):1.5*bearing;out.status=2002;
   }
  }
  v=cap(v,params.maxx,params.maxy,params.maxw);
  previous_x=v.x;out.x=float(v.x);out.y=float(v.y);out.w=float(v.w);return out;
 }
};
thread_local std::string error;
}
extern "C" {
const char* nos_error(){return recovered::error.c_str();}
void* nos_create(const char* p){try{return new recovered::Planner(p);}catch(const std::exception& e){recovered::error=e.what();return nullptr;}}
void nos_destroy(void* p){delete static_cast<recovered::Planner*>(p);}
int nos_params(void* p,const double* v,int n){
 if(n!=14)return 0;
 for(int i=0;i<n;++i)if(!std::isfinite(v[i])||(i<11&&v[i]<=0))return 0;
 if(v[0]>2||v[1]>1||v[2]>1)return 0;
 auto& planner=*static_cast<recovered::Planner*>(p);planner.reset();
 planner.params={v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8],v[9],v[10],v[11]!=0,v[12]!=0,v[13]!=0};return 1;
}
void nos_goal(void* p){static_cast<recovered::Planner*>(p)->goal();}
void nos_cancel(void* p){static_cast<recovered::Planner*>(p)->reset();}
int nos_cloud(void* p,const float* xyz,int n,int passability){try{if(n<0||n>2000000)throw std::runtime_error("cloud size");static_cast<recovered::Planner*>(p)->cloud(xyz,n,passability);return 1;}catch(const std::exception& e){recovered::error=e.what();return 0;}}
int nos_cloud_reference(void* p,const float* xyz,int n,int passability){try{if(n<0||n>2000000)throw std::runtime_error("cloud size");static_cast<recovered::Planner*>(p)->cloud(xyz,n,passability,true);return 1;}catch(const std::exception& e){recovered::error=e.what();return 0;}}
void nos_cloud_stats(void* p,double* output){auto& s=static_cast<recovered::Planner*>(p)->cloud_stats;std::copy(s.begin(),s.end(),output);}
int nos_obstacles(void* p,const float* xyz,int n){try{if(n<0||n>100000)throw std::runtime_error("obstacle count");static_cast<recovered::Planner*>(p)->obstacles(xyz,n);return 1;}catch(const std::exception& e){recovered::error=e.what();return 0;}}
int nos_filtered(void* p,float* output,int capacity){auto& c=*static_cast<recovered::Planner*>(p)->filtered;int n=int(c.size());if(capacity<n)return n;for(int i=0;i<n;++i){output[3*i]=c[i].x;output[3*i+1]=c[i].y;output[3*i+2]=c[i].z;}return n;}
int nos_plan(void* p,const double* local,const double* goal,int terminal,double mx,double my,double mw,recovered::Output* output){try{*output=static_cast<recovered::Planner*>(p)->dwa({local[0],local[1],0,local[2]},{goal[0],goal[1],0,goal[2]},terminal,mx,my,mw);return 1;}catch(const std::exception& e){recovered::error=e.what();return 0;}}
int nos_line(void* p,const double* goal,int backward,recovered::Output* output){try{*output=static_cast<recovered::Planner*>(p)->line({goal[0],goal[1],0,goal[2]},backward);return 1;}catch(const std::exception& e){recovered::error=e.what();return 0;}}
}
