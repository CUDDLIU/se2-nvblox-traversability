#pragma once
#include "first.hpp"
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <optional>
namespace first {
struct Key {int x,y,z;bool operator==(const Key&b)const{return x==b.x&&y==b.y&&z==b.z;}};
struct Hash {size_t operator()(const Key&k)const{return uint64_t(uint32_t(k.x))*73856093ULL^uint64_t(uint32_t(k.y))*19349663ULL^uint64_t(uint32_t(k.z))*83492791ULL;}};
struct Point {double x,y,z;};
struct Cell {int state=0;double seen=0;};
class Evidence {
 public:
 static constexpr double resolution=.05;
 static constexpr int nx=200,ny=200,nz=120;
 std::vector<int8_t> values;Key origin{};bool initialized=false;
 std::vector<Point> occupied_points;
 size_t offset(Key k)const{return (size_t(k.x-origin.x)*ny+(k.y-origin.y))*nz+(k.z-origin.z);}
 bool inside(Key k)const{return initialized&&k.x>=origin.x&&k.x<origin.x+nx&&k.y>=origin.y&&k.y<origin.y+ny&&k.z>=origin.z&&k.z<origin.z+nz;}
 void move_window(Point body){
  auto k=key(body);Key start{int(std::floor(k.x/10.))*10-nx/2,int(std::floor(k.y/10.))*10-ny/2,int(std::floor(k.z/10.))*10-nz/2};
  if(initialized&&start==origin)return;
  std::vector<int8_t> next(nx*ny*nz,0);
  if(initialized){int z0=std::max(start.z,origin.z),z1=std::min(start.z+nz,origin.z+nz);
   if(z1>z0)for(int x=std::max(start.x,origin.x);x<std::min(start.x+nx,origin.x+nx);++x)for(int y=std::max(start.y,origin.y);y<std::min(start.y+ny,origin.y+ny);++y){
    size_t dest=(size_t(x-start.x)*ny+y-start.y)*nz+z0-start.z;std::copy_n(values.data()+offset({x,y,z0}),z1-z0,next.data()+dest);}
  }values.swap(next);origin=start;initialized=true;
 }
 const std::vector<Point>& occupied()const{return occupied_points;}
 static Key key(Point p){return {int(std::floor(p.x/resolution)),int(std::floor(p.y/resolution)),int(std::floor(p.z/resolution))};}
 static Point center(Key k){return {(k.x+.5)*resolution,(k.y+.5)*resolution,(k.z+.5)*resolution};}
 // Exact voxel traversal, clipped far endpoints remain FREE; never infer free from silence.
 template<class Visit>static void visit_ray(Point a,Point b,Visit visit){
  Key k=key(a),end=key(b);
  double pos[]={a.x,a.y,a.z},delta[]={b.x-a.x,b.y-a.y,b.z-a.z};int idx[]={k.x,k.y,k.z},finish[]={end.x,end.y,end.z},step[3];double next[3],increment[3];
  for(int i=0;i<3;++i){step[i]=(delta[i]>0)-(delta[i]<0);increment[i]=step[i]?resolution/std::abs(delta[i]):INFINITY;
   next[i]=(step[i]&&idx[i]!=finish[i])?((idx[i]+(step[i]>0))*resolution-pos[i])/delta[i]:INFINITY;}
  for(int n=0;n<1000;++n){k={idx[0],idx[1],idx[2]};visit(k);if(k==end)return;
   int axis=int(std::min_element(next,next+3)-next);idx[axis]+=step[axis];next[axis]=(idx[axis]==finish[axis])?INFINITY:next[axis]+increment[axis];}
  throw std::runtime_error("ray traversal limit");
 }
 static std::vector<Key> ray(Point a,Point b){
  std::vector<Key> out;out.reserve(256);visit_ray(a,b,[&](Key k){out.push_back(k);});return out;
 }
 void clear_ray(Point a,Point b){
  auto begin=key(a),end=key(b);
  if(!inside(begin)||!inside(end)){
   visit_ray(a,b,[&](Key k){if(inside(k)){auto&v=values[offset(k)];if(v!=2)v=-1;}});return;
  }
  // A segment with both endpoints in the rectangular grid stays inside it.
  // Increment its linear address with the DDA axis instead of checking bounds
  // and multiplying indices at every visited voxel. Tie ordering is unchanged.
  double pos[]={a.x,a.y,a.z},delta[]={b.x-a.x,b.y-a.y,b.z-a.z};
  int idx[]={begin.x,begin.y,begin.z},finish[]={end.x,end.y,end.z},step[3];
  ptrdiff_t stride[3];double next[3],increment[3];
  for(int i=0;i<3;++i){step[i]=(delta[i]>0)-(delta[i]<0);stride[i]=step[i]*(i==0?ny*nz:i==1?nz:1);
   increment[i]=step[i]?resolution/std::abs(delta[i]):INFINITY;
   next[i]=(step[i]&&idx[i]!=finish[i])?((idx[i]+(step[i]>0))*resolution-pos[i])/delta[i]:INFINITY;
  }
  ptrdiff_t address=offset(begin);
  for(int n=0;n<1000;++n){auto&v=values[address];if(v!=2)v=-1;
   if(idx[0]==finish[0]&&idx[1]==finish[1]&&idx[2]==finish[2])return;
   int axis=next[1]<next[0]?1:0;if(next[2]<next[axis])axis=2;
   idx[axis]+=step[axis];address+=stride[axis];next[axis]=idx[axis]==finish[axis]?INFINITY:next[axis]+increment[axis];
  }
  throw std::runtime_error("ray traversal limit");
 }
 void ingest(const std::vector<std::pair<Point,Point>>&rays,Point body,double /*time*/){
  move_window(body);std::vector<std::pair<Point,Point>> clipped;clipped.reserve(rays.size());
  std::vector<Key> hits;hits.reserve(rays.size());
  // Mark this scan's endpoints first. State 2 prevents any same-scan free ray
  // from clearing a hit, including a return from the other physical sensor.
  for(auto [a,b]:rays){double dx=b.x-a.x,dy=b.y-a.y,dz=b.z-a.z,d=std::sqrt(dx*dx+dy*dy+dz*dz);if(!std::isfinite(d)||d<.01)continue;
   if(d<=4.){auto k=key(b);if(inside(k)){values[offset(k)]=2;hits.push_back(k);}}
   else b={a.x+4*dx/d,a.y+4*dy/d,a.z+4*dz/d};
   clipped.emplace_back(a,b);
  }
  for(auto [a,b]:clipped)clear_ray(a,b);
  // Only prior occupied cells and this scan's endpoints can remain occupied.
  // Preserve their exact values without scanning all 4.8 million grid cells.
  std::vector<Point> next;next.reserve(occupied_points.size()+hits.size());
  for(auto p:occupied_points){auto k=key(p);if(inside(k)&&values[offset(k)]>0){values[offset(k)]=1;next.push_back(p);}}
  for(auto k:hits){auto&v=values[offset(k)];if(v==2){v=1;next.push_back(center(k));}}
  occupied_points.swap(next);
 }
 int state(Point p)const{auto k=key(p);return inside(k)?values[offset(k)]:0;}

};
struct Support {double z;int component;};
class Terrain {
 public:
 static constexpr double resolution=.05;
 std::unordered_map<Key,std::vector<Support>,Hash> cells;
 struct Triangle {std::array<Point,3> p;int component;};
 std::vector<Triangle> triangles;
 std::unordered_map<Key,std::vector<size_t>,Hash> index;
 // Only the planning thread queries this cache. Store all intersecting static
 // surfaces, never the live selected floor or obstacle classification.
 mutable std::unordered_map<Key,std::vector<Support>,Hash> center_cache;
 void load(const std::string&path){center_cache.clear();std::ifstream f(path);std::string magic;int version;double r;f>>magic>>version>>r;if(magic!="FIRST_TERRAIN"||version!=1||std::abs(r-resolution)>1e-9)throw std::runtime_error("invalid terrain version");
  int x,y,c;double z;while(f>>x>>y>>z>>c){if(!std::isfinite(z)||c<0)throw std::runtime_error("invalid support");cells[{x,y,0}].push_back({z,c});}if(!f.eof()||cells.empty())throw std::runtime_error("invalid terrain cells");
  std::ifstream exact(path+".triangles");if(!exact)return;
  exact>>magic>>version;if(magic!="FIRST_TRIANGLES"||version!=1)throw std::runtime_error("invalid triangle version");
  Triangle t;while(exact>>t.p[0].x>>t.p[0].y>>t.p[0].z>>t.p[1].x>>t.p[1].y>>t.p[1].z>>t.p[2].x>>t.p[2].y>>t.p[2].z>>t.component){
   double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY;
   for(auto p:t.p){if(!std::isfinite(p.x+p.y+p.z))throw std::runtime_error("invalid triangle");xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);}
   if(xmax-xmin>2||ymax-ymin>2)throw std::runtime_error("unexpected large support triangle");
   for(int x=int(std::floor(xmin/resolution));x<=int(std::floor(xmax/resolution));++x)for(int y=int(std::floor(ymin/resolution));y<=int(std::floor(ymax/resolution));++y)index[{x,y,0}].push_back(triangles.size());
   triangles.push_back(t);
  }
  if(!exact.eof()||triangles.empty())throw std::runtime_error("invalid triangles");
 }
 std::optional<Support> at(double x,double y,double previous,int component=-1,double max_step=.201)const{
  Key key{int(std::floor(x/resolution)),int(std::floor(y/resolution)),0};std::optional<Support>best;double distance=max_step;
  if(!triangles.empty()){
   auto it=index.find(key);if(it==index.end())return best;
   for(size_t id:it->second){const auto&t=triangles[id];if(component>=0&&t.component!=component)continue;
    auto a=t.p[0],b=t.p[1],c=t.p[2];double det=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);if(std::abs(det)<1e-12)continue;
    double u=((x-a.x)*(c.y-a.y)-(y-a.y)*(c.x-a.x))/det;
    double v=((b.x-a.x)*(y-a.y)-(b.y-a.y)*(x-a.x))/det;
    if(u< -1e-6||v< -1e-6||u+v>1+1e-6)continue;
    double z=a.z+u*(b.z-a.z)+v*(c.z-a.z);if(std::abs(z-previous)<distance){distance=std::abs(z-previous);best=Support{z,t.component};}
   }return best;
  }
  auto i=cells.find(key);if(i==cells.end())return best;
  for(auto&s:i->second)if((component<0||s.component==component)&&std::abs(s.z-previous)<distance){distance=std::abs(s.z-previous);best=s;}
  return best;
 }
 // All footprint samples must have measured support in the same connected level.
 std::optional<Support> at_voxel_center(Key k,double previous,int component=-1,double max_step=.201)const{
  k.z=0;auto p=Evidence::center(k);
  if(triangles.empty())return at(p.x,p.y,previous,component,max_step);
  auto it=center_cache.find(k);
  if(it==center_cache.end()){
   auto row=index.find(k);if(row==index.end())return {};
   it=center_cache.try_emplace(k).first;
   for(size_t id:row->second){const auto&t=triangles[id];auto a=t.p[0],b=t.p[1],c=t.p[2];
   double det=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);if(std::abs(det)<1e-12)continue;
   double u=((p.x-a.x)*(c.y-a.y)-(p.y-a.y)*(c.x-a.x))/det;
   double v=((b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x))/det;
   if(u< -1e-6||v< -1e-6||u+v>1+1e-6)continue;
   it->second.push_back({a.z+u*(b.z-a.z)+v*(c.z-a.z),t.component});
   }
  }
  std::optional<Support> best;double distance=max_step;
  for(auto s:it->second)if((component<0||s.component==component)&&std::abs(s.z-previous)<distance){distance=std::abs(s.z-previous);best=s;}
  return best;
 }
 // All footprint samples must have measured support in the same connected level.
 // No polygon yaw certificates or saved obstacle surfaces enter this query.
 bool footprint(Pose p,int component)const{
  double c=std::cos(p.yaw),s=std::sin(p.yaw);
  // Match the established ASA footprint sampling: exact physical dimensions,
  // 10 cm support samples. Occupancy remains 5 cm; it is not a support grid.
  constexpr int nx=9,ny=6;std::array<double,ny+1> previous{},current{};
  double xx=0,yy=0,xz=0,yz=0;
  for(int i=0;i<=nx;++i){double x=-.41+.82*i/nx;
   for(int j=0;j<=ny;++j){double y=-.253+.506*j/ny;
    auto q=at(p.x+c*x-s*y,p.y+s*x+c*y,p.z,component,.401);if(!q)return false;
    if((i&&std::abs(q->z-previous[j])>.200001)||(j&&std::abs(q->z-current[j-1])>.200001))return false;
    current[j]=q->z;xx+=x*x;yy+=y*y;xz+=x*q->z;yz+=y*q->z;
   }previous=current;
  }
  return std::hypot(xz/xx,yz/yy)<=std::tan(40*pi/180)+1e-6;
 }
};
inline bool intersects(Pose p,Point q,double height){
 double dx=q.x-p.x,dy=q.y-p.y,c=std::cos(p.yaw),s=std::sin(p.yaw);
 // Rectangle plus voxel half-diagonal covers the actual .82 x .506 footprint.
 return std::abs(c*dx+s*dy)<=.41+.03536&&std::abs(-s*dx+c*dy)<=.253+.03536&&q.z>p.z+.10&&q.z<p.z+height+.03536;
}
// Broad phase only: exact footprint/height predicate and voxel padding are
// identical to intersects(). Avoid rescanning the whole cloud at each sample.
class ObstacleIndex {
 std::unordered_map<Key,std::vector<Point>,Hash> bins_;
 static int bin(double x){return int(std::floor(x/.5));}
 public:
 explicit ObstacleIndex(const std::vector<Point>&points){
  for(auto p:points)bins_[{bin(p.x),bin(p.y),0}].push_back(p);
 }
 bool collides(Pose p,double height)const{
  double c=std::cos(p.yaw),s=std::sin(p.yaw),hx=.41+.03536,hy=.253+.03536;
  double ex=std::abs(c)*hx+std::abs(s)*hy,ey=std::abs(s)*hx+std::abs(c)*hy;
  for(int x=bin(p.x-ex-1e-9);x<=bin(p.x+ex+1e-9);++x)
   for(int y=bin(p.y-ey-1e-9);y<=bin(p.y+ey+1e-9);++y){
    auto it=bins_.find({x,y,0});if(it==bins_.end())continue;
    for(auto q:it->second){double dx=q.x-p.x,dy=q.y-p.y;
     if(std::abs(c*dx+s*dy)<=hx&&std::abs(-s*dx+c*dy)<=hy&&q.z>p.z+.10&&q.z<p.z+height+.03536)return true;
    }
   }
  return false;
 }
};
// Interpolate primitive at <=2.5cm swept distance, including yaw sweep. Evaluate
// braking extension at the same commanded velocity conservatively (no speed boost).
inline std::vector<Pose> sweep(const Primitive&p,Pose base,double extra=0){
 std::vector<Pose> out{base};double c=std::cos(base.yaw),s=std::sin(base.yaw);Pose last{};
 auto append=[&](Pose next){int n=std::max(1,int(std::ceil((std::hypot(next.x-last.x,next.y-last.y)+.49*std::abs(next.yaw-last.yaw))/.025)));
  for(int i=1;i<=n;++i){double u=double(i)/n;double x=last.x+u*(next.x-last.x),y=last.y+u*(next.y-last.y);out.push_back({base.x+c*x-s*y,base.y+s*x+c*y,base.z,base.yaw+last.yaw+u*(next.yaw-last.yaw)});}last=next;};
 for(auto q:p.samples)append(q);
 for(double t=0;t<extra-1e-9;t+=.05){double dt=std::min(.05,extra-t),a=last.yaw;auto q=last;q.x+=dt*(std::cos(a)*p.velocity.x-std::sin(a)*p.velocity.y);q.y+=dt*(std::sin(a)*p.velocity.x+std::cos(a)*p.velocity.y);q.yaw+=dt*p.velocity.w;append(q);}return out;
}
}
