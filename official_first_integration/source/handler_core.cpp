// Reconstructed NOS handler numerical kernels. Reference SHA/address map is
// recorded by differential_handler.py. No DDS, robot I/O or vendor runtime.
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
namespace handler {
constexpr double pi=3.14159265358979323846;
void boundary(float front,float back,float width,float* out) {
  float half=width*.5f;
  float a=float(double(std::atan2(half,front)*180.f)/pi);
  float b=float(double(std::atan2(half,back)*180.f)/pi);
  float rf=std::sqrt(std::fma(front,front,half*half));
  float rb=std::sqrt(std::fma(back,back,half*half));
  for(int i=0;i<360;++i) {
    if(i<=a || 360-i<=a)out[i]=rf;
    else if(std::abs(i-180)<=b)out[i]=rb;
    else out[i]=half/std::abs(std::sin(float(double(i)*pi/180.)));
  }
}
void step(double* state,const double* velocity,double dt) {
  double s=std::sin(state[2]),c=std::cos(state[2]);
  state[0]+=std::fma(c*velocity[0],dt,-(s*velocity[1]*dt));
  state[1]+=std::fma(s*velocity[0],dt,c*velocity[1]*dt);
  state[2]=std::fma(velocity[2],dt,state[2]);
  std::copy(velocity,velocity+3,state+3);
}
void simpson(const double* before,const double* after,double dt,double* state) {
  double ax=(after[0]-before[0])/dt,ay=(after[1]-before[1])/dt;
  double halfaw=((after[2]-before[2])/dt)*.5;
  double x[3],y[3],theta=0;
  for(int i=0;i<3;++i) {
    double t=i==0 ? 0. : i==1 ? dt*.5 : dt;
    theta=std::fma(t,before[2],(t*halfaw)*t);
    double vx=std::fma(ax,t,before[0]),vy=std::fma(ay,t,before[1]);
    double s=std::sin(theta),c=std::cos(theta);
    x[i]=std::fma(vx,c,-vy*s);y[i]=std::fma(vx,s,vy*c);
  }
  state[0]=(std::fma(x[1],4.,x[0])+x[2])*(dt/6.);
  state[1]=(std::fma(y[1],4.,y[0])+y[2])*(dt/6.);
  state[2]=theta;std::copy(after,after+3,state+3);
}
bool judge(const double* states,int steps,const float* points,int n,
           double front,double back,double width,double pass,const float* table,double& distance) {
  if(!steps)return false;
  bool valid=true;
  for(int i=steps-1;i>=0 && valid;--i) {
    const double* p=states+i*6;
    for(int j=0;j<n;++j) {
      double px=points[j*3],py=points[j*3+1];
      if((p[0]>.5 && px<-.5)||(p[0]<-.5 && px>.5)||
         (p[1]>.5 && py<-.5)||(p[1]<-.5 && py>.5))continue;
      double dx=px-p[0],dy=py-p[1];float fx=dx,fy=dy;
      float r=std::sqrt(std::fma(fx,fx,fy*fy));if(double(r)>pass)continue;
      double s=std::sin(p[2]),c=std::cos(p[2]);
      float x=std::fma(dx,c,dy*s),y=std::fma(-dx,s,dy*c);
      if(double(x)<=front && double(x)>=-back && double(y)<=width*.5 && double(y)>=-width*.5) {valid=false;break;}
      float angle=float(double(std::atan2(y,x)*180.f)/pi);
      int index=(int(std::round(angle))+360)%360;
      double d=float(r-table[index]);if(d<.01)d=double(.01f);
      distance=std::min(distance,d);
    }
  }
  distance=std::min(distance,1.);return valid;
}
// Native piecewise limits: upper segment joins slow1 at slow2_distance to
// maximum at slow1_distance; lower segment joins slow2 at 0 to slow1 at slow2_distance.
void limits(float clearance,double pitch,const double* maxima,const double* minima,
            double d1,double d2,const double* slow1,const double* slow2,double* out) {
  double d=clearance;
  for(int i=0;i<3;++i) {
    double top=maxima[i],bottom=minima[i];
    if(d<d1 && d>=d2) {
      double slope=(maxima[i]-slow1[i])/(d1-d2);
      top=std::fma(d,slope,std::fma(-d1,slope,maxima[i]));bottom=-top;
    } else if(d<d2) {
      double slope=(slow1[i]-slow2[i])/d2;
      top=std::fma(d,slope,std::fma(-d2,slope,slow1[i]));bottom=-top;
    }
    out[2*i]=top;out[2*i+1]=bottom;
  }
  double c=std::cos((pitch/180.)*3.1415926);out[0]*=c*c;out[1]*=c*c;
}
}
extern "C" {
void handler_boundary(float front,float back,float width,float* out) {handler::boundary(front,back,width,out);}
void handler_step(double* state,const double* velocity,double dt) {handler::step(state,velocity,dt);}
void handler_simpson(const double* before,const double* after,double dt,double* state) {handler::simpson(before,after,dt,state);}
int handler_judge(const double* states,int steps,const float* points,int n,const double* shape,const float* table,double* distance) {
 return handler::judge(states,steps,points,n,shape[0],shape[1],shape[2],shape[3],table,*distance);
}
void handler_limits(float clearance,double pitch,const double* maxima,const double* minima,double d1,double d2,const double* slow1,const double* slow2,double* out) {
 handler::limits(clearance,pitch,maxima,minima,d1,d2,slow1,slow2,out);
}
double handler_cost_distance(double d) {return d>1. ? 0. : 1./d-1.;}
}

// Captured handler candidate lattice. cfg: shape[4], dt, steps, Simpson,
// spaces[3], samples[3], weights[x,y,yaw,clearance]. Bounds are max/min pairs.
extern "C" int handler_select(const float* points,int n,const double* desired,
 const double* measured,const double* bounds,const double* cfg,double* velocity,
 double* clearance,double* best_cost,int* accepted) {
 double lo[3],hi[3];int counts[3];
 for(int i=0;i<3;++i) {
   double top=measured[i]>0 ? measured[i]+cfg[7+i] : bounds[2*i];
   double bottom=measured[i]<0 ? measured[i]-cfg[7+i] : bounds[2*i+1];
   hi[i]=std::round(std::clamp(top,bounds[2*i+1],bounds[2*i])*10.)/10.;
   lo[i]=std::round(std::clamp(bottom,bounds[2*i+1],bounds[2*i])*10.)/10.;
   counts[i]=int(std::round((hi[i]-lo[i])/cfg[10+i]));
 }
 float table[360];handler::boundary(cfg[0],cfg[1],cfg[2],table);
 int steps=int(cfg[5]);std::vector<double> trajectory(steps*6);
 bool found=false;float best=1000.f;*accepted=0;
 std::fill(velocity,velocity+3,0.);*clearance=1.;*best_cost=1000.;
 for(int ix=0;ix<=counts[0];++ix)for(int iy=0;iy<=counts[1];++iy)for(int iw=0;iw<=counts[2];++iw) {
   int indices[3]={ix,iy,iw};double v[3],state[6]={};
   for(int i=0;i<3;++i)v[i]=std::clamp(std::fma(cfg[10+i],double(indices[i]),lo[i]),bounds[2*i+1],bounds[2*i]);
   for(int i=0;i<steps;++i) {
     if(i==0 && cfg[6]!=0.)handler::simpson(measured,v,cfg[4],state);
     else handler::step(state,v,cfg[4]);
     std::copy(state,state+6,trajectory.data()+i*6);
   }
   double d=100.;if(!handler::judge(trajectory.data(),steps,points,n,cfg[0],cfg[1],cfg[2],cfg[3],table,d))continue;
   ++*accepted;double cost=std::abs(desired[0]-v[0])*cfg[13];
   cost=std::fma(cfg[16],d>1. ? 0. : 1./d-1.,cost);
   cost=std::fma(cfg[14],std::abs(desired[1]-v[1]),cost);
   cost=std::fma(cfg[15],std::abs(desired[2]-v[2]),cost);
   if(cost<double(best)) {found=true;best=float(cost);*best_cost=cost;*clearance=d;std::copy(v,v+3,velocity);}
 }
 return found;
}

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
// ProcessCloud 0x9d1a8, captured settings; output capacity >= input count.
extern "C" int handler_cloud(const float* xyz,int n,float* output) {
 try {
  using Cloud=pcl::PointCloud<pcl::PointXYZ>;
  auto input=Cloud::Ptr(new Cloud),outer=Cloud::Ptr(new Cloud),body=Cloud::Ptr(new Cloud);
  for(int i=0;i<n;++i)if(std::isfinite(xyz[3*i])&&std::isfinite(xyz[3*i+1])&&std::isfinite(xyz[3*i+2]))
    input->push_back(pcl::PointXYZ(xyz[3*i],xyz[3*i+1],xyz[3*i+2]));
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setMin(Eigen::Vector4f(-3.f,-3.f,-.6f,1.f));crop.setMax(Eigen::Vector4f(3.f,3.f,.3f,1.f));
  crop.setInputCloud(input);crop.filter(*outer);
  crop.setMin(Eigen::Vector4f(-.4f,-.2f,-.6f,1.f));crop.setMax(Eigen::Vector4f(.4f,.2f,.3f,1.f));
  crop.setNegative(true);crop.setInputCloud(outer);crop.filter(*body);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setLeafSize(.1f,.1f,3.f);voxel.setInputCloud(body);
  Cloud filtered;voxel.filter(filtered);
  for(size_t i=0;i<filtered.size();++i){output[3*i]=filtered[i].x;output[3*i+1]=filtered[i].y;output[3*i+2]=filtered[i].z;}
  return int(filtered.size());
 } catch(...) {return -1;}
}
