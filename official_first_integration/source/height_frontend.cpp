// AOS height_map_nav cloud front end, reconstructed from instructions.
// syncCallback 0x92a00, findImu 0x8e148, getW2G 0x8c730,
// rayTracing 0x8d5f8, processPointCloudRange 0x8d1f8.
// Captured configuration only: no interpolation/image/depth debug branches.
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

namespace aos {
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
using Octree=pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>;
using Q=Eigen::Quaternionf;
thread_local std::string error;
Cloud::Ptr read(const float* data,int n) {
  if(n<0 || (n && !data)) throw std::invalid_argument("invalid cloud");
  Cloud::Ptr p(new Cloud); p->reserve(n);
  for(int i=0;i<n;++i) {
    if(!std::isfinite(data[3*i]) || !std::isfinite(data[3*i+1]) || !std::isfinite(data[3*i+2]))
      continue; // Transport sanitation, outside the recovered finite-input kernel.
    p->push_back(pcl::PointXYZ(data[3*i],data[3*i+1],data[3*i+2]));
  }
  return p;
}
Q quat(const float* p) {return Q(p[3],p[0],p[1],p[2]);}
Eigen::Matrix4f transform(const float* pose) {
  Eigen::Matrix4f m=Eigen::Matrix4f::Identity();
  m.block<3,3>(0,0)=quat(pose+3).toRotationMatrix();
  m.block<3,1>(0,3)=Eigen::Map<const Eigen::Vector3f>(pose); return m;
}
Q gravity(const Q& q) {
  const Eigen::Vector3f x=q*Eigen::Vector3f::UnitX();
  Q heading(Eigen::AngleAxisf(std::atan2(x.y(),x.x()),Eigen::Vector3f::UnitZ()));
  return heading.inverse()*q;
}
void voxel(Cloud::Ptr& p,float xy,float z) {
  if(xy==0.f || z==0.f || p->empty()) return;
  pcl::VoxelGrid<pcl::PointXYZ> f; f.setInputCloud(p); f.setLeafSize(xy,xy,z);
  Cloud::Ptr out(new Cloud); f.filter(*out); p=out;
}
Cloud::Ptr outer(const Cloud::Ptr& p) {
  Cloud::Ptr out(new Cloud(*p)); pcl::PassThrough<pcl::PointXYZ> f;
  for(const auto* axis: {"x","y","z"}) {
    f.setInputCloud(out); f.setFilterFieldName(axis);
    f.setFilterLimits(axis[0]=='z'?-1.5f:-4.f,axis[0]=='z'?.5f:4.f);
    f.filter(*out);
  }
  return out;
}
bool ray(const pcl::PointXYZ& p,bool parallel,Eigen::Vector3f& origin,Eigen::Vector3f& end) {
  if(double(std::abs(p.y))>.2 && double(std::abs(p.x))<=.4) return false;
  origin=Eigen::Vector3f(parallel?(p.x<0.f?-.4f:.4f):0.f,0.f,0.f);
  float dx=p.x-origin.x();
  float r=std::sqrt(std::fma(p.z,p.z,std::fma(dx,dx,p.y*p.y)));
  if(double(r)<=.1) return false;
  float scale=(r-.1f)/r;
  end=Eigen::Vector3f(std::fma(dx,scale,origin.x()),p.y*scale,p.z*scale);
  return true;
}
bool remove(const pcl::PointXYZ& p,const Eigen::Vector3f& end,bool parallel) {
  if(!(std::abs(p.x)<std::abs(end.x()) && std::abs(p.y)<std::abs(end.y()) && std::abs(p.z)<std::abs(end.z()))) return false;
  if(!parallel) return true;
  float origin=p.x>0.f?.4f:-.4f, dx=p.x-origin;
  float norm=std::sqrt(std::fma(dx,dx,p.y*p.y));
  if(double(norm)>=.3) return true;
  float ex=end.x()-origin;
  float dot=std::fma(end.z(),0.f,std::fma(dx,ex,end.y()*p.y));
  float b=std::sqrt(std::fma(end.z(),end.z(),std::fma(ex,ex,end.y()*end.y())));
  // Native b.gt leaves NaN angles on the erase branch. Do not clamp cosine.
  return !(std::acos(dot/(norm*b))>.1f);
}
Cloud::Ptr clear(const Cloud::Ptr& now,const Cloud::Ptr& old,bool parallel) {
  Cloud::Ptr out(new Cloud); if(old->empty()) return out;
  Octree tree(double(.1f)); tree.setInputCloud(old); tree.addPointsFromInputCloud();
  std::vector<unsigned char> removed(old->size()); std::vector<int> indices;
  for(const auto& p:*now) {
    Eigen::Vector3f origin,end; if(!ray(p,parallel,origin,end)) continue;
    indices.clear(); tree.getIntersectedVoxelIndices(origin,end-origin,indices,4);
    for(int i:indices) if(remove((*old)[i],end,parallel)) removed[i]=1;
  }
  out->reserve(old->size());
  for(size_t i=0;i<old->size();++i) {
    const auto& p=(*old)[i];
    bool keep=!parallel && (std::abs(p.x)<=.5f || (std::abs(p.x)<=1.f && double(std::abs(p.y))>=.4 && double(std::abs(p.y))<=.8));
    if(!removed[i] || keep) out->push_back(p);
  }
  return out;
}
struct Imu {double stamp;Q q;};
struct Frontend {
  std::deque<Imu> imu;
  Q latest=Q::Identity(),aligned=Q::Identity();
  Cloud::Ptr body{new Cloud}, ground{new Cloud};
  Eigen::Matrix4f previous=Eigen::Matrix4f::Identity();
  bool first=true,matched=false,accumulate=true,casting=true,parallel=true;
  float pitch=0.f;
  bool find(double t) {
    for(size_t i=0;i+1<imu.size();++i) {
      if(imu[i].stamp<=t && imu[i+1].stamp>=t) {
        Q a=imu[i].q,b=imu[i+1].q; a.normalize();b.normalize();
        aligned=a.slerp(float((t-imu[i].stamp)/(imu[i+1].stamp-imu[i].stamp)),b);
        imu.erase(imu.begin(),imu.begin()+i+1); return true;
      }
    }
    return false;
  }
  void process(const float* p,int n,double stamp,const float* pose) {
    Cloud::Ptr now=read(p,n); voxel(now,.05f,.05f);
    pcl::CropBox<pcl::PointXYZ> crop;crop.setInputCloud(now);crop.setNegative(true);
    crop.setMin(Eigen::Vector4f(-.4f,-.2f,-.5f,1.f));crop.setMax(Eigen::Vector4f(.4f,.2f,.4f,1.f));crop.filter(*now);
    Cloud::Ptr limited=outer(now); Eigen::Matrix4f current=transform(pose);
    if(!accumulate) body=limited;
    else if(first) {body=limited;first=false;}
    else {
      Cloud::Ptr moved(new Cloud); Eigen::Matrix4f delta=current.inverse()*previous;
      pcl::transformPointCloud(*body,*moved,delta);
      body=casting?clear(now,moved,parallel):moved;
      *body+=*limited;
    }
    body=outer(body);voxel(body,.05f,.05f);
    matched=find(stamp);
    if(matched) {
      Eigen::Matrix4f g=Eigen::Matrix4f::Identity();g.block<3,3>(0,0)=gravity(aligned).toRotationMatrix();
      pcl::transformPointCloud(*body,*ground,g);
    } else *ground=*body;
    previous=current;
    // Builder uses the latest received IMU, distinct from the interpolated IMU.
    pitch=latest.toRotationMatrix().eulerAngles(2,1,0)[1];
  }
};
int write(const Cloud& p,float* out,int capacity) {
  if(!out) return int(p.size());
  if(capacity<int(p.size())) throw std::invalid_argument("cloud output capacity");
  for(size_t i=0;i<p.size();++i) {out[3*i]=p[i].x;out[3*i+1]=p[i].y;out[3*i+2]=p[i].z;}
  return int(p.size());
}
}
extern "C" {
const char* aos_error(){return aos::error.c_str();}
void* aos_create(){try{return new aos::Frontend;}catch(const std::exception& e){aos::error=e.what();return nullptr;}}
void aos_destroy(void* h){delete static_cast<aos::Frontend*>(h);}
void aos_options(void* h,int accumulate,int casting,int parallel){auto& f=*static_cast<aos::Frontend*>(h);f.accumulate=accumulate;f.casting=casting;f.parallel=parallel;}
void aos_imu(void* h,double stamp,const float* q){auto& f=*static_cast<aos::Frontend*>(h);f.latest=aos::quat(q);f.imu.push_back({stamp,f.latest});if(f.imu.size()>200)f.imu.pop_front();}
int aos_process(void* h,const float* p,int n,double stamp,const float* pose){try{static_cast<aos::Frontend*>(h)->process(p,n,stamp,pose);return 1;}catch(const std::exception& e){aos::error=e.what();return 0;}}
int aos_cloud(void* h,float* out,int capacity,int body){try{auto& f=*static_cast<aos::Frontend*>(h);return aos::write(body?*f.body:*f.ground,out,capacity);}catch(const std::exception& e){aos::error=e.what();return -1;}}
int aos_matched(void* h){return static_cast<aos::Frontend*>(h)->matched;}
float aos_pitch(void* h){return static_cast<aos::Frontend*>(h)->pitch;}
void aos_rotation(void* h,float* out){auto& f=*static_cast<aos::Frontend*>(h);Eigen::Map<Eigen::Vector4f> dst(out);dst=(f.matched?aos::gravity(f.aligned):aos::Q::Identity()).coeffs();}
// Offline differential entrypoints use the exact same geometry as production.
int aos_ray_query(const float* old,int n,const float* o,const float* d,int* out){auto p=aos::read(old,n);if(p->empty())return 0;aos::Octree tree(double(.1f));tree.setInputCloud(p);tree.addPointsFromInputCloud();std::vector<int> ids;tree.getIntersectedVoxelIndices(Eigen::Map<const Eigen::Vector3f>(o),Eigen::Map<const Eigen::Vector3f>(d),ids,4);std::copy(ids.begin(),ids.end(),out);return ids.size();}
int aos_clear(const float* now,int n,const float* old,int m,int parallel,float* out){try{return aos::write(*aos::clear(aos::read(now,n),aos::read(old,m),parallel),out,m);}catch(const std::exception& e){aos::error=e.what();return -1;}}
void aos_gravity(const float* q,float* out){Eigen::Map<Eigen::Vector4f> dst(out);dst=aos::gravity(aos::quat(q)).coeffs();}
int aos_interpolate(void* h,double stamp,float* out){auto& f=*static_cast<aos::Frontend*>(h);if(!f.find(stamp))return 0;Eigen::Map<Eigen::Vector4f> dst(out);dst=f.aligned.coeffs();return 1;}
int aos_pcl_filter(const float* p,int n,int mode,const float* args,float* out){auto cloud=aos::read(p,n);if(mode==0)aos::voxel(cloud,args[0],args[2]);else if(mode==1){pcl::CropBox<pcl::PointXYZ> f;f.setInputCloud(cloud);f.setNegative(true);f.setMin(Eigen::Map<const Eigen::Vector4f>(args));f.setMax(Eigen::Map<const Eigen::Vector4f>(args+4));f.filter(*cloud);}else{pcl::PassThrough<pcl::PointXYZ> f;int i=0;for(const char* axis:{"x","y","z"}){f.setInputCloud(cloud);f.setFilterFieldName(axis);f.setFilterLimits(args[2*i+1],args[2*i]);f.filter(*cloud);++i;}}return aos::write(*cloud,out,n);}
int aos_pcl_transform(const float* p,int n,const float* matrix,float* out){auto cloud=aos::read(p,n);aos::Cloud result;Eigen::Matrix4f t=Eigen::Map<const Eigen::Matrix4f>(matrix);pcl::transformPointCloud(*cloud,result,t);return aos::write(result,out,n);}
}
