#include <nvblox_ros/conversions/pointcloud_conversions.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace nvblox;
using Cloud=sensor_msgs::msg::PointCloud2;
void require(bool b,const char* message) {if(!b) throw std::runtime_error(message);}
std::shared_ptr<Cloud> cloud(const std::vector<Vector3f>& points, int height=1, bool big=false) {
  auto c=std::make_shared<Cloud>(); c->width=points.size()/height; c->height=height;
  c->header.frame_id="lidar_link";
  c->point_step=26; c->row_step=c->width*26+7; c->is_bigendian=big;
  c->data.resize(c->height*c->row_step,0xA5);
  for (int j=0;j<3;++j) {
    sensor_msgs::msg::PointField f;f.name=std::string(1,"xyz"[j]);
    f.offset=2+j*5; f.count=1; f.datatype=f.FLOAT32;c->fields.push_back(f);
  }
  for(size_t i=0;i<points.size();++i) for(int j=0;j<3;++j) {
    float v=points[i][j];uint32_t bits;std::memcpy(&bits,&v,4);
    if(big) bits=__builtin_bswap32(bits);
    auto* dst=c->data.data()+(i/c->width)*c->row_step+(i%c->width)*26+2+j*5;
    std::memcpy(dst,&bits,4);
  }
  return c;
}
void verify(conversions::PointcloudConverter& converter, const Lidar& model,
            const std::vector<Vector3f>& points,int height=1,bool big=false) {
  DepthImage image(MemoryType::kDevice);std::vector<float> expected(model.numel(),0),actual(model.numel());
  for(const auto& p:points) {
    Index2D uv;
    if(!p.allFinite() || !model.isInValidRange(p) || !model.project(p,&uv)) continue;
    float d=model.getDepth(p);auto& old=expected[uv.y()*model.cols()+uv.x()];
    if(d>0 && std::isfinite(d) && (!old || d<old)) old=d;
  }
  for(int repeat=0;repeat<5;++repeat) {
    converter.depthImageFromPointcloudGPU(cloud(points,height,big),model,&image);
    image.copyTo(actual.data());
    for(size_t i=0;i<actual.size();++i)
      require(std::abs(actual[i]-expected[i])<1e-4f,"GPU differs from nearest-range CPU reference");
  }
}
int main() {
  conversions::PointcloudConverter converter;
  Lidar model(360,64,.1f,20.f,3.14f);
  float nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
  std::vector<Vector3f> small={{1,0,0},{4,0,0},{2,0,0},{nan,0,0},{inf,0,0},{0,0,0},{30,0,0},{0,2,1}};
  verify(converter,model,small,2);verify(converter,model,small,2,true);
  verify(converter,model,{});
  std::vector<Vector3f> large;
  for(int i=0;i<110837;++i) {
    // Non-boundary angular centers; count exceeds the number of image pixels.
    int x=(i*37)%model.cols(),y=(i*13)%model.rows();
    large.push_back(model.unprojectFromPixelIndices(Index2D(x,y),1.f+(i%77)*.02f));
  }
  verify(converter,model,large);
  auto msg=cloud(large);DepthImage image(MemoryType::kDevice);
  for(int i=0;i<5;++i) converter.depthImageFromPointcloudGPU(msg,model,&image);
  auto start=std::chrono::steady_clock::now();
  for(int i=0;i<100;++i) converter.depthImageFromPointcloudGPU(msg,model,&image);
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/100;
  auto malformed=cloud(small);malformed->data.resize(2);
  bool rejected=false;try {converter.depthImageFromPointcloudGPU(malformed,model,&image);} catch(const std::invalid_argument&) {rejected=true;}
  require(rejected,"malformed dimensions not rejected");
  setenv("SE2_LIDAR_SELF_FILTER","1",1);
  auto self=cloud({Vector3f(.4f,0,0),Vector3f(1.f,0,0)});
  converter.depthImageFromPointcloudGPU(self,model,&image);
  std::vector<float> self_depth(model.numel());image.copyTo(self_depth.data());
  Index2D pixel;model.project(Vector3f(1,0,0),&pixel);
  require(std::abs(self_depth[pixel.y()*model.cols()+pixel.x()]-1.f)<1e-6,"robot return was not removed");
  // The same body envelope must be applied after translating the cloud origin.
  for(int sensor=0;sensor<2;++sensor)for(bool axes:{false,true})for(bool optical:{false,true}){
    if(optical&&!axes)continue;
    const float origin_x=optical?.36560f:.32028f;
    Vector3f origin(sensor==0?origin_x:-origin_x,0.f,-.013f);
    auto convert=[&](const Vector3f& body)->Vector3f{
      Vector3f v=body-origin;
      return axes?Vector3f(v.z(),sensor==0?-v.y():v.y(),sensor==0?v.x():-v.x()):v;
    };
    auto translated=cloud({convert(Vector3f(.4f,0,0)),convert(Vector3f(1.f,0,0))});
    translated->header.frame_id=optical?(sensor==0?"nvblox_replay/front_optical_sensor":"nvblox_replay/rear_optical_sensor"):
        (axes?(sensor==0?"nvblox_replay/front_sensor":"nvblox_replay/rear_sensor"):
        (sensor==0?"nvblox_replay/front_origin":"nvblox_replay/rear_origin"));
    converter.depthImageFromPointcloudGPU(translated,model,&image);image.copyTo(self_depth.data());
    int populated=0;for(float d:self_depth)if(d>0)++populated;
    require(populated==1,"translated-origin body filter changed physical envelope");
    Vector3f exterior=convert(Vector3f(1.f,0,0));model.project(exterior,&pixel);
    require(std::abs(self_depth[pixel.y()*model.cols()+pixel.x()]-exterior.norm())<1e-6,
      "translated physical range incorrect");
  }
  self->header.frame_id="unverified_sensor_frame";rejected=false;
  try {converter.depthImageFromPointcloudGPU(self,model,&image);}catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"body filter accepted an unverified frame");
  unsetenv("SE2_LIDAR_SELF_FILTER");
  std::cout<<"PASS: nearest collisions, actual count, invalid points, endian, row padding, malformed cloud; 110837 points mean_ms="<<ms<<std::endl;
}
