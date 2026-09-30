#include "split_cloud.hpp"
#include <Eigen/Geometry>
#include <iostream>

using namespace se2_multi_lidar;
void require(bool ok,const char* m){if(!ok)throw std::runtime_error(m);}
Cloud input(bool big=false){
  Cloud c;c.header.frame_id="lidar_link";c.header.stamp.sec=123;
  c.width=2;c.height=2;c.point_step=26;c.row_step=57;c.is_bigendian=big;
  for(int j=0;j<4;++j){sensor_msgs::msg::PointField f;
    f.name=std::array<const char*,4>{"x","y","z","ring"}[j];
    f.offset=std::array<unsigned,4>{0,4,8,16}[j];f.datatype=j==3?4:7;f.count=1;c.fields.push_back(f);}
  c.data.resize(c.row_step*c.height,0xA5);
  for(int i=0;i<4;++i){
    auto* p=c.data.data()+(i/2)*c.row_step+(i%2)*26;
    for(int j=0;j<3;++j){float v=i+1.f+j*.25f;uint32_t b;std::memcpy(&b,&v,4);
      if(big)b=__builtin_bswap32(b);std::memcpy(p+4*j,&b,4);}
    uint16_t ring=std::array<uint16_t,4>{0,95,96,191}[i];
    if(big)ring=__builtin_bswap16(ring);std::memcpy(p+16,&ring,2);
  }
  return c;
}
template<class F>void rejects(F f){bool ok=false;try{f();}catch(const std::invalid_argument&){ok=true;}require(ok,"invalid cloud accepted");}
int main(){
  for(bool big:{false,true})for(bool axes:{false,true})for(bool optical:{false,true}){
    if(optical && !axes)continue;
    auto c=input(big);const auto bytes=c.data;auto parts=split(c,axes,optical);
    require(c.data==bytes,"source mutated");
    Eigen::Isometry3f world_body=Eigen::Isometry3f::Identity();
    world_body.linear()=Eigen::AngleAxisf(.7f,Eigen::Vector3f(1,2,3).normalized()).toRotationMatrix();
    world_body.translation()=Eigen::Vector3f(2,-4,6);
    for(int sensor=0;sensor<2;++sensor){
      const auto& part=*parts[sensor];require(part.width==2&&part.height==1&&part.row_step==24,"wrong split size");
      require(part.header.stamp.sec==123&&part.header.frame_id==(optical?kOpticalFrames[sensor]:(axes?kSensorFrames[sensor]:kFrames[sensor])),"wrong reference header");
      for(int j=0;j<2;++j){Eigen::Vector3f p;std::memcpy(p.data(),part.data.data()+12*j,12);
        const int i=sensor*2+j;Eigen::Vector3f expected(i+1.f,i+1.25f,i+1.5f);
        Eigen::Vector3f body=p;
        if(axes)body=Eigen::Vector3f(sensor==0?p.z():-p.z(),sensor==0?-p.y():p.y(),p.x());
        body+=Eigen::Vector3f(originX(sensor,optical),0,kOriginZ);
        require((world_body*body-world_body*expected).norm()<1e-5,"world endpoint moved after origin split");}
    }
  }
  auto c=input();c.width=1;c.height=1;c.row_step=26;c.data.resize(26);
  auto parts=split(c);require(parts[0]->width==1&&parts[1]->width==0,"front-only scan lost");
  uint16_t bad=192;std::memcpy(c.data.data()+16,&bad,2);rejects([&]{split(c);});
  c=input();c.data.resize(20);rejects([&]{split(c);});
  c=input();c.header.frame_id="unknown";rejects([&]{split(c);});
  c=input();c.fields.pop_back();rejects([&]{split(c);});
  c=input();rejects([&]{split(c,false,true);});
  std::cout<<"PASS split: endpoints invariant, endian, padded rows, sensor separation, front-only, bad ring/frame/layout"<<std::endl;
}
