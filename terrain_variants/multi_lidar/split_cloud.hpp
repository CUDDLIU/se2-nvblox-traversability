#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace se2_multi_lidar {
using Cloud = sensor_msgs::msg::PointCloud2;
constexpr std::array<float,2> kOriginX{.32028f,-.32028f};
constexpr float kOriginZ = -.013f;
constexpr float kOpticalAxialOffset = .04532f;
inline float originX(int sensor,bool optical=false) {
  return kOriginX[sensor]+(optical?(sensor==0?1.f:-1.f)*kOpticalAxialOffset:0.f);
}
constexpr std::array<const char*,2> kFrames{
    "nvblox_replay/front_origin", "nvblox_replay/rear_origin"};
constexpr std::array<const char*,2> kSensorFrames{
    "nvblox_replay/front_sensor", "nvblox_replay/rear_sensor"};
constexpr std::array<const char*,2> kOpticalFrames{
    "nvblox_replay/front_optical_sensor", "nvblox_replay/rear_optical_sensor"};

inline unsigned offset(const Cloud& c, const char* name, int type, unsigned bytes) {
  for (const auto& f:c.fields) if(f.name==name && f.datatype==type && f.count==1 &&
      uint64_t(f.offset)+bytes<=c.point_step) return f.offset;
  throw std::invalid_argument(std::string("invalid point field: ")+name);
}
template<class T> T read(const uint8_t* p,bool swap) {
  std::array<uint8_t,sizeof(T)> b;
  for(size_t i=0;i<b.size();++i)b[i]=p[swap?b.size()-1-i:i];
  T v;std::memcpy(&v,b.data(),sizeof(v));return v;
}

// Input remains immutable. Internal output coordinates are body-aligned but
// translated to each physical origin. They are never published as body clouds.
inline std::array<Cloud::ConstSharedPtr,2> split(const Cloud& source,bool sensor_axes=false,bool optical=false) {
  if(optical && !sensor_axes)throw std::invalid_argument("optical origin requires sensor axes");
  if(source.header.frame_id!="lidar_link" && source.header.frame_id!="base_link_dog")
    throw std::invalid_argument("unverified merged cloud frame");
  uint64_t row_bytes=uint64_t(source.width)*source.point_step;
  uint64_t required=source.height?uint64_t(source.height-1)*source.row_step+row_bytes:0;
  uint64_t count=uint64_t(source.width)*source.height;
  if(row_bytes>source.row_step || required>source.data.size() ||
     count>std::numeric_limits<uint32_t>::max()/12)
    throw std::invalid_argument("malformed merged cloud dimensions");
  const auto ox=offset(source,"x",7,4),oy=offset(source,"y",7,4),
             oz=offset(source,"z",7,4),ring=offset(source,"ring",4,2);
  const uint16_t endian=1;
  const bool swap=source.is_bigendian!=(*reinterpret_cast<const uint8_t*>(&endian)==0);
  std::array<Cloud::SharedPtr,2> result{std::make_shared<Cloud>(),std::make_shared<Cloud>()};
  for(int i=0;i<2;++i){
    auto& out=*result[i];out.header=source.header;
    out.header.frame_id=optical?kOpticalFrames[i]:(sensor_axes?kSensorFrames[i]:kFrames[i]);
    out.height=1;out.point_step=12;out.is_bigendian=false;out.is_dense=false;
    for(int j=0;j<3;++j){sensor_msgs::msg::PointField f;
      f.name=std::array<const char*,3>{"x","y","z"}[j];f.offset=4*j;
      f.datatype=7;f.count=1;out.fields.push_back(f);}
    out.data.reserve(count*6);
  }
  for(size_t row=0;row<source.height;++row)for(size_t col=0;col<source.width;++col){
    const auto* p=source.data.data()+row*source.row_step+col*source.point_step;
    uint16_t r=read<uint16_t>(p+ring,swap);
    if(r>=192)throw std::invalid_argument("ring outside verified 0..191 mapping");
    int sensor=r/96;
    std::array<float,3> xyz{read<float>(p+ox,swap)-originX(sensor,optical),
      read<float>(p+oy,swap),read<float>(p+oz,swap)-kOriginZ};
    if(sensor_axes)xyz={xyz[2],sensor==0?-xyz[1]:xyz[1],sensor==0?xyz[0]:-xyz[0]};
    auto& out=*result[sensor];size_t n=out.data.size();out.data.resize(n+12);
    std::memcpy(out.data.data()+n,xyz.data(),12);++out.width;
  }
  for(auto& out:result)out->row_step=out->width*out->point_step;
  return {result[0],result[1]};
}
}  // namespace se2_multi_lidar
