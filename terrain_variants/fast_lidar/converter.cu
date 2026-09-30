// Local ABI-compatible replacement for nvblox_ros PointcloudConverter.
// Only experiment mapper processes opt in through LD_PRELOAD; system libraries
// are never modified. Build against the installed nvblox headers on the Jetson.
#include <nvblox_ros/conversions/pointcloud_conversions.hpp>
#include <cuda_runtime.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>

namespace nvblox { namespace conversions {
namespace {
void cudaCheck(cudaError_t status) {
  if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
int offset(const sensor_msgs::msg::PointCloud2& msg, const char* name) {
  for (const auto& f : msg.fields) if (f.name == name) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32 || f.count != 1 ||
        uint64_t(f.offset) + sizeof(float) > msg.point_step)
      throw std::invalid_argument("fast_lidar requires scalar FLOAT32 XYZ fields");
    return f.offset;
  }
  throw std::invalid_argument("fast_lidar missing XYZ field");
}
float readFloat(const uint8_t* p, bool swap) {
  uint32_t bits;
  std::memcpy(&bits, p, sizeof(bits));
  if (swap) bits = __builtin_bswap32(bits);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
// Zero means unknown. Retain the nearest valid surface when beams collide in
// one angular pixel; concurrent GPU writes must not select a random surface.
__global__ void projectNearest(const Vector3f* points, size_t count,
                               Lidar lidar, bool self_filter, Vector3f body_origin, int sensor_axes, float* depth) {
  size_t i = size_t(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= count) return;
  const Vector3f p = points[i];
  if (!isfinite(p.x()) || !isfinite(p.y()) || !isfinite(p.z()) ||
      !lidar.isInValidRange(p)) return;
  // M20 cloud is already in base_link_dog coordinates. Remove returns from
  // the known robot/leg envelope before integrating them as fixed terrain.
  Vector3f body_p=p;
  if(sensor_axes==1)body_p=Vector3f(p.z(),-p.y(),p.x());
  else if(sensor_axes==2)body_p=Vector3f(-p.z(),p.y(),p.x());
  body_p+=body_origin;
  if (self_filter && fabsf(body_p.x())<=.49f && fabsf(body_p.y())<=.333f &&
      body_p.z()>=-.60f && body_p.z()<=.50f) return;
  Index2D uv;
  if (!lidar.project(p, &uv)) return;
  float d = lidar.getDepth(p);
  if (!isfinite(d) || d <= 0.0f) return;
  auto* target = reinterpret_cast<unsigned int*>(depth + uv.y()*lidar.cols()+uv.x());
  unsigned int old = atomicCAS(target, 0u, __float_as_uint(d));
  while (old != 0u && __uint_as_float(old) > d) {
    unsigned int observed = atomicCAS(target, old, __float_as_uint(d));
    if (old == observed) break;
    old = observed;
  }
}
}

void PointcloudConverter::depthImageFromPointcloudGPU(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
    const Lidar& lidar, DepthImage* output) {
  static const bool announced = [] {
    std::fprintf(stderr, "FAST_LIDAR v5: bulk XYZ upload, nearest range, body-coordinate filter, optional optical-plane origin\n");
    return true;
  }();
  (void)announced;
  if (!msg || !output || (output->memory_type() != MemoryType::kDevice &&
                         output->memory_type() != MemoryType::kUnified))
    throw std::invalid_argument("fast_lidar requires a cloud and GPU output");
  const size_t count = size_t(msg->width) * msg->height;
  const size_t row_bytes = size_t(msg->width) * msg->point_step;
  const size_t required = msg->height ? size_t(msg->height-1)*msg->row_step+row_bytes : 0;
  if (row_bytes > msg->row_step || required > msg->data.size() ||
      count > size_t(std::numeric_limits<int>::max()))
    throw std::invalid_argument("fast_lidar malformed PointCloud2 dimensions");
  const int ox=offset(*msg,"x"), oy=offset(*msg,"y"), oz=offset(*msg,"z");
  const uint16_t endian = 1;
  const bool host_big_endian = *reinterpret_cast<const uint8_t*>(&endian) == 0;
  const bool swap = msg->is_bigendian != host_big_endian;
  const char* setting=std::getenv("SE2_LIDAR_SELF_FILTER");
  const bool self_filter=setting && std::strcmp(setting,"1")==0;
  Vector3f body_origin=Vector3f::Zero();
  int sensor_axes=0;
  bool physical_origin=false;
  if(msg->header.frame_id=="nvblox_replay/front_origin"){
    body_origin=Vector3f(.32028f,0.f,-.013f);physical_origin=true;
  }else if(msg->header.frame_id=="nvblox_replay/rear_origin"){
    body_origin=Vector3f(-.32028f,0.f,-.013f);physical_origin=true;
  }else if(msg->header.frame_id=="nvblox_replay/front_sensor"){
    body_origin=Vector3f(.32028f,0.f,-.013f);physical_origin=true;sensor_axes=1;
  }else if(msg->header.frame_id=="nvblox_replay/rear_sensor"){
    body_origin=Vector3f(-.32028f,0.f,-.013f);physical_origin=true;sensor_axes=2;
  }else if(msg->header.frame_id=="nvblox_replay/front_optical_sensor"){
    body_origin=Vector3f(.36560f,0.f,-.013f);physical_origin=true;sensor_axes=1;
  }else if(msg->header.frame_id=="nvblox_replay/rear_optical_sensor"){
    body_origin=Vector3f(-.36560f,0.f,-.013f);physical_origin=true;sensor_axes=2;
  }
  if (self_filter && !physical_origin && msg->header.frame_id!="lidar_link" && msg->header.frame_id!="base_link_dog")
    throw std::invalid_argument("self filter requires the verified body-aligned lidar_link frame");
  if (output->rows()!=lidar.rows() || output->cols()!=lidar.cols())
    *output = DepthImage(lidar.rows(), lidar.cols(), MemoryType::kDevice);
  output->setZeroAsync(*cuda_stream_);
  // push_back(value) constructs a temporary owning CUDA stream per point in
  // this installed nvblox version. Resize once, then fill the pinned buffer.
  lidar_pointcloud_host_.resizeAsync(count, *cuda_stream_);
  Vector3f* packed = lidar_pointcloud_host_.data();
  for (size_t row=0; row<msg->height; ++row) {
    const uint8_t* bytes = msg->data.data()+row*msg->row_step;
    for (size_t col=0; col<msg->width; ++col) {
      const uint8_t* p=bytes+col*msg->point_step;
      packed[row*msg->width+col] = Vector3f(readFloat(p+ox,swap),
          readFloat(p+oy,swap),readFloat(p+oz,swap));
    }
  }
  if (count) {
    lidar_pointcloud_device_.copyFromAsync(lidar_pointcloud_host_, *cuda_stream_);
    projectNearest<<<(count+255)/256,256,0,*cuda_stream_>>>(
        lidar_pointcloud_device_.data(),count,lidar,self_filter,body_origin,sensor_axes,output->dataPtr());
    cudaCheck(cudaGetLastError());
  }
  cudaCheck(cudaStreamSynchronize(*cuda_stream_));
}
}}  // namespace nvblox::conversions
