// Exact layered connectivity, boundary distance and swept yaw masks in one call.
// Query halo participates in connectivity; only requested core roots are tested.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

extern "C" int span_neighbors(size_t, const int64_t*, const double*, const double*,
    const uint8_t*, const uint8_t*, double, double, int64_t*, uint8_t*, int32_t*, int64_t*);

extern "C" int classify_selected(size_t n, const int64_t* xy, const double* z,
    const double* roof, const uint8_t* walk, const uint8_t* slope,
    double step, double height, double resolution, double radius, uint64_t full,
    const int64_t* roots, size_t root_count,
    const int64_t* parents, const int64_t* directions, const int64_t* checks,
    const int64_t* mask_offsets, const int64_t* check_offsets,
    const uint64_t* bits, size_t mask_count,
    uint64_t* geometry, double* distances, int32_t* reasons) {
  try {
    std::vector<int64_t> neighbors((n+1)*4), components(n);
    std::vector<uint8_t> missing((n+1)*4), invalid(n);
    if (span_neighbors(n, xy, z, roof, walk, slope, step, height,
        neighbors.data(), missing.data(), reasons, components.data())) return -1;
    int64_t component_count=0;
    for (size_t i=0;i<n;++i) {
      geometry[i]=0;distances[i]=0.;
      invalid[i]=!walk[i];
      for (int d=0;d<4;++d) invalid[i]|=neighbors[4*i+d]==static_cast<int64_t>(n);
      component_count=std::max(component_count,components[i]+1);
    }
    std::vector<std::vector<size_t>> sources(component_count);
    for (size_t i=0;i<n;++i)
      if (invalid[i] && components[i]>=0) sources[components[i]].push_back(i);
    size_t max_length=0;
    for (size_t m=0;m<mask_count;++m)
      max_length=std::max(max_length,static_cast<size_t>(mask_offsets[m+1]-mask_offsets[m]));
    std::vector<int64_t> mapped(max_length);
    for (size_t ri=0;ri<root_count;++ri) {
      const int64_t root=roots[ri];
      if (root<0 || root>=static_cast<int64_t>(n)) return -2;
      if (!walk[root]) continue;
      double squared=std::numeric_limits<double>::infinity();
      // The same Euclidean distance, on the same connected layer, as cKDTree.
      // Small local tiles make this contiguous loop cheaper than Python/trees.
      for (size_t seed:sources[components[root]]) {
        const double dx=xy[2*root]*resolution-xy[2*seed]*resolution;
        const double dy=xy[2*root+1]*resolution-xy[2*seed+1]*resolution;
        squared=std::min(squared,dx*dx+dy*dy);
      }
      distances[root]=std::sqrt(squared);
      if (!invalid[root]) reasons[root]=6; // footprint_or_boundary_distance
      if (distances[root]<radius-1e-9) continue;
      for (size_t m=0;m<mask_count;++m) {
        const int64_t offset=mask_offsets[m],length=mask_offsets[m+1]-offset;
        if (length<=0) return -3;
        mapped[0]=root;
        double floor=z[root],ceiling=roof[root];
        bool ok=true;
        for (int64_t j=1;j<length;++j) {
          const int64_t p=mapped[parents[offset+j]];
          const int64_t q=neighbors[4*p+directions[offset+j]];
          // A failed path can never be repaired by later footprint cells.
          if (q==static_cast<int64_t>(n) || !walk[q]) {ok=false;break;}
          mapped[j]=q;
          floor=std::max(floor,z[q]);ceiling=std::min(ceiling,roof[q]);
        }
        if (!ok || ceiling<floor+height) continue;
        for (int64_t j=check_offsets[m];j<check_offsets[m+1];++j) {
          const int64_t a=checks[3*j],b=checks[3*j+1],d=checks[3*j+2];
          if (neighbors[4*mapped[a]+d]!=mapped[b]) {ok=false;break;}
        }
        if (ok) geometry[root]|=bits[m];
      }
      if (geometry[root]) reasons[root]=geometry[root]==full?8:7;
    }
    return 0;
  } catch (...) {return -1;}
}
