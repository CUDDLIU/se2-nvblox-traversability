#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <vector>
#include "raster_types.h"

struct V { double x, y, z; };

static std::vector<V> clip(const std::vector<V>& in, int axis, double bound, bool above) {
  std::vector<V> out;
  if (in.empty()) return out;
  out.reserve(8);
  auto coord = [axis](const V& p) { return axis == 0 ? p.x : p.y; };
  auto inside = [&](const V& p) { return above ? coord(p) >= bound : coord(p) <= bound; };
  V prev = in.back();
  bool pin = inside(prev);
  for (const V& cur : in) {
    bool cin = inside(cur);
    if (pin != cin) {
      double t = (bound-coord(prev))/(coord(cur)-coord(prev));
      out.push_back({prev.x+t*(cur.x-prev.x),prev.y+t*(cur.y-prev.y),prev.z+t*(cur.z-prev.z)});
    }
    if (cin) out.push_back(cur);
    prev=cur; pin=cin;
  }
  return out;
}

extern "C" int raster(const double* vertices, size_t nv, const int64_t* indices,
                     size_t nt, double res, double slope_cos, Record** result, size_t* count) {
  *result=nullptr; *count=0;
  try {
    std::vector<Record> records;
    records.reserve(nt*3);
    for (size_t i=0;i<nt;++i) {
      std::vector<V> tri;
      for (int j=0;j<3;++j) {
        int64_t k=indices[3*i+j];
        if (k<0 || size_t(k)>=nv) throw std::runtime_error("bad index");
        V p{vertices[3*k],vertices[3*k+1],vertices[3*k+2]};
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          throw std::runtime_error("nonfinite vertex");
        tri.push_back(p);
      }
      V u{tri[1].x-tri[0].x,tri[1].y-tri[0].y,tri[1].z-tri[0].z};
      V v{tri[2].x-tri[0].x,tri[2].y-tri[0].y,tri[2].z-tri[0].z};
      V n{u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z,u.x*v.y-u.y*v.x};
      double norm=std::sqrt(n.x*n.x+n.y*n.y+n.z*n.z);
      if (norm<1e-12) continue;
      bool support=n.z/norm>=slope_cos;
      double lx=tri[0].x,hx=lx,ly=tri[0].y,hy=ly;
      for (const V& p:tri) { lx=std::min(lx,p.x); hx=std::max(hx,p.x); ly=std::min(ly,p.y); hy=std::max(hy,p.y); }
      double ax=std::floor((lx-1e-9)/res),bx=std::floor((hx+1e-9)/res);
      double ay=std::floor((ly-1e-9)/res),by=std::floor((hy+1e-9)/res);
      if (std::max({std::abs(ax),std::abs(bx),std::abs(ay),std::abs(by)})>1e8 ||
          (bx-ax+1)*(by-ay+1)>100000) throw std::runtime_error("extent too large");
      for (int x=int(ax);x<=int(bx);++x) for (int y=int(ay);y<=int(by);++y) {
        auto poly=clip(tri,0,x*res,true);
        poly=clip(poly,0,(x+1)*res,false);
        poly=clip(poly,1,y*res,true);
        poly=clip(poly,1,(y+1)*res,false);
        if (poly.empty()) continue;
        if (poly.size()>8) throw std::runtime_error("clip overflow");
        Record r{}; r.ix=x; r.iy=y; r.reserved=static_cast<int32_t>(i); r.lo=r.hi=poly[0].z;
        double twice_area=0;
        for (size_t k=0;k<poly.size();++k) {
          const auto& p=poly[k]; const auto& q=poly[(k+1)%poly.size()];
          twice_area+=p.x*q.y-q.x*p.y;
          r.lo=std::min(r.lo,p.z);r.hi=std::max(r.hi,p.z);
        }
        r.support=support && std::abs(twice_area)>.000000000002;
        for (size_t k=0;k<8;++k) { const auto& p=poly[std::min(k,poly.size()-1)];r.xyz[3*k]=p.x;r.xyz[3*k+1]=p.y;r.xyz[3*k+2]=p.z; }
        records.push_back(r);
        if (records.size()>2000000) throw std::runtime_error("record limit exceeded");
      }
    }
    if (!records.empty()) {
      *result=static_cast<Record*>(std::malloc(records.size()*sizeof(Record)));
      if (!*result) throw std::bad_alloc();
      std::memcpy(*result,records.data(),records.size()*sizeof(Record));
    }
    *count=records.size();
    return 0;
  } catch (...) { std::free(*result); *result=nullptr; *count=0; return -1; }
}
extern "C" void free_records(Record* records) { std::free(records); }

// Merge raster records into the integer vertical spans consumed by the paper
// pipeline.  This intentionally mirrors paper_pipeline.voxelize exactly:
// floor(lo/dz+1e-6), ceil(hi/dz-1e-6), touching intervals merge, and an equal
// top height ORs the support flag.  It avoids crossing the Python/native ABI
// for every clipped triangle while retaining all collision intervals.
extern "C" int merge_mixed_records(const Record* records, size_t record_count,
                                      const SpanRecord* cached,size_t cached_count,
                                      double dz, double height,
                                      SpanRecord** result, size_t* count) {
  *result = nullptr;
  *count = 0;
  if (!(std::isfinite(dz) && dz > 0.0 && std::isfinite(height) && height >= 0.0)) {
    return -1;
  }
  try {
    struct Quantized {
      int32_t ix, iy;
      int64_t lo, hi;
      bool support;
    };
    std::vector<Quantized> quantized;
    quantized.reserve(record_count+cached_count);
    for(size_t i=0;i<cached_count;++i) {
      const SpanRecord& s=cached[i];
      quantized.push_back({s.ix,s.iy,s.lo,s.hi,s.slope_ok!=0});
    }
    for (size_t i = 0; i < record_count; ++i) {
      const Record& record = records[i];
      const double lo_value = std::floor(record.lo / dz + 1e-6);
      const double hi_value = std::ceil(record.hi / dz - 1e-6);
      if (!std::isfinite(lo_value) || !std::isfinite(hi_value) ||
          static_cast<long double>(lo_value) < static_cast<long double>(INT64_MIN) ||
          static_cast<long double>(lo_value) >= static_cast<long double>(INT64_MAX) ||
          static_cast<long double>(hi_value) < static_cast<long double>(INT64_MIN) ||
          static_cast<long double>(hi_value) >= static_cast<long double>(INT64_MAX)) {
        return -1;
      }
      const int64_t lo = static_cast<int64_t>(lo_value);
      const int64_t hi = std::max<int64_t>(lo + 1, static_cast<int64_t>(hi_value));
      quantized.push_back({record.ix, record.iy, lo, hi, record.support != 0});
    }
    std::sort(quantized.begin(), quantized.end(), [](const auto& a, const auto& b) {
      if (a.ix != b.ix) return a.ix < b.ix;
      if (a.iy != b.iy) return a.iy < b.iy;
      if (a.lo != b.lo) return a.lo < b.lo;
      return a.hi < b.hi;
    });
    std::vector<SpanRecord> spans;
    spans.reserve(quantized.size());
    size_t pos = 0;
    while (pos < quantized.size()) {
      const int32_t ix = quantized[pos].ix;
      const int32_t iy = quantized[pos].iy;
      std::vector<Quantized> merged;
      while (pos < quantized.size() && quantized[pos].ix == ix &&
             quantized[pos].iy == iy) {
        const Quantized current = quantized[pos++];
        if (merged.empty() || current.lo > merged.back().hi) {
          merged.push_back(current);
        } else {
          Quantized& top = merged.back();
          if (current.hi > top.hi) {
            top.hi = current.hi;
            top.support = current.support;
          } else if (current.hi == top.hi) {
            top.support = top.support || current.support;
          }
          top.hi = std::max(top.hi, current.hi);
        }
      }
      for (size_t i = 0; i < merged.size(); ++i) {
        const Quantized& span = merged[i];
        const int64_t ceiling = i + 1 < merged.size()
                                    ? merged[i + 1].lo
                                    : INT64_MAX;
        // Keep the same order of double operations as Python voxelize: the
        // individually quantized top/ceiling become metres before subtraction.
        const double ceiling_z = static_cast<double>(ceiling) * dz;
        const double top_z = static_cast<double>(span.hi) * dz;
        const bool enough_headroom = ceiling == INT64_MAX ||
                                      ceiling_z - top_z >= height - 1e-9;
        spans.push_back({ix, iy, span.lo, span.hi, ceiling,
                         span.support ? 1 : 0,
                         (span.support && enough_headroom) ? 1 : 0});
      }
    }
    if (!spans.empty()) {
      *result = static_cast<SpanRecord*>(std::malloc(spans.size() * sizeof(SpanRecord)));
      if (!*result) throw std::bad_alloc();
      std::memcpy(*result, spans.data(), spans.size() * sizeof(SpanRecord));
    }
    *count = spans.size();
    return 0;
  } catch (...) {
    std::free(*result);
    *result = nullptr;
    *count = 0;
    return -1;
  }
}

extern "C" int merge_raster_records(const Record* records,size_t record_count,
    double dz,double height,SpanRecord** result,size_t* count) {
  return merge_mixed_records(records,record_count,nullptr,0,dz,height,result,count);
}

extern "C" int raster_spans(const double* vertices, size_t nv,
                              const int64_t* indices, size_t nt,
                              double res, double slope_cos, double dz,
                              double height, SpanRecord** result, size_t* count) {
  *result = nullptr;
  *count = 0;
  if (!(std::isfinite(res) && res > 0.0 && std::isfinite(slope_cos))) return -1;
  Record* records = nullptr;
  size_t n = 0;
  if (raster(vertices, nv, indices, nt, res, slope_cos, &records, &n)) return -1;
  int code = merge_raster_records(records, n, dz, height, result, count);
  std::free(records);
  return code;
}

extern "C" void free_span_records(SpanRecord* records) { std::free(records); }

// Inclusive, exact projected pixel rectangles. No rounded square expansion.
extern "C" void rectangles_clear(const float* depth, int h, int w,
                                  const int64_t* lo, const int64_t* hi,
                                  const double* threshold, size_t count, uint8_t* clear) {
  for (size_t i=0;i<count;++i) {
    int64_t x0=lo[2*i], y0=lo[2*i+1], x1=hi[2*i], y1=hi[2*i+1];
    bool ok=std::isfinite(threshold[i]) && x0>=0 && y0>=0 && x1<w && y1<h && x1>=x0 && y1>=y0;
    for (int64_t y=y0;ok && y<=y1;++y)
      for (int64_t x=x0;x<=x1;++x) {
        float z=depth[y*w+x];
        if (!std::isfinite(z) || z<=0 || z<=threshold[i]) { ok=false; break; }
      }
    clear[i]=ok;
  }
}

extern "C" void evaluate_mask(size_t n, const int64_t* neighbors, const uint8_t* missing,
 const double* z, const double* ceiling, const double* known, const uint8_t* covered,
 const int64_t* parents, const int64_t* directions, size_t length,
 const int64_t* checks, size_t check_count, double height, uint64_t bit,
 uint64_t* geometric, uint64_t* verified, uint64_t* unknown) {
  std::vector<int64_t> mapped(length);
  for (size_t root=0;root<n;++root) {
    mapped[0]=root;
    bool uncertain=!covered[root],ok=covered[root];
    double floor=z[root], roof=ceiling[root], free_top=known[root];
    for (size_t j=1;j<length;++j) {
      int64_t p=mapped[parents[j]], d=directions[j];
      uncertain=uncertain || missing[4*p+d];
      int64_t q=neighbors[4*p+d]; mapped[j]=q;
      ok=ok && covered[q];
      floor=std::max(floor,z[q]); roof=std::min(roof,ceiling[q]);
      free_top=std::min(free_top,known[q]);
    }
    if (ok) for (size_t j=0;j<check_count;++j) {
      int64_t a=checks[3*j], b=checks[3*j+1], d=checks[3*j+2];
      if (neighbors[4*mapped[a]+d]!=mapped[b]) {ok=false;break;}
    }
    // The paper pipeline passes the physical required height. Legacy callers
    // may pass a negative value after checking clearance in their own model.
    ok=ok && (height<0 || roof>=floor+height);
    if (uncertain && !ok) unknown[root]|=bit;
    if (ok) {
      geometric[root]|=bit;
      if (free_top>=floor+height) verified[root]|=bit;
    }
  }
}
