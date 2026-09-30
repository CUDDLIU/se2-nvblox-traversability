// Recover a rasterized riser only between two observed, walkable treads.
// No missing XY column is filled; every intervening height and headroom must
// be observed. Promotion is one pass from original support, never recursive.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "column_index.hpp"

// Estimate the slope of an existing upward surface from a small neighborhood.
// The measured height/ceiling is never modified and missing cells stay missing.
// IRLS reduces individual TSDF facet-normal noise without averaging floors.
extern "C" int support_from_patch(size_t n,const int64_t* xy,const double* z,
    const double* roof,uint8_t* walk,uint8_t* slope,double step,double height,
    double resolution,double max_slope_degrees) {
  try {
    const ColumnIndex columns(n,xy);
    const std::vector<uint8_t> candidates(slope,slope+n);
    const double limit=std::tan(max_slope_degrees*3.141592653589793/180.);
    int accepted=0;
    #pragma omp parallel for schedule(dynamic,64) reduction(+:accepted)
    for(size_t i=0;i<n;++i) {
      slope[i]=walk[i]=0;
      if(!candidates[i])continue;
      struct Sample {double x,y,z;};std::vector<Sample> points;points.reserve(25);
      for(int dx=-2;dx<=2;++dx)for(int dy=-2;dy<=2;++dy) {
        auto column=columns.get(xy[2*i]+dx,xy[2*i+1]+dy);
        if(column.empty())continue;
        size_t chosen=n;double nearest=step+.03;
        for(size_t j:column)if(candidates[j] && std::abs(z[j]-z[i])<nearest) {
          chosen=j;nearest=std::abs(z[j]-z[i]);
        }
        if(chosen!=n)points.push_back({dx*resolution,dy*resolution,z[chosen]-z[i]});
      }
      if(points.size()<8)continue;
      double a=0,b=0,c=0;bool fit=true;
      for(int iteration=0;iteration<3;++iteration) {
        double sw=0,sx=0,sy=0,sz=0,sxx=0,syy=0,sxy=0,sxz=0,syz=0;
        for(const auto& p:points) {
          double error=std::abs(p.z-a*p.x-b*p.y-c);
          double w=iteration?std::min(1.,.04/std::max(1e-8,error)):1.;
          sw+=w;sx+=w*p.x;sy+=w*p.y;sz+=w*p.z;
          sxx+=w*p.x*p.x;syy+=w*p.y*p.y;sxy+=w*p.x*p.y;sxz+=w*p.x*p.z;syz+=w*p.y*p.z;
        }
        double xx=sxx-sx*sx/sw,yy=syy-sy*sy/sw,xy_=sxy-sx*sy/sw;
        double xz=sxz-sx*sz/sw,yz=syz-sy*sz/sw,det=xx*yy-xy_*xy_;
        if(det<1e-8){fit=false;break;}
        a=(xz*yy-yz*xy_)/det;b=(yz*xx-xz*xy_)/det;c=(sz-a*sx-b*sy)/sw;
      }
      double error2=0;for(const auto& p:points) {
        double r=p.z-a*p.x-b*p.y-c;error2+=r*r;
      }
      // Allow the residual of a sampled physical stair tread, but reject an
      // isolated fragment far from its supporting neighborhood.
      if(fit && std::hypot(a,b)<=limit+1e-9 && std::abs(c)<=.07 &&
         std::sqrt(error2/points.size())<=.08) {
        slope[i]=1;walk[i]=roof[i]-z[i]>=height-1e-8;++accepted;
      }
    }
    return accepted;
  }catch(...){return -1;}
}

extern "C" int promote_stair_risers(size_t n,const int64_t* xy,const double* z,
    const double* roof,uint8_t* walk,uint8_t* slope,double step,double height,
    double resolution,double max_slope_degrees) {
  try {
    const ColumnIndex columns(n,xy);
    const std::vector<uint8_t> original(walk,walk+n);
    const double grade=std::tan(max_slope_degrees*3.141592653589793/180.);
    int promoted=0;
    #pragma omp parallel for schedule(dynamic,64) reduction(+:promoted)
    for(size_t i=0;i<n;++i) {
      if(original[i] || roof[i]-z[i]<height-1e-8)continue;
      bool allow=false;
      for(int axis=0;axis<2 && !allow;++axis) {
        int dx=axis==0,dy=axis==1;
        for(int left=1;left<=2 && !allow;++left) for(int right=1;right<=2 && !allow;++right) {
          auto a=columns.get(xy[2*i]-left*dx,xy[2*i+1]-left*dy);
          auto b=columns.get(xy[2*i]+right*dx,xy[2*i+1]+right*dy);
          if(a.empty() || b.empty())continue;
          for(size_t j:a)for(size_t k:b) {
            if(!original[j] || !original[k])continue;
            double low=std::min(z[j],z[k]),high=std::max(z[j],z[k]);
            if(high-low<std::min(.05,step*.5) || high-low>step+1e-8 ||
               high-low>grade*(left+right)*resolution+1e-8 ||
               z[i]<low-.015 || z[i]>high+.025)continue;
            bool observed=true;
            for(int d=-left+1;d<right && observed;++d) {
              auto middle=columns.get(xy[2*i]+d*dx,xy[2*i+1]+d*dy);
              bool found=false;
              for(size_t m:middle)
                if(z[m]>=low-.015 && z[m]<=high+.025 && roof[m]-high>=height-1e-8)found=true;
              observed=found;
            }
            if(observed)allow=true;
          }
        }
      }
      if(allow){walk[i]=1;slope[i]=1;++promoted;}
    }
    return promoted;
  }catch(...){return -1;}
}
