// Geometry used by the existing SE(2) classifier. There is no portal detector,
// historical permission, post-classification fill or visualization-only path.
namespace pose_clearance {
using P=std::array<double,3>;
constexpr double eps=1e-7;
struct Triangle {
  P p[3]; double lo[3],hi[3],nz,norm,angle,area;
  bool support;
  Triangle(const double* vertices,const int64_t* ids) {
    for(int k=0;k<3;++k) for(int d=0;d<3;++d)p[k][d]=vertices[3*ids[k]+d];
    double a[3],b[3],normal[3];
    for(int d=0;d<3;++d){a[d]=p[1][d]-p[0][d];b[d]=p[2][d]-p[0][d];
      lo[d]=std::min({p[0][d],p[1][d],p[2][d]});hi[d]=std::max({p[0][d],p[1][d],p[2][d]});}
    for(int d=0;d<3;++d)normal[d]=a[(d+1)%3]*b[(d+2)%3]-a[(d+2)%3]*b[(d+1)%3];
    nz=normal[2];norm=std::hypot(std::hypot(normal[0],normal[1]),nz);area=norm/2;
    support=nz>norm*std::cos(80.*M_PI/180.);
    angle=std::atan2(normal[0],-normal[1]);
    while(angle<0)angle+=M_PI;
    while(angle>=M_PI)angle-=M_PI;
  }
  bool at(double x,double y,double& z) const {
    if(!support || x<lo[0]-eps || x>hi[0]+eps || y<lo[1]-eps || y>hi[1]+eps)return false;
    double u=((x-p[0][0])*(p[2][1]-p[0][1])-(y-p[0][1])*(p[2][0]-p[0][0]))/nz;
    double v=((p[1][0]-p[0][0])*(y-p[0][1])-(p[1][1]-p[0][1])*(x-p[0][0]))/nz;
    if(u<-eps || v<-eps || u+v>1+eps)return false;
    z=p[0][2]+u*(p[1][2]-p[0][2])+v*(p[2][2]-p[0][2]);return true;
  }
};
struct Polygon {
  std::array<P,8> values;size_t count=0;
  Polygon()=default;
  Polygon(std::initializer_list<P> p){for(const auto& v:p)push_back(v);}
  void push_back(const P& p){values[count++]=p;}
  bool empty()const{return count==0;}
  size_t size()const{return count;}
  const P& back()const{return values[count-1];}
  P& operator[](size_t i){return values[i];}
  const P& operator[](size_t i)const{return values[i];}
  P* begin(){return values.data();} P* end(){return values.data()+count;}
  const P* begin()const{return values.data();} const P* end()const{return values.data()+count;}
};
Polygon clip(const Polygon& vertices,double z,bool above) {
  Polygon out;if(vertices.empty())return out;
  P previous=vertices.back();bool before=above?previous[2]>=z:previous[2]<=z;
  for(const auto& point:vertices) {
    bool inside=above?point[2]>=z:point[2]<=z;
    if(inside!=before) {
      double t=(z-previous[2])/(point[2]-previous[2]);P cut;
      for(int d=0;d<3;++d)cut[d]=previous[d]+t*(point[d]-previous[d]);
      out.push_back(cut);
    }
    if(inside)out.push_back(point);
    previous=point;before=inside;
  }
  return out;
}
struct Scene {
  std::vector<Triangle> triangles;
  std::unordered_map<uint64_t,std::vector<size_t>> ground;
  double cell=.1,quantum=.01,slope_limit=INFINITY;
  static uint64_t key(int64_t x,int64_t y) {return (uint64_t(uint32_t(x))<<32)|uint32_t(y);}
  Scene(MeshStore& mesh,double x,double y,double z,double radius,double rise,double height,double resolution,double slope=89.,double dz=.01):cell(resolution),quantum(dz),slope_limit(std::tan(slope*M_PI/180.)) {
    double low[]={x-radius,y-radius,z-rise},high[]={x+radius,y+radius,z+rise+height};
    std::vector<int64_t> blocks;size_t visits=0;mesh.top->query(low,high,blocks,visits,false);
    for(auto block:blocks) {
      const auto& b=*mesh.ordered[block];std::vector<int64_t> ids;
      b.tree->query(low,high,ids,visits,false);
      for(auto id:ids)triangles.emplace_back(b.vertices.data(),b.triangles.data()+3*id);
    }
    for(size_t i=0;i<triangles.size();++i) {
      const auto& t=triangles[i];if(!t.support)continue;
      for(int64_t ix=std::floor(std::max(low[0],t.lo[0])/cell);ix<=std::floor(std::min(high[0],t.hi[0])/cell);++ix)
        for(int64_t iy=std::floor(std::max(low[1],t.lo[1])/cell);iy<=std::floor(std::min(high[1],t.hi[1])/cell);++iy)
          ground[key(ix,iy)].push_back(i);
    }
  }
  bool floor_at(double x,double y,double expected,double rise,double& z) const {
    auto found=ground.find(key(std::floor(x/cell),std::floor(y/cell)));
    if(found==ground.end())return false;
    double best=std::numeric_limits<double>::infinity();
    for(auto id:found->second) {double candidate;
      if(triangles[id].at(x,y,candidate) && std::abs(candidate-expected)<=rise && std::abs(candidate-expected)<best) {
        best=std::abs(candidate-expected);z=candidate;
      }
    }
    return std::isfinite(best);
  }
  bool support(double x,double y,double z,double angle,double a,double b,double step,double rise,double& maximum) const {
    const int nx=std::max(1,int(std::ceil(2*a/cell))),ny=std::max(1,int(std::ceil(2*b/cell)));
    std::vector<double> previous(ny+1),current(ny+1);
    double c=std::cos(angle),s=std::sin(angle),center=0.;
    if(!floor_at(x,y,z,rise,center) || std::abs(center-z)>step+.02)return false;
    maximum=center;double xx=0.,yy=0.,xz=0.,yz=0.;
    for(int i=0;i<=nx;++i) {
      double dx=-a+2*a*i/nx;
      for(int j=0;j<=ny;++j) {
        double dy=-b+2*b*j/ny,h=0.;
        if(!floor_at(x+c*dx-s*dy,y+s*dx+c*dy,z,rise,h))return false;
        if((i && std::abs(h-previous[j])>step+eps) || (j && std::abs(h-current[j-1])>step+eps))return false;
        current[j]=h;maximum=std::max(maximum,h);
        xx+=dx*dx;yy+=dy*dy;xz+=dx*h;yz+=dy*h;
      }
      previous.swap(current);
    }
    // Match the original span's upper voxel face. Sub-voxel Mesh ripples
    // must not become body obstacles below the quantized support surface.
    maximum=std::ceil(maximum/quantum-1e-9)*quantum;
    return std::hypot(xz/xx,yz/yy)<=slope_limit+eps;
  }
  // Exact SAT against the height-clipped Mesh face. The candidate rectangle
  // is the configured physical body, not an inflated union of grid cells.
  bool collision(const Triangle& t,double x,double y,double floor,double height,double c,double s,double a,double b,
                 double low_x,double high_x,double low_y,double high_y,double& shift_x,double& shift_y) const {
    if(t.hi[2]<=floor+eps || t.lo[2]>=floor+height-eps)return false;
    double rx=a*std::abs(c)+b*std::abs(s),ry=a*std::abs(s)+b*std::abs(c);
    if(t.lo[0]>=x+rx-eps || t.hi[0]<=x-rx+eps || t.lo[1]>=y+ry-eps || t.hi[1]<=y-ry+eps)return false;
    auto points=clip(clip({t.p[0],t.p[1],t.p[2]},floor+eps,true),floor+height-eps,false);
    if(points.size()<2)return false;
    for(auto& p:points){double dx=p[0]-x,dy=p[1]-y;p[0]=c*dx+s*dy;p[1]=-s*dx+c*dy;}
    double best=std::numeric_limits<double>::infinity();shift_x=shift_y=0.;
    for(size_t axis=0;axis<points.size()+2;++axis) {
      double u=axis==0?1.:0.,v=axis==1?1.:0.;
      if(axis>=2) {auto& p=points[axis-2];auto& q=points[(axis-1)%points.size()];u=p[1]-q[1];v=q[0]-p[0];
        double length=std::hypot(u,v);if(length<eps)continue;u/=length;v/=length;}
      double lo=INFINITY,hi=-INFINITY;
      for(auto& p:points){double projection=u*p[0]+v*p[1];lo=std::min(lo,projection);hi=std::max(hi,projection);}
      double radius=a*std::abs(u)+b*std::abs(v);
      if(lo>=radius-eps || hi<=-radius+eps)return false;
      for(double offset:{lo-radius,hi+radius}) {
        double dx=offset*(c*u-s*v),dy=offset*(s*u+c*v);
        if(x+dx<low_x-eps || x+dx>high_x+eps || y+dy<low_y-eps || y+dy>high_y+eps)continue;
        if(std::abs(offset)<best){best=std::abs(offset);shift_x=dx;shift_y=dy;}
      }
    }
    return true;
  }
  bool pose(double root_x,double root_y,double z,double angle,double length,double width,double height,double step,double rise,
            double& x,double& y,double& floor,bool move=true) const {
    double half=move?cell/2-1e-9:0.,lx=root_x-half,hx=root_x+half,ly=root_y-half,hy=root_y+half;
    x=root_x;y=root_y;double c=std::cos(angle),s=std::sin(angle);
    for(int iteration=0;iteration<16;++iteration) {
      bool supported=support(x,y,z,angle,length/2,width/2,step,rise,floor);
      if(!supported && !floor_at(x,y,z,step+.02,floor))return false;
      bool changed=false;
      for(const auto& t:triangles) {
        double dx,dy;
        if(collision(t,x,y,floor,height,c,s,length/2,width/2,lx,hx,ly,hy,dx,dy)) {
          if(std::hypot(dx,dy)<eps)return false;
          x+=dx;y+=dy;changed=true;break;
        }
      }
      if(!changed)return supported;
    }
    return false;
  }
  double angular_radius(double x,double y,double z,double floor,double angle,double length,double width,
                        double height,double step,double rise,double bound) const {
    // Separating-axis gaps bound corner displacement for every angle in the
    // returned interval. Contact has zero angular freedom, but remains valid.
    double clearance=INFINITY,c=std::cos(angle),s=std::sin(angle),a=length/2,b=width/2;
    for(const auto& t:triangles) {
      if(t.hi[2]<=floor+eps || t.lo[2]>=floor+height-eps)continue;
      auto points=clip(clip({t.p[0],t.p[1],t.p[2]},floor+eps,true),floor+height-eps,false);
      if(points.size()<2)continue;
      for(auto& p:points){double dx=p[0]-x,dy=p[1]-y;p[0]=c*dx+s*dy;p[1]=-s*dx+c*dy;}
      double separation=0.;
      for(size_t axis=0;axis<points.size()+2;++axis) {
        double u=axis==0?1.:0.,v=axis==1?1.:0.;
        if(axis>=2){auto& p=points[axis-2];auto& q=points[(axis-1)%points.size()];u=p[1]-q[1];v=q[0]-p[0];
          double d=std::hypot(u,v);if(d<eps)continue;u/=d;v/=d;}
        double lo=INFINITY,hi=-INFINITY;
        for(auto& p:points){double d=u*p[0]+v*p[1];lo=std::min(lo,d);hi=std::max(hi,d);}
        double r=a*std::abs(u)+b*std::abs(v);
        separation=std::max(separation,std::max(lo-r,-r-hi));
      }
      clearance=std::min(clearance,separation);
    }
    double radius=std::hypot(a,b);
    double delta=std::min(bound,2*std::asin(std::min(1.,clearance/(2*radius))));
    for(int iteration=0;iteration<6 && delta>1e-6;++iteration) {
      double pad=2*radius*std::sin(delta/2),h=0.;
      if(support(x,y,z,angle,a+pad,b+pad,step,rise,h) && h<=floor+eps)return delta;
      delta*=.5;
    }
    return 0.;
  }

};
}

extern "C" int mesh_store_body_states(void* ptr,size_t n,const int64_t* xy,const double* z,const uint8_t* walk,
    const int64_t* roots,size_t count,double dz,double resolution,double length,double width,double height,double step,double slope,
    int bins,const uint64_t* comfortable,const double* distances,uint64_t* masks,double* states) {
  try {
    auto& mesh=*static_cast<MeshStore*>(ptr);mesh.rebuild();
    const uint64_t full=bins==64?~uint64_t(0):(uint64_t(1)<<bins)-1;
    const double radius=std::hypot(length,width)/2+resolution;
    const double rise=std::hypot(length,width)/2*std::tan(slope*M_PI/180.)+step+.03;
    std::fill(states,states+n*bins*6,std::numeric_limits<double>::quiet_NaN());
    std::copy(comfortable,comfortable+n,masks);
    #pragma omp parallel for schedule(dynamic,8)
    for(size_t ri=0;ri<count;++ri) {
      size_t root=roots[ri];if(!walk[root])continue;
      double x=(xy[2*root]+.5)*resolution,y=(xy[2*root+1]+.5)*resolution;
      for(int bin=0;bin<bins;++bin)if(comfortable[root]&(uint64_t(1)<<bin)) {
        double* state=states+(root*bins+bin)*6;state[0]=x;state[1]=y;state[2]=z[root];state[3]=2*M_PI*bin/bins;
        state[4]=state[3]-M_PI/bins;state[5]=state[3]+M_PI/bins;
      }
      if(comfortable[root]==full || distances[root]+2*resolution<std::min(length,width)/2)continue;
      pose_clearance::Scene scene(mesh,x,y,z[root],radius,rise,height,resolution,slope,dz);
      // Ordinary orientation candidates plus measured face directions. These
      // are orientation coordinates in the same state search, not doors or
      // corridors detected by a separate model.
      std::vector<double> angles(bins),weight(bins,0.);
      for(int bin=0;bin<bins;++bin)angles[bin]=2*M_PI*bin/bins;
      for(const auto& t:scene.triangles)if(std::abs(t.nz)<t.norm*.15 && t.hi[2]>z[root]+.1) {
        for(double angle:{t.angle,t.angle+M_PI}) {
          int bin=int(std::llround(angle*bins/(2*M_PI)))%bins;
          if(t.area>weight[bin]){weight[bin]=t.area;angles[bin]=angle;}
        }
      }
      for(int bin=0;bin<(bins%2?bins:bins/2);++bin) {
        if(masks[root]&(uint64_t(1)<<bin))continue;
        double choices[]={2*M_PI*bin/bins,angles[bin]};
        for(double angle:choices) {
          double px,py,floor;
          if(scene.pose(x,y,z[root],angle,length,width,height,step,rise,px,py,floor)) {
            masks[root]|=uint64_t(1)<<bin;
            double* state=states+(root*bins+bin)*6;state[0]=px;state[1]=py;state[2]=floor;state[3]=angle;
            double center=2*M_PI*bin/bins;
            while(angle-center>M_PI)angle-=2*M_PI;
            while(angle-center<-M_PI)angle+=2*M_PI;
            state[3]=angle;
            double delta=scene.angular_radius(px,py,z[root],floor,angle,length,width,height,step,rise,
                M_PI/bins+std::abs(angle-center));
            state[4]=std::max(angle-delta,center-M_PI/bins);
            state[5]=std::min(angle+delta,center+M_PI/bins);
            break;
          }
        }
      }
      // A rectangular footprint is invariant under a 180-degree rotation.
      if(bins%2==0)for(int bin=0;bin<bins/2;++bin)if(masks[root]&(uint64_t(1)<<bin)) {
        int other=bin+bins/2;
        if(comfortable[root]&(uint64_t(1)<<other))continue;
        masks[root]|=uint64_t(1)<<other;
        double* src=states+(root*bins+bin)*6;double* dst=states+(root*bins+other)*6;
        std::copy(src,src+6,dst);for(int k=3;k<6;++k)dst[k]+=M_PI;
      }
    }
    return 0;
  }catch(...){return -1;}
}

extern "C" int mesh_store_geometry(void* ptr,const double* low,const double* high,double** output,size_t* count) {
  try {
    auto& mesh=*static_cast<MeshStore*>(ptr);mesh.rebuild();
    std::vector<double> data;std::vector<int64_t> blocks;size_t visits=0;
    mesh.top->query(low,high,blocks,visits,false);
    for(auto id:blocks) {
      auto& b=*mesh.ordered[id];std::vector<int64_t> faces;
      b.tree->query(low,high,faces,visits,false);
      for(auto face:faces)for(int k=0;k<3;++k) {
        auto v=b.triangles[3*face+k];
        data.insert(data.end(),b.vertices.begin()+3*v,b.vertices.begin()+3*v+3);
      }
    }
    *count=data.size()/9;*output=static_cast<double*>(std::malloc(data.size()*sizeof(double)));
    if(!data.empty() && !*output)return -1;
    std::copy(data.begin(),data.end(),*output);return 0;
  }catch(...){return -1;}
}

extern "C" int mesh_store_motions(void* ptr,size_t count,const double* motions,const int64_t* groups,
    double dz,double resolution,double length,double width,double height,double step,double slope,uint8_t* accepted) {
  try {
    auto& mesh=*static_cast<MeshStore*>(ptr);mesh.rebuild();
    const double radius=std::hypot(length,width)/2+3*resolution;
    const double rise=std::hypot(length,width)/2*std::tan(slope*M_PI/180.)+step+.03;
    std::vector<size_t> starts={0};
    for(size_t i=1;i<count;++i)if(groups[i]!=groups[i-1])starts.push_back(i);
    starts.push_back(count);
    #pragma omp parallel for schedule(dynamic,4)
    for(size_t group=0;group<starts.size()-1;++group) {
      size_t begin=starts[group],end=starts[group+1];if(begin==end)continue;
      const double* first=motions+12*begin;
      pose_clearance::Scene scene(mesh,first[0],first[1],first[2],radius,rise,height,resolution,slope,dz);
      for(size_t i=begin;i<end;++i) {
        const double* a=motions+12*i;const double* b=a+6;
        double delta=std::remainder(b[3]-a[3],2*M_PI),angle=a[3]+delta/2;
        double dx=b[0]-a[0],dy=b[1]-a[1],c=std::cos(angle),s=std::sin(angle);
        double pad=std::hypot(length,width)*std::sin(std::abs(delta)/4);
        double swept_length=length+std::abs(c*dx+s*dy)+2*pad;
        double swept_width=width+std::abs(-s*dx+c*dy)+2*pad;
        double x,y,z;
        accepted[i]=std::abs(a[2]-b[2])<=step+1e-7 &&
          scene.pose((a[0]+b[0])/2,(a[1]+b[1])/2,(a[2]+b[2])/2,angle,
                     swept_length,swept_width,height,step,rise,x,y,z,false);
      }
    }
    return 0;
  }catch(...){return -1;}
}
