#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include <vector>
#include <map>
#include <deque>
#include <limits>
#include <fstream>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <sys/wait.h>
using Cloud=sensor_msgs::msg::PointCloud2;
struct V {float x,y,z,w;};
struct Link {std::string name;std::vector<std::array<float,3>> pts;};
struct Capsule {std::string a,b;float radius;};
bool sendall(int fd,const void *data,size_t n){auto *p=(const char*)data;while(n){auto k=write(fd,p,n);if(k<=0)return false;p+=k;n-=k;}return true;}
bool readall(int fd,void *data,size_t n){auto*p=(char*)data;while(n){auto k=read(fd,p,n);if(k<=0)return false;p+=k;n-=k;}return true;}
std::array<double,12> mat(const geometry_msgs::msg::Transform &t){auto q=t.rotation;double x=q.x,y=q.y,z=q.z,w=q.w;return {1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w,t.translation.x,2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w,t.translation.y,2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y,t.translation.z};}
const char * required_env(const char * name) {
 const char * value = std::getenv(name);
 if (!value || !*value) throw std::runtime_error(std::string("Missing ") + name);
 return value;
}
class N:public rclcpp::Node{
public:
 N():Node("codex_segmentacion_cpp"),buffer_(get_clock()),listener_(buffer_){
  std::ifstream f(required_env("THESIS_SEGMENT_MESH"),std::ios::binary);uint32_t count=0;f.read((char*)&count,4);if(!f||count!=13)throw std::runtime_error("meshes unavailable");
  for(uint32_t i=0;i<count;i++){uint32_t len,num;f.read((char*)&len,4);f.read((char*)&num,4);if(len>200||num>100000)throw std::runtime_error("invalid mesh");Link link;link.name.resize(len);f.read(link.name.data(),len);link.pts.resize(num);f.read((char*)link.pts.data(),num*12);links_.push_back(std::move(link));}if(!f)throw std::runtime_error("invalid mesh data");
  auto y=YAML::LoadFile(required_env("THESIS_SEGMENT_CAPSULES"))["capsule_visualizer"]["ros__parameters"];auto a=y["segment_start_frames"].as<std::vector<std::string>>();auto b=y["segment_end_frames"].as<std::vector<std::string>>();auto rr=y["segment_radii"].as<std::vector<float>>();if(a.size()!=6||b.size()!=6||rr.size()!=6)throw std::runtime_error("invalid capsules");for(size_t i=0;i<6;i++)caps_.push_back({a[i],b[i],rr[i]});
  int to[2],from[2];if(pipe(to)||pipe(from))throw std::runtime_error("pipe failure");child_=fork();if(child_==0){dup2(to[0],0);dup2(from[1],1);close(to[0]);close(to[1]);close(from[0]);close(from[1]);setenv("__NV_PRIME_RENDER_OFFLOAD","1",1);setenv("__GLX_VENDOR_LIBRARY_NAME","nvidia",1);execl(required_env("THESIS_SEGMENT_GPU"),"codex_gpu_mask",nullptr);_exit(127);}close(to[0]);close(from[1]);input_=to[1];output_=from[0];
  auto q=rclcpp::SensorDataQoS().keep_last(1);sub_=create_subscription<Cloud>("/camera/d435i/depth/color/points",q,[this](Cloud::ConstSharedPtr m){accept_raw(m);});
  arm_=create_publisher<Cloud>("/codex/segmentacion_fused/brazo",q);unc_=create_publisher<Cloud>("/codex/segmentacion_fused/incierto",q);rest_=create_publisher<Cloud>("/codex/segmentacion_fused/resto",q);
  timer_=create_wall_timer(std::chrono::milliseconds(10),[this](){process();});RCLCPP_WARN(get_logger(),"Experimental, no calibration or safety use");
 }
 ~N(){close(input_);close(output_);int s;waitpid(child_,&s,0);}
private:
 void accept_raw(Cloud::ConstSharedPtr src){
  if(src->header.frame_id!="d435i_depth_optical_frame"||src->is_bigendian||!src->point_step)return;
  int off[3]={-1,-1,-1};for(const auto &f:src->fields)for(int j=0;j<3;j++)if(f.name==std::string(1,"xyz"[j])&&f.datatype==7)off[j]=f.offset;
  for(int i:off)if(i<0||size_t(i+4)>src->point_step)return;
  if(src->data.size()<size_t(src->row_step)*src->height||src->row_step<size_t(src->width)*src->point_step)return;
  auto dst=std::make_shared<Cloud>();dst->header=src->header;dst->header.frame_id="j2n6s300_link_base";dst->height=1;dst->fields=src->fields;dst->is_bigendian=false;dst->point_step=src->point_step;dst->is_dense=true;dst->data.reserve(src->data.size());
  static const double T[12]={0.99678283379185739,-0.030613137601291703,0.074073058962612362,-0.049989844477949082,-0.063846257727633193,0.25541372412541591,0.9647214545673598,-0.65888731019659497,-0.04845242648259121,-0.96634707291665667,0.25263748144992021,0.24566380982590308};
  for(size_t row=0;row<src->height;row++)for(size_t col=0;col<src->width;col++){
   const auto *p=src->data.data()+row*src->row_step+col*src->point_step;float xyz[3];for(int i=0;i<3;i++)memcpy(&xyz[i],p+off[i],4);
   double d=double(xyz[0])*xyz[0]+double(xyz[1])*xyz[1]+double(xyz[2])*xyz[2];if(!std::isfinite(d)||d<.01||d>2.25)continue;
   auto start=dst->data.size();dst->data.insert(dst->data.end(),p,p+src->point_step);
   for(int i=0;i<3;i++){float val=float(T[4*i]*xyz[0]+T[4*i+1]*xyz[1]+T[4*i+2]*xyz[2]+T[4*i+3]);memcpy(dst->data.data()+start+off[i],&val,4);}
  }
  dst->width=dst->data.size()/dst->point_step;dst->row_step=dst->data.size();queue_.push_back(dst);if(queue_.size()>45)queue_.pop_front();
 }
 void process(){
  if(queue_.empty())return;Cloud::ConstSharedPtr cloud;try{auto latest=buffer_.lookupTransform("j2n6s300_link_base","j2n6s300_link_1",tf2::TimePointZero);auto t=rclcpp::Time(latest.header.stamp);for(auto it=queue_.rbegin();it!=queue_.rend();++it)if(rclcpp::Time((*it)->header.stamp)<=t && (now()-rclcpp::Time((*it)->header.stamp)).seconds()<1.5){cloud=*it;break;}}catch(...){return;}
  if(!cloud||rclcpp::Time(cloud->header.stamp).nanoseconds()==last_)return;
  try{
   if(cloud->header.frame_id!="j2n6s300_link_base"||cloud->is_bigendian||cloud->height!=1)return;
   int off[3]={-1,-1,-1};for(auto &f:cloud->fields)for(int j=0;j<3;j++)if(f.name==std::string(1,"xyz"[j])&&f.datatype==7)off[j]=f.offset;for(int v:off)if(v<0||size_t(v+4)>cloud->point_step)return;
   auto stamp=rclcpp::Time(cloud->header.stamp);std::vector<V> mesh,bounds;std::map<std::string,V> centers;centers["j2n6s300_link_base"]={0,0,0,0};
   for(auto &link:links_){std::array<double,12> R={1,0,0,0,0,1,0,0,0,0,1,0};if(link.name!="j2n6s300_link_base"){auto t=buffer_.lookupTransform("j2n6s300_link_base",link.name,stamp).transform;R=mat(t);centers[link.name]={float(t.translation.x),float(t.translation.y),float(t.translation.z),0};}V lo={INFINITY,INFINITY,INFINITY,float(mesh.size())},hi={-INFINITY,-INFINITY,-INFINITY,float(mesh.size()+link.pts.size())};for(auto &p:link.pts){V x={float(R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+R[3]),float(R[4]*p[0]+R[5]*p[1]+R[6]*p[2]+R[7]),float(R[8]*p[0]+R[9]*p[1]+R[10]*p[2]+R[11]),0};lo.x=std::min(lo.x,x.x);lo.y=std::min(lo.y,x.y);lo.z=std::min(lo.z,x.z);hi.x=std::max(hi.x,x.x);hi.y=std::max(hi.y,x.y);hi.z=std::max(hi.z,x.z);mesh.push_back(x);}bounds.push_back(lo);bounds.push_back(hi);}
   auto ef=buffer_.lookupTransform("j2n6s300_link_base","j2n6s300_end_effector",stamp).transform.translation;centers["j2n6s300_end_effector"]={float(ef.x),float(ef.y),float(ef.z),0};std::vector<V> caps;for(auto &c:caps_){auto a=centers.at(c.a),b=centers.at(c.b);a.w=c.radius;caps.push_back(a);caps.push_back(b);}for(int i=1;i<=6;i++)caps.push_back(centers.at("j2n6s300_link_"+std::to_string(i)));
   std::vector<V> pts;pts.reserve(cloud->width);for(size_t i=0;i<cloud->width;i++){auto p=cloud->data.data()+i*cloud->point_step;V v{};std::memcpy(&v.x,p+off[0],4);std::memcpy(&v.y,p+off[1],4);std::memcpy(&v.z,p+off[2],4);pts.push_back(v);}
   uint32_t header[4]={uint32_t(pts.size()),uint32_t(mesh.size()),uint32_t(links_.size()),uint32_t(caps_.size())};if(!sendall(input_,header,sizeof(header))||!sendall(input_,pts.data(),pts.size()*16)||!sendall(input_,mesh.data(),mesh.size()*16)||!sendall(input_,bounds.data(),bounds.size()*16)||!sendall(input_,caps.data(),caps.size()*16))throw std::runtime_error("GPU pipe failed");std::vector<uint32_t> labels(pts.size());if(!readall(output_,labels.data(),labels.size()*4))throw std::runtime_error("GPU read failed");
   std::array<std::unique_ptr<Cloud>,3> out;for(auto &o:out){o=std::make_unique<Cloud>();o->header=cloud->header;o->height=1;o->fields=cloud->fields;o->is_bigendian=cloud->is_bigendian;o->point_step=cloud->point_step;o->is_dense=true;o->data.reserve(cloud->data.size());}
   for(size_t i=0;i<pts.size();i++){if(!std::isfinite(pts[i].x)||!std::isfinite(pts[i].y)||!std::isfinite(pts[i].z)||labels[i]>2)continue;auto b=cloud->data.begin()+i*cloud->point_step;out[labels[i]]->data.insert(out[labels[i]]->data.end(),b,b+cloud->point_step);}
   for(auto &o:out){o->width=o->data.size()/o->point_step;o->row_step=o->data.size();}arm_->publish(std::move(out[0]));unc_->publish(std::move(out[1]));rest_->publish(std::move(out[2]));last_=stamp.nanoseconds();
  }catch(const std::exception&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Unavailable: %s",e.what());}
 }
 tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;std::vector<Link> links_;std::vector<Capsule> caps_;std::deque<Cloud::ConstSharedPtr> queue_;int input_,output_;pid_t child_;int64_t last_=0;rclcpp::TimerBase::SharedPtr timer_;rclcpp::Subscription<Cloud>::SharedPtr sub_;rclcpp::Publisher<Cloud>::SharedPtr arm_,unc_,rest_;
};
int main(int argc,char**argv){std::signal(SIGPIPE, SIG_IGN);rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<N>());rclcpp::shutdown();}
