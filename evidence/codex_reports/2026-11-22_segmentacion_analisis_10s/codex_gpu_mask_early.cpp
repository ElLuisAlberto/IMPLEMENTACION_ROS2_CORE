#include <GL/glew.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>
static const char *shader=R"GLSL(#version 430
layout(local_size_x=128) in;
layout(std430,binding=0) readonly buffer InP {vec4 pts[];};
layout(std430,binding=1) readonly buffer Mesh {vec4 mesh[];};
layout(std430,binding=2) readonly buffer Bounds {vec4 bounds[];};
layout(std430,binding=3) readonly buffer Caps {vec4 caps[];};
layout(std430,binding=4) writeonly buffer Out {uint labels[];};
uniform uint count; uniform uint nlinks; uniform uint ncaps;
void main(){uint id=gl_GlobalInvocationID.x;if(id>=count)return;vec3 p=pts[id].xyz;float best=.0036;
for(uint i=0u;i<nlinks;i++){vec4 lo=bounds[2u*i];vec4 hi=bounds[2u*i+1u];if(any(lessThan(p,lo.xyz-vec3(.06)))||any(greaterThan(p,hi.xyz+vec3(.06))))continue;uint begin=uint(lo.w),end=uint(hi.w);for(uint k=begin;k<end;k++){vec3 d=p-mesh[k].xyz;float ds=dot(d,d);if(ds<.0004){labels[id]=0u;return;}best=min(best,ds);}}
if(best<.0004){labels[id]=0u;return;}
bool margin=false;for(uint i=0u;i<ncaps;i++){vec4 a=caps[2u*i];vec4 b=caps[2u*i+1u];vec3 axis=b.xyz-a.xyz;float den=dot(axis,axis);float t=den>1e-12?clamp(dot(p-a.xyz,axis)/den,0.,1.):0.;vec3 d=p-(a.xyz+t*axis);if(dot(d,d)<=a.w*a.w)margin=true;}
if(best<.0036){for(uint i=0u;i<6u;i++){vec3 d=p-caps[2u*ncaps+i].xyz;if(dot(d,d)<.0196)margin=true;}}
labels[id]=margin?1u:2u;}
)GLSL";
bool readn(void *p,size_t n){return fread(p,1,n,stdin)==n;}
int main(){Display*d=XOpenDisplay(nullptr);if(!d){fprintf(stderr,"X unavailable\n");return 1;}int attr[]={GLX_RGBA,GLX_DOUBLEBUFFER,None};XVisualInfo*v=glXChooseVisual(d,DefaultScreen(d),attr);if(!v)return 2;Colormap c=XCreateColormap(d,RootWindow(d,v->screen),v->visual,AllocNone);XSetWindowAttributes a{};a.colormap=c;Window w=XCreateWindow(d,RootWindow(d,v->screen),0,0,1,1,0,v->depth,InputOutput,v->visual,CWColormap,&a);GLXContext ctx=glXCreateContext(d,v,nullptr,True);if(!ctx||!glXMakeCurrent(d,w,ctx))return 3;glewExperimental=GL_TRUE;if(glewInit()!=GLEW_OK)return 4;fprintf(stderr,"GPU renderer: %s\n",glGetString(GL_RENDERER));
GLuint s=glCreateShader(GL_COMPUTE_SHADER);glShaderSource(s,1,&shader,nullptr);glCompileShader(s);GLint ok=0;glGetShaderiv(s,GL_COMPILE_STATUS,&ok);if(!ok){char log[4096];glGetShaderInfoLog(s,4096,nullptr,log);fprintf(stderr,"%s\n",log);return 5;}GLuint program=glCreateProgram();glAttachShader(program,s);glLinkProgram(program);glGetProgramiv(program,GL_LINK_STATUS,&ok);if(!ok)return 6;glUseProgram(program);GLuint b[5];glGenBuffers(5,b);
while(true){uint32_t h[4];if(!readn(h,sizeof(h)))break;uint32_t n=h[0],m=h[1],l=h[2],cnum=h[3];if(n>1000000||m>1000000||l>100||cnum>20)return 7;std::vector<float>points(n*4),mesh(m*4),bounds(l*8),caps((cnum*2+6)*4);if(!readn(points.data(),points.size()*4)||!readn(mesh.data(),mesh.size()*4)||!readn(bounds.data(),bounds.size()*4)||!readn(caps.data(),caps.size()*4))return 8;const void*data[]={points.data(),mesh.data(),bounds.data(),caps.data(),nullptr};size_t bytes[]={points.size()*4,mesh.size()*4,bounds.size()*4,caps.size()*4,n*4};for(int i=0;i<5;i++){glBindBuffer(GL_SHADER_STORAGE_BUFFER,b[i]);glBufferData(GL_SHADER_STORAGE_BUFFER,bytes[i],data[i],GL_STREAM_DRAW);glBindBufferBase(GL_SHADER_STORAGE_BUFFER,i,b[i]);}glUniform1ui(glGetUniformLocation(program,"count"),n);glUniform1ui(glGetUniformLocation(program,"nlinks"),l);glUniform1ui(glGetUniformLocation(program,"ncaps"),cnum);glDispatchCompute((n+127)/128,1,1);glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT|GL_SHADER_STORAGE_BARRIER_BIT);std::vector<uint32_t>out(n);glBindBuffer(GL_SHADER_STORAGE_BUFFER,b[4]);glGetBufferSubData(GL_SHADER_STORAGE_BUFFER,0,n*4,out.data());if(fwrite(out.data(),4,n,stdout)!=n||fflush(stdout))return 9;}
}
