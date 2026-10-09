#pragma once
#include <CL/cl.h>
#include <vector>
#include <string>
#include <map>
#include <fstream>
struct Config {
 int N=32, width=1200,height=720,max_steps=200,device=0,mode=1,fps=60;
 double m=1,k=40,k1=4,dt=.002,L0=2,L0_acople=1,x0=1,y_eq=0,amplitude=.7,scale=1,radius=.16,total=60;
 bool forced=false;
 double force_amplitude=.5,force_frequency=.4;
 double escala_y=1;
 long long steps=0;
 bool first=false,last=false,hud=true,edit=true,release_zero=false,vsync=true,record_hud=true;
 std::string pattern="pulso_extremo",output="grabaciones";
 std::vector<float> y,v,bg{.025f,.04f,.07f},mass{.2f,.75f,1.f};
 static Config load(const std::string&);
};
// Menu state. Physical parameters remain in Config, read from the file.
struct SimParams {
 int N = 20;
 bool primero_fijo = true;
 bool ultimo_fijo = true;
 bool indefinido = false;
};
std::string readFile(const std::string&);
std::vector<float> initial(const Config&);
std::vector<float> acceleration(const Config&,const std::vector<float>&);
double energy(const Config&,const std::vector<float>&,const std::vector<float>&,double* kinetic=nullptr);
class Dynamics {
 Config c; double drive_phase=0; cl_context context=nullptr; cl_command_queue queue=nullptr; cl_program program=nullptr; cl_kernel drift=nullptr,kick=nullptr;
 cl_mem pos=nullptr,vel=nullptr,next=nullptr,half=nullptr;
 public:
 std::vector<float> y,v,previous; long long iteration=0;
 explicit Dynamics(const Config&); ~Dynamics(); void reset(); void setDrive(bool,double,double); void reconfigure(const Config&); void step(int dragged=-1,float target=0,float speed=0); void read(); void set(int,float,float);
};
class Recorder {
 int fd=-1,pid=-1,w=0,h=0; std::vector<unsigned char> pixels;
 public: ~Recorder(); bool active()const{return fd>=0;} void start(const Config&); void frame(); void stop();
};
