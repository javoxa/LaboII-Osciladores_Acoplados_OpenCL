#include "simulation.hpp"
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <sstream>
// Common baseline for Clover 1.1, PoCL and newer NVIDIA/AMD runtimes.
static constexpr const char* buildOptions="-cl-std=CL1.1";
static void check(cl_int e){if(e!=CL_SUCCESS)throw std::runtime_error("Error OpenCL "+std::to_string(e));}
static std::string deviceText(cl_device_id device,cl_device_info key){
 size_t size=0;auto error=clGetDeviceInfo(device,key,0,nullptr,&size);
 if(error!=CL_SUCCESS)return "no disponible ("+std::to_string(error)+")";
 std::string value(size,'\0');error=clGetDeviceInfo(device,key,size,value.data(),nullptr);
 if(error!=CL_SUCCESS)return "no disponible ("+std::to_string(error)+")";
 while(!value.empty()&&value.back()=='\0')value.pop_back();
 return value;
}
static std::string buildFailure(cl_program program,cl_device_id device,cl_int error){
 const char*name=error==CL_BUILD_PROGRAM_FAILURE?"CL_BUILD_PROGRAM_FAILURE":
 error==CL_COMPILER_NOT_AVAILABLE?"CL_COMPILER_NOT_AVAILABLE":
 error==CL_INVALID_BUILD_OPTIONS?"CL_INVALID_BUILD_OPTIONS":"error de build";
 std::ostringstream out;out<<"clBuildProgram: "<<name<<" ("<<error<<")\nOpciones: "<<buildOptions<<"\n";
 cl_build_status status;auto statusError=clGetProgramBuildInfo(program,device,CL_PROGRAM_BUILD_STATUS,sizeof(status),&status,nullptr);
 if(statusError==CL_SUCCESS)out<<"Estado del build: "<<status<<'\n';
 else out<<"No se pudo consultar estado: "<<statusError<<'\n';
 size_t size=0;auto logError=clGetProgramBuildInfo(program,device,CL_PROGRAM_BUILD_LOG,0,nullptr,&size);
 std::string log;
 if(logError==CL_SUCCESS&&size){log.resize(size);logError=clGetProgramBuildInfo(program,device,CL_PROGRAM_BUILD_LOG,size,log.data(),nullptr);}
 while(!log.empty()&&log.back()=='\0')log.pop_back();
 if(logError!=CL_SUCCESS)out<<"No se pudo consultar log: "<<logError;
 else if(log.find_first_not_of(" \t\r\n")==std::string::npos)out<<"El runtime no proporciono un log de compilacion.";
 else out<<"Log de compilacion:\n"<<log;
 return out.str();
}
Dynamics::Dynamics(const Config&cfg):c(cfg){
 try {
 cl_uint n=0;check(clGetPlatformIDs(0,nullptr,&n));std::vector<cl_platform_id>p(n);check(clGetPlatformIDs(n,p.data(),nullptr));std::vector<cl_device_id>devices;
 for(auto platform:p){cl_uint count=0;auto e=clGetDeviceIDs(platform,CL_DEVICE_TYPE_ALL,0,nullptr,&count);if(e==CL_DEVICE_NOT_FOUND)continue;check(e);std::vector<cl_device_id>d(count);check(clGetDeviceIDs(platform,CL_DEVICE_TYPE_ALL,count,d.data(),nullptr));devices.insert(devices.end(),d.begin(),d.end());}
 if(c.device>=int(devices.size()))throw std::runtime_error("No existe el dispositivo OpenCL solicitado");
 auto d=devices[c.device];char name[256];check(clGetDeviceInfo(d,CL_DEVICE_NAME,sizeof(name),name,nullptr));std::cout<<"OpenCL: "<<name<<'\n';
 std::cout<<"Vendor: "<<deviceText(d,CL_DEVICE_VENDOR)
 <<"\nDriver: "<<deviceText(d,CL_DRIVER_VERSION)
 <<"\nRuntime: "<<deviceText(d,CL_DEVICE_VERSION)
 <<"\nOpenCL C: "<<deviceText(d,CL_DEVICE_OPENCL_C_VERSION)<<'\n';
 cl_bool compiler=CL_FALSE;check(clGetDeviceInfo(d,CL_DEVICE_COMPILER_AVAILABLE,sizeof(compiler),&compiler,nullptr));
 std::cout<<"Compilador disponible: "<<(compiler?"si":"no")<<std::endl;
 if(!compiler)throw std::runtime_error("CL_DEVICE_COMPILER_AVAILABLE=false: este runtime no puede compilar kernels desde fuente");
 cl_int e;context=clCreateContext(nullptr,1,&d,nullptr,nullptr,&e);check(e);queue=clCreateCommandQueue(context,d,0,&e);check(e);
 auto source=readFile(std::string(RESOURCE_DIR)+"/kernels/verlet.cl");auto ptr=source.c_str();program=clCreateProgramWithSource(context,1,&ptr,nullptr,&e);check(e);e=clBuildProgram(program,1,&d,buildOptions,nullptr,nullptr);
 if(e!=CL_SUCCESS)throw std::runtime_error(buildFailure(program,d,e));
 drift=clCreateKernel(program,"drift",&e);check(e);kick=clCreateKernel(program,"kick",&e);check(e);
 reconfigure(c);
 }catch(...){releaseResources();throw;}
}
Dynamics::~Dynamics(){releaseResources();}
void Dynamics::releaseResources(){for(auto b:{pos,vel,next,half})if(b)clReleaseMemObject(b);if(drift)clReleaseKernel(drift);if(kick)clReleaseKernel(kick);if(program)clReleaseProgram(program);if(queue)clReleaseCommandQueue(queue);if(context)clReleaseContext(context);}
// Preserve context, queue and compiled kernels. Build replacement buffers first
// so allocation failures do not destroy the current simulation.
void Dynamics::reconfigure(const Config&cfg){
 Config replacement=cfg;
 auto newY=initial(replacement);
 auto newV=replacement.v.empty()?std::vector<float>(replacement.N,0):replacement.v;
 if(newY.size()!=size_t(replacement.N)||newV.size()!=size_t(replacement.N))
  throw std::runtime_error("Condiciones iniciales incompatibles con N");
 if(replacement.first)newV.front()=0;
 if(replacement.last)newV.back()=0;
 auto newPrevious=newY;
 cl_mem buffers[4]={nullptr,nullptr,nullptr,nullptr};
 try {
  check(clFinish(queue));
  for(int i=0;i<4;i++){
   cl_int error;
   float*data=i==1?newV.data():newY.data();
   buffers[i]=clCreateBuffer(context,CL_MEM_READ_WRITE|CL_MEM_COPY_HOST_PTR,
                            newY.size()*sizeof(float),data,&error);
   check(error);
  }
 }catch(...){for(auto buffer:buffers)if(buffer)clReleaseMemObject(buffer);throw;}
 for(auto buffer:{pos,vel,next,half})if(buffer)clReleaseMemObject(buffer);
 pos=buffers[0];vel=buffers[1];next=buffers[2];half=buffers[3];
 c=std::move(replacement);y=std::move(newY);v=std::move(newV);
 previous=std::move(newPrevious);iteration=0;drive_phase=0;
}
void Dynamics::reset(){y=initial(c);v=c.v.empty()?std::vector<float>(c.N,0):c.v;if(c.first)v[0]=0;if(c.last)v.back()=0;previous=y;iteration=0;drive_phase=0;check(clEnqueueWriteBuffer(queue,pos,CL_TRUE,0,y.size()*sizeof(float),y.data(),0,nullptr,nullptr));check(clEnqueueWriteBuffer(queue,vel,CL_TRUE,0,v.size()*sizeof(float),v.data(),0,nullptr,nullptr));}
void Dynamics::setDrive(bool enabled,double amplitude,double frequency){
 if(!std::isfinite(amplitude)||!std::isfinite(frequency)||amplitude<0||frequency<=0)
  throw std::runtime_error("Parametros de fuerza invalidos");
 c.forced=enabled;c.force_amplitude=amplitude;c.force_frequency=frequency;
}
void Dynamics::step(int dragged,float target,float speed){
 auto arg=[](cl_kernel kernel,cl_uint&i,const auto&value){check(clSetKernelArg(kernel,i++,sizeof(value),&value));};
 double phase_next=std::remainder(drive_phase+6.283185307179586*c.force_frequency*c.dt,6.283185307179586);
 float force0=c.forced?float(c.force_amplitude*std::sin(drive_phase)):0;
 float force1=c.forced?float(c.force_amplitude*std::sin(phase_next)):0;
 float m=c.m,k=c.k,k1=c.k1,eq=c.y_eq,dt=c.dt;int first=c.first,last=c.last;cl_uint i=0;
 for(cl_mem b:{pos,vel,next,half})arg(drift,i,b);
 arg(drift,i,c.N);for(float x:{m,k,k1,eq,dt})arg(drift,i,x);for(int x:{first,last,dragged})arg(drift,i,x);arg(drift,i,target);arg(drift,i,speed);arg(drift,i,force0);
 size_t size=c.N;check(clEnqueueNDRangeKernel(queue,drift,1,nullptr,&size,nullptr,0,nullptr,nullptr));i=0;
 for(cl_mem b:{next,half,vel})arg(kick,i,b);
 arg(kick,i,c.N);for(float x:{m,k,k1,eq,dt})arg(kick,i,x);for(int x:{first,last,dragged})arg(kick,i,x);arg(kick,i,speed);arg(kick,i,force1);
 check(clEnqueueNDRangeKernel(queue,kick,1,nullptr,&size,nullptr,0,nullptr,nullptr));std::swap(pos,next);++iteration;drive_phase=phase_next;
}
void Dynamics::read(){check(clEnqueueReadBuffer(queue,next,CL_TRUE,0,previous.size()*sizeof(float),previous.data(),0,nullptr,nullptr));check(clEnqueueReadBuffer(queue,pos,CL_TRUE,0,y.size()*sizeof(float),y.data(),0,nullptr,nullptr));check(clEnqueueReadBuffer(queue,vel,CL_TRUE,0,v.size()*sizeof(float),v.data(),0,nullptr,nullptr));}
void Dynamics::set(int i,float value,float speed){y.at(i)=value;v.at(i)=speed;previous.at(i)=value;check(clEnqueueWriteBuffer(queue,pos,CL_TRUE,i*sizeof(float),sizeof(float),&value,0,nullptr,nullptr));check(clEnqueueWriteBuffer(queue,vel,CL_TRUE,i*sizeof(float),sizeof(float),&speed,0,nullptr,nullptr));}
