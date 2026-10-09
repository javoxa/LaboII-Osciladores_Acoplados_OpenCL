#include "simulation.hpp"
#include <GL/freeglut.h>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <chrono>
#include <cerrno>
#include <stdexcept>
#include <iostream>
Recorder::~Recorder(){stop();}
void Recorder::start(const Config&c){
 if(active())return;
 std::filesystem::create_directories(c.output);w=c.width-(c.width%2);h=c.height-(c.height%2);pixels.resize(size_t(w)*h*3);
 auto stamp=std::chrono::system_clock::now().time_since_epoch().count();auto file=c.output+"/ondas_"+std::to_string(stamp)+".mp4";auto size=std::to_string(w)+"x"+std::to_string(h);auto fps=std::to_string(c.fps);
 int pipefd[2];if(pipe(pipefd))throw std::runtime_error("pipe falló");pid=fork();
 if(pid<0){close(pipefd[0]);close(pipefd[1]);throw std::runtime_error("fork falló");}
 if(pid==0){dup2(pipefd[0],STDIN_FILENO);close(pipefd[0]);close(pipefd[1]);execlp("ffmpeg","ffmpeg","-hide_banner","-loglevel","error","-y","-f","rawvideo","-pixel_format","rgb24","-video_size",size.c_str(),"-framerate",fps.c_str(),"-i","pipe:0","-vf","vflip","-c:v","libx264","-pix_fmt","yuv420p",file.c_str(),(char*)nullptr);_exit(127);}
 close(pipefd[0]);fd=pipefd[1];std::cout<<"Grabando: "<<file<<'\n';
}
void Recorder::frame(){if(!active())return;glPixelStorei(GL_PACK_ALIGNMENT,1);glReadBuffer(GL_BACK);glReadPixels(0,0,w,h,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());size_t offset=0;
 while(offset<pixels.size()){auto count=write(fd,pixels.data()+offset,pixels.size()-offset);if(count<0&&errno==EINTR)continue;if(count<=0){stop();throw std::runtime_error("ffmpeg cerró el pipe; revise dependencia/encoder/disco");}offset+=size_t(count);}}
void Recorder::stop(){if(fd<0)return;close(fd);fd=-1;int status=0;while(waitpid(pid,&status,0)<0&&errno==EINTR){}pid=-1;if(!WIFEXITED(status)||WEXITSTATUS(status)!=0)std::cerr<<"ffmpeg terminó con error\n";}
