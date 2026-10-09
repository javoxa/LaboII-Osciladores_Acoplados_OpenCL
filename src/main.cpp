#include "simulation.hpp"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#ifdef __linux__
#include <GL/glx.h>
#endif
#include <chrono>
#include <memory>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <csignal>
#include <cctype>
#include <algorithm>
#include <stdexcept>
static Config c;static std::unique_ptr<Dynamics>d;static Recorder recorder;
static bool paused=false,visible=true,renderTest=false,menuVisible=true;static int testFrames=0;static int drag=-1;static float target=0,speed=0;static double acc=0,dropped=0,mouseTime=0,captureAcc=0;static SimParams params;static bool needs_reset=false;static Config fileConfig;
static auto last=std::chrono::steady_clock::now();static std::vector<float>renderY;
static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static bool fixed(int i){return(i==0&&c.first)||(i==c.N-1&&c.last);}
static float leftBound,rightBound,bottomBound,topBound;
static void camera(){
 double amplitude=std::max(.5,std::abs(c.amplitude));for(float y:c.y)amplitude=std::max(amplitude,std::abs(double(y-c.y_eq)));
 double vertical=std::max(c.L0+2*amplitude+1.,std::max(c.x0*2,(c.N-1)*c.x0+2*c.x0)*c.height/c.width)/c.scale;
 double span=vertical*c.width/c.height,center=(c.N-1)*c.x0*.5,cy=(-c.L0+c.y_eq)*.5;
 leftBound=center-span*.5;rightBound=center+span*.5;bottomBound=cy-vertical*.5;topBound=cy+vertical*.5;
}
static void sceneProjection(){
 glMatrixMode(GL_PROJECTION);glLoadIdentity();
 glOrtho(leftBound,rightBound,bottomBound,topBound,-1,1);
 glMatrixMode(GL_MODELVIEW);glLoadIdentity();
}
static void geometry(const std::vector<float>&points,GLenum type,const std::vector<float>&color){
 sceneProjection();glColor3fv(color.data());glBegin(type);
 for(size_t i=0;i+1<points.size();i+=2)glVertex2f(points[i],points[i+1]);
 glEnd();
}
static void spring(float x1,float y1,float x2,float y2,float natural){std::vector<float>pts;float dx=x2-x1,dy=y2-y1,len=std::hypot(dx,dy);for(int j=0;j<=24;j++){float t=j/24.f,zig=(j==0||j==24)?0:((j%2)?1:-1)*std::min(c.x0*.09,c.radius*.6);pts.push_back(x1+t*dx-(len?dy/len:0)*zig);pts.push_back(y1+t*dy+(len?dx/len:0)*zig);}float stretch=std::min(1.f,std::abs(len-natural)/float(std::max(natural,.01f)));geometry(pts,GL_LINE_STRIP,{.35f+.6f*stretch,.55f-.3f*stretch,.55f-.3f*stretch});}
static void text(int x,int y,const std::string&s){
 glMatrixMode(GL_PROJECTION);glPushMatrix();glLoadIdentity();glOrtho(0,c.width,0,c.height,-1,1);
 glMatrixMode(GL_MODELVIEW);glPushMatrix();glLoadIdentity();
 glRasterPos2i(x,y);for(unsigned char ch:s)glutBitmapCharacter(GLUT_BITMAP_8_BY_13,ch);
 glPopMatrix();glMatrixMode(GL_PROJECTION);glPopMatrix();glMatrixMode(GL_MODELVIEW);
}
static void restoreMenuParams(){params.N=c.N;params.primero_fijo=c.first;params.ultimo_fijo=c.last;}
static bool reachedLimit(const Config&cfg,long long iteration){
 return (cfg.steps>0&&iteration>=cfg.steps)||
        (cfg.total>0&&(iteration+1)*cfg.dt>cfg.total+1e-12);
}
static void toggleUnlimited(){
 bool wasAtLimit=reachedLimit(c,d->iteration);
 params.indefinido=!params.indefinido;
 c.total=params.indefinido?0:fileConfig.total;
 c.steps=params.indefinido?0:fileConfig.steps;
 if(params.indefinido&&wasAtLimit){paused=false;acc=0;last=std::chrono::steady_clock::now();}
}
static void cycleBorder(){
 if(!params.primero_fijo&&!params.ultimo_fijo){params.primero_fijo=params.ultimo_fijo=true;}
 else if(params.primero_fijo&&params.ultimo_fijo){params.primero_fijo=false;params.ultimo_fijo=true;}
 else {params.primero_fijo=params.ultimo_fijo=false;}
 needs_reset=true;
}
static void updateDrive(){d->setDrive(c.forced,c.force_amplitude,c.force_frequency);}
static void toggleDrive(){
 c.forced=!c.forced;
 if(c.forced&&(c.first||params.primero_fijo)){
  params.primero_fijo=false;needs_reset=true;
 }
 updateDrive();
}
static void changeForce(double delta){c.force_amplitude=std::max(0.,c.force_amplitude+delta);updateDrive();}
static void changeFrequency(double delta){c.force_frequency=std::max(.01,c.force_frequency+delta);updateDrive();}
static void cycleOverlay(){
 if(c.hud&&menuVisible)c.hud=false;
 else if(!c.hud&&menuVisible)menuVisible=false;
 else if(!c.hud&&!menuVisible)c.hud=true;
 else menuVisible=true;
}
static void changeScaleY(double factor){
 // Change visual displacement only; preserve physical mouse position if dragging.
 c.escala_y=std::clamp(c.escala_y*factor,.01,1000.);
}
static double displayY(double physicalY){return -c.L0+c.y_eq+c.escala_y*(physicalY-c.y_eq);}
static void drawMenu(){
 if(!menuVisible)return;
 glPushAttrib(GL_ENABLE_BIT|GL_CURRENT_BIT|GL_POLYGON_BIT);
 glDisable(GL_DEPTH_TEST);glDisable(GL_LIGHTING);glDisable(GL_TEXTURE_2D);
 glMatrixMode(GL_PROJECTION);glPushMatrix();glLoadIdentity();glOrtho(0,c.width,0,c.height,-1,1);
 glMatrixMode(GL_MODELVIEW);glPushMatrix();glLoadIdentity();
 int x=std::max(5,c.width-300),y=c.height-70;
 glColor3f(.10f,.14f,.20f);glBegin(GL_QUADS);
 glVertex2i(x,y);glVertex2i(x+290,y);glVertex2i(x+290,y-310);glVertex2i(x,y-310);glEnd();
 glColor3f(.9f,.95f,1.f);
 text(x+10,y-20,"Osciladores: N="+std::to_string(params.N));
 text(x+10,y-45,"[-]     [+]   (abajo / arriba)");
 const char*border=params.primero_fijo?(params.ultimo_fijo?"Fijos":"Primero fijo / ultimo libre"):(params.ultimo_fijo?"Primero libre / ultimo fijo":"Libres");
 text(x+10,y-70,std::string("Borde: ")+border);
 text(x+10,y-95,"[ Libre primero / fijo ultimo ]");
 text(x+10,y-120,std::string(params.indefinido?"[X]":"[ ]")+" Tiempo indefinido  [I]");
 text(x+10,y-145,"[ Reiniciar ]   R | F: borde");
 text(x+10,y-175,std::string(c.forced?"[X]":"[ ]")+" Primera masa forzada [O]");
 std::ostringstream driveText;driveText<<std::fixed<<std::setprecision(2)<<"F0="<<c.force_amplitude<<" N";
 text(x+10,y-200,driveText.str());text(x+210,y-200,"[-] [+]");driveText.str("");driveText<<"f="<<c.force_frequency<<" Hz";text(x+10,y-225,driveText.str());text(x+210,y-225,"[-] [+]");
 text(x+10,y-250,"Fuerza: [ ] | Frecuencia: , .");
 driveText.str("");driveText<<"Escala Y: x"<<c.escala_y; text(x+10,y-275,driveText.str());text(x+210,y-275,"[-] [+]");
 text(x+10,y-300,needs_reset?"Cambio pendiente":"N / borde reinician simulacion");
 glPopMatrix();glMatrixMode(GL_PROJECTION);glPopMatrix();glMatrixMode(GL_MODELVIEW);glPopAttrib();
}
static void requestN(int delta){params.N=std::clamp(params.N+delta,2,200);needs_reset=true;}
static void applyReset(){
 if(!needs_reset)return;
 needs_reset=false;drag=-1;target=0;speed=0;mouseTime=0;
 Config nextConfig=fileConfig;
 nextConfig.N=params.N;nextConfig.first=params.primero_fijo;nextConfig.last=params.ultimo_fijo;
 nextConfig.forced=c.forced;nextConfig.force_amplitude=c.force_amplitude;nextConfig.force_frequency=c.force_frequency;
 if(nextConfig.first)nextConfig.forced=false;
 if(params.indefinido){nextConfig.total=0;nextConfig.steps=0;}
 // Explicit file profiles: preserve prefix, truncate or fill added masses.
 if(!nextConfig.y.empty())nextConfig.y.resize(params.N,float(nextConfig.y_eq));
 if(!nextConfig.v.empty())nextConfig.v.resize(params.N,0.f);
 int mobile=params.N-int(params.primero_fijo)-int(params.ultimo_fijo);
 if(nextConfig.pattern=="modo_normal"){
  if(mobile==0)nextConfig.pattern="cero";
  else nextConfig.mode=std::clamp(nextConfig.mode,1,mobile);
  if((params.primero_fijo||params.ultimo_fijo)&&nextConfig.y_eq!=0){
   std::cerr<<"Modo normal con extremos fijos requiere y_eq=0; cambio rechazado\n";
   restoreMenuParams();return;
  }
 }
 try{d->reconfigure(nextConfig);}catch(const std::exception&e){
  std::cerr<<"Reset rechazado: "<<e.what()<<'\n';restoreMenuParams();return;
 }
 nextConfig.width=c.width;nextConfig.height=c.height;
 nextConfig.escala_y=c.escala_y;nextConfig.hud=c.hud;nextConfig.edit=c.edit;nextConfig.record_hud=c.record_hud;
 c=std::move(nextConfig);renderY=d->y;acc=0;dropped=0;paused=false;camera();
 last=std::chrono::steady_clock::now();
}
static void hud(){double T,E=energy(c,d->y,d->v,&T);glColor3f(.9,.95,1);std::ostringstream s;s<<std::fixed<<std::setprecision(4)<<"t="<<d->iteration*c.dt<<"  dt="<<c.dt<<"  paso="<<d->iteration<<"  "<<(paused?"PAUSA":"ACTIVA");text(15,c.height-25,s.str());s.str("");s<<"Ec="<<T<<"  Ep="<<E-T<<"  E="<<E<<"  atraso descartado="<<dropped<<"s";text(15,c.height-45,s.str());text(15,20,"Espacio: pausa | R: reinicio | E: arrastre | H: HUD | A: animacion | G: MP4 | B: HUD MP4 | menu: click derecho");
 text(25,145,"Perfil y_i - y_eq vs indice i (escala vertical automatica)");
 // Mini graph uses fixed-function projection with temporarily changed world bounds.
 auto l=leftBound,r=rightBound,b=bottomBound,t=topBound;leftBound=-.1*c.N;rightBound=c.N*4;bottomBound=-4;topBound=16;
 std::vector<float>pts;double amplitude=std::max(.1,std::abs(c.amplitude));for(float y:renderY)amplitude=std::max(amplitude,std::abs(double(y-c.y_eq)));for(int i=0;i<c.N;i++){pts.push_back(i);pts.push_back((renderY[i]-c.y_eq)/amplitude+1);}geometry(pts,GL_LINE_STRIP,{.4f,1,.6f});leftBound=l;rightBound=r;bottomBound=b;topBound=t;
}
static void drawScene(){if(!visible)return;geometry({leftBound,0,rightBound,0},GL_LINES,{.8f,.8f,.85f});for(int i=0;i<c.N;i++){float x=i*c.x0,y=displayY(renderY[i]);spring(x,0,x,y,c.L0);if(i)spring((i-1)*c.x0,displayY(renderY[i-1]),x,y,c.L0_acople);
 if(fixed(i)){float r=c.radius*1.5;geometry({x-r,y-r,x+r,y-r,x+r,y+r,x-r,y+r,x-r,y-r},GL_LINE_STRIP,{1,.6f,.2f});}
 std::vector<float>pts{x,y};for(int j=0;j<=32;j++){double theta=j*6.283185307179586/32;pts.push_back(x+c.radius*std::cos(theta));pts.push_back(y+c.radius*std::sin(theta));}geometry(pts,GL_TRIANGLE_FAN,i==drag?std::vector<float>{1,.3f,.2f}:(i==0&&c.forced?std::vector<float>{1,.75f,.15f}:c.mass));}}
static void fail(const std::exception&e){std::cerr<<e.what()<<'\n';recorder.stop();glutLeaveMainLoop();}
static bool screenToWorld(int,int,float&,float&);
static void display(){try{
 applyReset();
 if(renderTest&&testFrames==0)recorder.start(c);
 // Exercise frame-boundary resets in the optional visual smoke test.
 if(renderTest&&(testFrames==5||testFrames==10||testFrames==15)){
  params.N=testFrames==5?200:(testFrames==10?2:20);
  params.primero_fijo=false;params.ultimo_fijo=testFrames==10;needs_reset=true;
 }
 auto current=std::chrono::steady_clock::now();double elapsed=std::chrono::duration<double>(current-last).count();last=current;
 if(!paused){acc+=elapsed;int count=0;while(acc>=c.dt&&count<c.max_steps){if(reachedLimit(c,d->iteration)){paused=true;acc=0;break;}d->step(drag,target,(now()-mouseTime>.08)?0:speed);acc-=c.dt;++count;}if(count)d->read();if(count==c.max_steps&&acc>=c.dt){double retain=std::fmod(acc,c.dt);dropped+=acc-retain;acc=retain;}}
 renderY=d->y;double alpha=paused?1:std::clamp(acc/c.dt,0.,1.);for(int i=0;i<c.N;i++)renderY[i]=d->previous[i]*(1-alpha)+d->y[i]*alpha;if(drag>=0)renderY[drag]=target;
 glViewport(0,0,c.width,c.height);glClearColor(c.bg[0],c.bg[1],c.bg[2],1);glClear(GL_COLOR_BUFFER_BIT);sceneProjection();drawScene();
 int capture=0;if(recorder.active()){captureAcc+=elapsed;capture=int(captureAcc*c.fps);captureAcc-=double(capture)/c.fps;}
 if(capture){if(c.record_hud)hud();for(int i=0;i<capture;i++)recorder.frame();if(c.record_hud&&!c.hud){glClear(GL_COLOR_BUFFER_BIT);sceneProjection();drawScene();}else if(c.hud&&!c.record_hud)hud();}else if(c.hud)hud();drawMenu();sceneProjection();
 if(renderTest){
  GLdouble model[16],projection[16],sx,sy,sz;GLint viewport[4];
  glGetDoublev(GL_MODELVIEW_MATRIX,model);glGetDoublev(GL_PROJECTION_MATRIX,projection);glGetIntegerv(GL_VIEWPORT,viewport);
  float wx,wy;double x=(leftBound+rightBound)*.5,y=(bottomBound+topBound)*.5;
  if(!gluProject(x,y,0,model,projection,viewport,&sx,&sy,&sz)||
     !screenToWorld(int(sx),viewport[3]-1-int(sy),wx,wy)||
     std::abs(wx-x)>(rightBound-leftBound)/c.width||
     std::abs(displayY(wy)-y)>(topBound-bottomBound)/c.height)
   throw std::runtime_error("Picking GLU incorrecto");
  if(glGetError()!=GL_NO_ERROR)throw std::runtime_error("Error de render OpenGL");
  if(testFrames==0){
   std::vector<unsigned char>pixels(size_t(c.width)*c.height*3);
   glPixelStorei(GL_PACK_ALIGNMENT,1);glReadBuffer(GL_BACK);
   glReadPixels(0,0,c.width,c.height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
   std::ofstream preview(std::string(RESOURCE_DIR)+"/build/menu_preview.ppm",std::ios::binary);
   preview<<"P6\n"<<c.width<<" "<<c.height<<"\n255\n";
   for(int row=c.height-1;row>=0;--row)preview.write(reinterpret_cast<char*>(pixels.data()+size_t(row)*c.width*3),c.width*3);
  }
 }
 glutSwapBuffers();if(renderTest&&++testFrames>=20){recorder.stop();glutLeaveMainLoop();}
 }catch(const std::exception&e){fail(e);}}
static void timer(int){glutPostRedisplay();glutTimerFunc(std::max(1,1000/c.fps),timer,0);}
static void finishDrag(){if(needs_reset){drag=-1;return;}if(drag<0)return;float release=(c.release_zero||now()-mouseTime>.08)?0:speed;d->set(drag,target,release);drag=-1;}
static void action(int key){try{switch(key){case ' ':paused=!paused;acc=0;break;case 'r':needs_reset=true;break;case 'f':cycleBorder();break;case 'i':toggleUnlimited();break;case 'o':toggleDrive();break;case '[':changeForce(-.1);break;case ']':changeForce(.1);break;case ',':changeFrequency(-.01);break;case '.':changeFrequency(.01);break;case 'h':cycleOverlay();break;case '+':case '=':changeScaleY(1.25);break;case '-':changeScaleY(1/1.25);break;case 'a':visible=!visible;break;case 'e':finishDrag();c.edit=!c.edit;break;case 'b':c.record_hud=!c.record_hud;break;case 'g':if(recorder.active())recorder.stop();else {recorder.start(c);captureAcc=0;}break;case 27:recorder.stop();glutLeaveMainLoop();break;}}catch(const std::exception&e){std::cerr<<e.what()<<'\n';}}
static void keyboard(unsigned char key,int,int){action(std::tolower(key));}
static bool screenToWorld(int x,int y,float&wx,float&wy){
 // Restore scene matrices explicitly; HUD/menu projections must not affect picking.
 sceneProjection();GLdouble model[16],projection[16],px,py,pz;GLint viewport[4];
 glGetDoublev(GL_MODELVIEW_MATRIX,model);glGetDoublev(GL_PROJECTION_MATRIX,projection);
 glGetIntegerv(GL_VIEWPORT,viewport);
 if(!gluUnProject(x,viewport[3]-1-y,.5,model,projection,viewport,&px,&py,&pz))return false;
 wx=float(px);wy=float(c.y_eq+(py+c.L0-c.y_eq)/c.escala_y);return true;
}
static void special(int key,int,int){if(key==GLUT_KEY_UP)requestN(1);else if(key==GLUT_KEY_DOWN)requestN(-1);}
static void mouse(int button,int state,int x,int y){if(button!=GLUT_LEFT_BUTTON)return;try{if(state==GLUT_UP){finishDrag();return;}int panelX=std::max(5,c.width-300),localY=y-70;
 if(menuVisible&&x>=panelX&&x<=panelX+290&&localY>=0&&localY<=310){
  if(localY>=25&&localY<55)requestN(x<panelX+65?-1:1);
  else if(localY>=55&&localY<80)cycleBorder();
  else if(localY>=80&&localY<105){params.primero_fijo=false;params.ultimo_fijo=true;needs_reset=true;}
  else if(localY>=105&&localY<130)toggleUnlimited();
  else if(localY>=130&&localY<155)needs_reset=true;
  else if(localY>=155&&localY<185)toggleDrive();
  else if(localY>=185&&localY<210&&x>=panelX+200)changeForce(x<panelX+240?-.1:.1);
  else if(localY>=210&&localY<240&&x>=panelX+200)changeFrequency(x<panelX+240?-.01:.01);
  else if(localY>=260&&localY<285&&x>=panelX+200)changeScaleY(x<panelX+240?1/1.25:1.25);
  return;
 }
 if(needs_reset||!c.edit||!visible)return;
 float wx,wy;if(!screenToWorld(x,y,wx,wy))return;double best=1e9;int pick=-1;float tolerance=std::max(float(c.radius)*1.6f,(rightBound-leftBound)*12/c.width);for(int i=0;i<c.N;i++){double distance=std::hypot(wx-i*c.x0,c.escala_y*(wy-renderY[i]));if(!fixed(i)&&distance<tolerance&&distance<best){best=distance;pick=i;}}if(pick>=0){drag=pick;target=d->y[pick];speed=0;mouseTime=now();}}catch(const std::exception&e){fail(e);}}
static void motion(int x,int y){if(needs_reset||drag<0)return;try{double t=now(),delta=t-mouseTime;float wx,value;if(!screenToWorld(x,y,wx,value))return;speed=delta>.001?(value-target)/delta:0;target=value;mouseTime=t;d->set(drag,target,speed);}catch(const std::exception&e){fail(e);}}
static void resize(int w,int h){if(recorder.active()&&(w!=c.width||h!=c.height)){recorder.stop();std::cout<<"Grabacion finalizada por cambio de tamaño\n";}c.width=w;c.height=std::max(h,1);camera();}
static int selfTest(){Config cfg;cfg.N=17;cfg.first=cfg.last=true;cfg.pattern="modo_normal";cfg.mode=2;cfg.total=0;Dynamics sim(cfg);double E=energy(cfg,sim.y,sim.v),maxerr=0;for(int i=0;i<10000;i++){sim.step();if(i%100==0){sim.read();maxerr=std::max(maxerr,std::abs(energy(cfg,sim.y,sim.v)-E)/E);if(sim.y.front()!=0||sim.y.back()!=0)throw std::runtime_error("Extremos no fijos");}}if(maxerr>.002)throw std::runtime_error("Deriva energetica excesiva");
 cfg.N=1;cfg.first=cfg.last=false;cfg.k=0;cfg.k1=4;cfg.y={.5f};Dynamics single(cfg);for(int i=0;i<500;i++)single.step();single.read();if(std::abs(single.y[0]-.5*std::cos(2.))>1e-4)throw std::runtime_error("Oscilador analitico incorrecto");
 cfg.N=3;cfg.y={1,0,0};cfg.k1=0;cfg.k=4;cfg.v={0,0,0};Dynamics free(cfg);for(int i=0;i<500;i++)free.step();free.read();double momentum=0;for(float v:free.v)momentum+=v;if(std::abs(momentum)>1e-4)throw std::runtime_error("Momento con k1=0 incorrecto");free.step(1,.7f,.2f);free.read();if(std::abs(free.y[1]-.7)>1e-6||std::abs(free.v[1]-.2)>1e-6)throw std::runtime_error("Arrastre incorrecto");for(int cycle=0;cycle<60;cycle++){
 Config changed;changed.N=cycle%2?2:200;changed.first=cycle%3==0;changed.last=cycle%3!=1;
 free.reconfigure(changed);
 if(free.iteration!=0||free.y.size()!=size_t(changed.N)||free.previous!=free.y)
  throw std::runtime_error("Reset incompleto");
 for(int step=0;step<5;step++)free.step();
 free.read();
 for(float y:free.y)if(!std::isfinite(y))throw std::runtime_error("Reset produjo datos no finitos");
 if((changed.first&&free.y.front()!=0)||(changed.last&&free.y.back()!=0))throw std::runtime_error("Reset de bordes incorrecto");
 }
 Config limitTest;limitTest.dt=.25;limitTest.total=60;
 if(!reachedLimit(limitTest,240))throw std::runtime_error("Limite temporal incorrecto");
 limitTest.total=0;limitTest.steps=0;
 if(reachedLimit(limitTest,1000000000))throw std::runtime_error("Modo indefinido limitado");
 Config mixed; mixed.N=20;mixed.first=false;mixed.last=true;mixed.total=0;
 free.reconfigure(mixed);for(int step=0;step<200;step++)free.step();free.read();
 if(free.y.back()!=0||free.v.back()!=0||std::abs(free.y.front())<1e-5)
  throw std::runtime_error("Primero libre / ultimo fijo incorrecto");
 Config forced;forced.N=1;forced.k=0;forced.k1=4;forced.pattern="cero";forced.forced=true;forced.force_amplitude=.5;forced.force_frequency=1./6.283185307179586;
 free.reconfigure(forced);for(int step=0;step<500;step++)free.step();free.read();
 double expected=.5/3.*(std::sin(1.)-.5*std::sin(2.));
 if(std::abs(free.y[0]-expected)>1e-5)throw std::runtime_error("Oscilador forzado analitico incorrecto");
 auto oldIteration=free.iteration;free.setDrive(true,1,.5);
 if(free.iteration!=oldIteration)throw std::runtime_error("Cambiar fuerza reinicio la dinamica");
 free.step();free.read();if(!std::isfinite(free.y[0]))throw std::runtime_error("Cambio de fuerza invalido");
 c.hud=true;menuVisible=true;
 cycleOverlay();if(c.hud||!menuVisible)throw std::runtime_error("Primer H incorrecto");
 cycleOverlay();if(c.hud||menuVisible)throw std::runtime_error("Segundo H incorrecto");
 cycleOverlay();if(!c.hud||menuVisible)throw std::runtime_error("Tercer H incorrecto");
 cycleOverlay();if(!c.hud||!menuVisible)throw std::runtime_error("Cuarto H incorrecto");
 std::cout<<"Pruebas OpenCL OK; error relativo energia="<<maxerr<<'\n';return 0;}
int main(int argc,char**argv){try{
 std::signal(SIGPIPE,SIG_IGN);if(argc>1&&std::string(argv[1])=="--self-test")return selfTest();std::string path=argc>1?argv[1]:std::string(RESOURCE_DIR)+"/parameters/ejemplo.cfg";c=Config::load(path);fileConfig=c;restoreMenuParams();params.indefinido=c.total==0&&c.steps==0;
 if(argc>2&&std::string(argv[2])=="--validate"){std::cout<<"Configuracion valida\n";return 0;}
 renderTest=argc>2&&std::string(argv[2])=="--render-test";
 d=std::make_unique<Dynamics>(c);
 if(argc>2&&std::string(argv[2])=="--headless"){long long timeLimit=c.total>0?static_cast<long long>(std::floor(c.total/c.dt+1e-10)):0;long long limit=c.steps>0?(timeLimit>0?std::min(c.steps,timeLimit):c.steps):timeLimit;if(limit<=0)throw std::runtime_error("headless requiere limite temporal");double initialE=energy(c,d->y,d->v);for(long long i=0;i<limit;i++)d->step();d->read();std::cout<<"pasos="<<d->iteration<<" E0="<<initialE<<" E="<<energy(c,d->y,d->v)<<'\n';return 0;}
 glutInit(&argc,argv);glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB);glutInitContextVersion(1,2);glutInitWindowSize(c.width,c.height);glutCreateWindow("Ondas mecanicas - osciladores OpenCL/OpenGL");glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#ifdef __linux__
 auto swap=(PFNGLXSWAPINTERVALEXTPROC)glXGetProcAddressARB((const GLubyte*)"glXSwapIntervalEXT");if(swap)swap(glXGetCurrentDisplay(),glXGetCurrentDrawable(),c.vsync?1:0);else std::cerr<<"Vsync no configurable con GLX_EXT_swap_control\n";
#endif
 camera();renderY=d->y;
 glutDisplayFunc(display);glutKeyboardFunc(keyboard);glutSpecialFunc(special);glutMouseFunc(mouse);glutMotionFunc(motion);glutReshapeFunc(resize);glutCreateMenu(action);glutAddMenuEntry("Guardar/detener MP4 [G]",'g');glutAddMenuEntry("HUD durante grabacion [B]",'b');glutAddMenuEntry("Ocultar/mostrar animacion [A]",'a');glutAddMenuEntry("Primera masa forzada [O]",'o');glutAddMenuEntry("Tiempo indefinido [I]",'i');glutAddMenuEntry("Cambiar borde [F]",'f');glutAddMenuEntry("Reiniciar [R]",'r');glutAddMenuEntry("Pausa/reanudar [Espacio]",' ');glutAddMenuEntry("Modo edicion [E]",'e');glutAddMenuEntry("HUD [H]",'h');glutAddMenuEntry("Salir",27);glutAttachMenu(GLUT_RIGHT_BUTTON);last=std::chrono::steady_clock::now();glutTimerFunc(0,timer,0);glutMainLoop();recorder.stop();d.reset();return 0;
 }catch(const std::exception&e){std::cerr<<"Error: "<<e.what()<<'\n';return 1;}}
