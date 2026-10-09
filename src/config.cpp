#include "simulation.hpp"
#include <sstream>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <stdexcept>
std::string readFile(const std::string&p){std::ifstream f(p); if(!f)throw std::runtime_error("No se puede abrir "+p); return {std::istreambuf_iterator<char>(f),{}};}
Config Config::load(const std::string&p){
 Config c; std::istringstream in(readFile(p)); std::string line; int ln=0; std::map<std::string,std::string>a;
 while(std::getline(in,line)){++ln; line=line.substr(0,line.find('#')); if(line.find_first_not_of(" \t\r")==std::string::npos)continue; auto eq=line.find('='); if(eq==std::string::npos)throw std::runtime_error("Falta = en linea "+std::to_string(ln));
 auto trim=[](std::string s){auto b=s.find_first_not_of(" \t\r\"'"); if(b==s.npos)return std::string(); return s.substr(b,s.find_last_not_of(" \t\r\"'")-b+1);};
 auto key=trim(line.substr(0,eq)); if(a.count(key))throw std::runtime_error("Parametro duplicado: "+key); a[key]=trim(line.substr(eq+1)); }
 auto get=[&](const char*n){auto s=a.at(n);a.erase(n);return s;};
 auto num=[&](const char*n,double&v){if(a.count(n)){auto s=get(n);size_t t;v=std::stod(s,&t);if(t!=s.size()||!std::isfinite(v))throw std::runtime_error(std::string("Numero invalido: ")+n);}};
 auto integer=[&](const char*n,auto&v){double d=double(v);num(n,d); if(d!=std::floor(d)||d<0||d>2147483647)throw std::runtime_error(std::string("Entero invalido: ")+n);v=static_cast<std::decay_t<decltype(v)>>(d);};
 auto boolean=[&](const char*n,bool&v){if(a.count(n)){auto s=get(n);if(s=="true"||s=="True")v=true;else if(s=="false"||s=="False")v=false;else throw std::runtime_error(std::string("Bool invalido: ")+n);}};
 auto array=[&](const char*n,std::vector<float>&v){if(a.count(n)){auto s=get(n); if(s.size()<2||s.front()!='['||s.back()!=']')throw std::runtime_error("Se esperaba lista []");s=s.substr(1,s.size()-2);std::replace(s.begin(),s.end(),',',' ');std::istringstream ss(s);v.clear();double x;while(ss>>x){if(!std::isfinite(x))throw std::runtime_error("Lista no finita");v.push_back(float(x));}if(!ss.eof())throw std::runtime_error("Lista invalida");}};
 integer("N",c.N);integer("ancho",c.width);integer("alto",c.height);integer("max_pasos_frame",c.max_steps);integer("dispositivo",c.device);integer("modo",c.mode);integer("fps",c.fps);integer("n_pasos",c.steps);
 num("escala_y",c.escala_y);num("amplitud_fuerza",c.force_amplitude);num("frecuencia_fuerza",c.force_frequency);
 num("m",c.m);num("k",c.k);num("k1",c.k1);num("dt",c.dt);num("L0",c.L0);num("L0_acople",c.L0_acople);num("x0",c.x0);num("y_eq",c.y_eq);num("amplitud",c.amplitude);num("escala",c.scale);num("radio",c.radius);num("tiempo_total",c.total);
 boolean("primer_oscilador_forzado",c.forced);
 boolean("extremo_superior_fijo",c.first);boolean("extremo_inferior_fijo",c.last);boolean("mostrar_HUD",c.hud);boolean("editar",c.edit);boolean("soltar_velocidad_cero",c.release_zero);boolean("vsync",c.vsync);boolean("HUD_grabacion",c.record_hud);
 if(a.count("patron"))c.pattern=get("patron");
 if(a.count("directorio_video"))c.output=get("directorio_video");
 array("y_inicial",c.y);array("v_inicial",c.v);array("color_fondo",c.bg);array("color_masa",c.mass);
 if(!a.empty())throw std::runtime_error("Parametro desconocido: "+a.begin()->first);
 if(c.escala_y<.01||c.escala_y>1000)throw std::runtime_error("escala_y debe estar en [0.01,1000]");
 if(c.force_amplitude<0||c.force_frequency<=0)throw std::runtime_error("Fuerza: amplitud >=0 y frecuencia >0 requeridas");
 if(c.forced&&c.first)throw std::runtime_error("El primer oscilador forzado debe estar libre");
 if(c.N<1||c.N>1000000||c.m<=0||c.k<0||c.k1<0||c.dt<=0||c.L0<=0||c.L0_acople<=0||c.x0<=0||c.radius<=0||c.scale<=0||c.total<0||c.width<320||c.height<240||c.width%2||c.height%2||c.max_steps<1||c.fps<1||c.fps>240)throw std::runtime_error("Parametros fuera de rango (video requiere dimensiones pares)");
 for(auto*v:{&c.y,&c.v})if(!v->empty()&&v->size()!=size_t(c.N))throw std::runtime_error("y_inicial/v_inicial deben tener N elementos");
 for(auto*v:{&c.bg,&c.mass}){if(v->size()!=3)throw std::runtime_error("Color debe tener 3 elementos");for(float x:*v)if(x<0||x>1)throw std::runtime_error("Color fuera de [0,1]");}
 if(c.pattern!="cero"&&c.pattern!="pulso_extremo"&&c.pattern!="modo_normal"&&c.pattern!="arbitrario")throw std::runtime_error("Patron desconocido");
 if(c.pattern=="arbitrario"&&c.y.empty())throw std::runtime_error("arbitrario requiere y_inicial");
 if(c.pattern=="modo_normal"&&(c.first||c.last)&&c.y_eq!=0)throw std::runtime_error("Modo normal con extremos fijos requiere y_eq=0");
 int free=c.N-(c.N==1?int(c.first||c.last):int(c.first)+int(c.last));if(c.pattern=="modo_normal"&&(free<1||c.mode<1||c.mode>free))throw std::runtime_error("Modo fuera de rango");
 double omega=std::sqrt((c.k1+4*c.k)/c.m);if(omega>0&&c.dt>=2/omega)std::cerr<<"ADVERTENCIA: dt >= 2/omega_max; Verlet puede ser inestable\n";
 return c;
}
std::vector<float> initial(const Config&c){std::vector<float> y(c.N,float(c.y_eq));if(!c.y.empty())y=c.y;else if(c.pattern=="pulso_extremo"){int i=c.first?1:0;if(i<c.N&&!(i==c.N-1&&c.last))y[i]+=c.amplitude;}else if(c.pattern=="modo_normal"){
 int start=c.first?1:0,end=c.N-(c.last?1:0),M=end-start;
 for(int j=0;j<M;j++){double phase=0;
 if(c.first&&c.last)phase=3.141592653589793*c.mode*(j+1)/(M+1);
 else if(!c.first&&!c.last){y[start+j]+=c.amplitude*std::cos(3.141592653589793*(c.mode-1)*(j+.5)/M);continue;}
 else {int q=c.first?M-1-j:j;phase=3.141592653589793*(c.mode-.5)*(q+.5)/(M+.5);y[start+j]+=c.amplitude*std::cos(phase);continue;}
 y[start+j]+=c.amplitude*std::sin(phase);}}
 if(c.first)y[0]=0;
 if(c.last)y.back()=0;
 return y;}
std::vector<float> acceleration(const Config&c,const std::vector<float>&y){std::vector<float>a(c.N);for(int i=0;i<c.N;i++){if((i==0&&c.first)||(i==c.N-1&&c.last))continue;double f=-c.k1*(y[i]-c.y_eq);if(i>0)f-=c.k*(y[i]-y[i-1]);if(i+1<c.N)f-=c.k*(y[i]-y[i+1]);a[i]=f/c.m;}return a;}
double energy(const Config&c,const std::vector<float>&y,const std::vector<float>&v,double*T){double t=0,u=0;for(int i=0;i<c.N;i++){t+=.5*c.m*v[i]*v[i];u+=.5*c.k1*std::pow(y[i]-c.y_eq,2);if(i)u+=.5*c.k*std::pow(y[i]-y[i-1],2);}if(T)*T=t;return t+u;}
