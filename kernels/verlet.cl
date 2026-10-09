// Two launches provide a GLOBAL boundary between drift and the second kick.
float accel(__global const float *y,int i,int n,float m,float k,float k1,float eq,float force){
 float f=-k1*(y[i]-eq)+(i==0?force:0.0f);if(i>0)f-=k*(y[i]-y[i-1]);if(i+1<n)f-=k*(y[i]-y[i+1]);return f/m;
}
__kernel void drift(__global const float*y,__global const float*v,__global float*z,__global float*h,int n,float m,float k,float k1,float eq,float dt,int first,int last,int drag,float target,float speed,float force){
 int i=get_global_id(0);if(i>=n)return;
 if((i==0&&first)||(i==n-1&&last)){z[i]=0;h[i]=0;return;}
 if(i==drag){z[i]=target;h[i]=speed;return;}
 h[i]=v[i]+.5f*dt*accel(y,i,n,m,k,k1,eq,force);z[i]=y[i]+dt*h[i];
}
__kernel void kick(__global const float*z,__global const float*h,__global float*v,int n,float m,float k,float k1,float eq,float dt,int first,int last,int drag,float speed,float force){
 int i=get_global_id(0);if(i>=n)return;
 if((i==0&&first)||(i==n-1&&last)){v[i]=0;return;}
 v[i]=i==drag?speed:h[i]+.5f*dt*accel(z,i,n,m,k,k1,eq,force);
}
