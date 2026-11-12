#include "fpv_flight.h"
#include <math.h>
#include <string.h>
static float clamp(float x,float a,float b){return x<a?a:x>b?b:x;}
void fpv_flight_reset(fpv_flight *s,const float origin[3],float yaw){
    memset(s,0,sizeof *s); memcpy(s->origin,origin,12); memcpy(s->p,origin,12);
    s->p[2]+=2.5f; s->yaw=yaw;
}
void fpv_flight_basis(const fpv_flight *s,float f[3],float u[3]){
    if(s->acro){
        float w=s->q[0],x=s->q[1],y=s->q[2],z=s->q[3];
        f[0]=1-2*(y*y+z*z);f[1]=2*(x*y+w*z);f[2]=2*(x*z-w*y);
        u[0]=2*(x*z+w*y);u[1]=2*(y*z-w*x);u[2]=1-2*(x*x+y*y);return;
    }
    float cy=cosf(s->yaw),sy=sinf(s->yaw),cp=cosf(s->pitch),sp=sinf(s->pitch);
    float cr=cosf(s->roll),sr=sinf(s->roll);
    f[0]=cy*cp; f[1]=sy*cp; f[2]=-sp;
    /* Positive roll banks right. Pitch positive looks down / accelerates forward. */
    u[0]=cy*sp*cr+sy*sr; u[1]=sy*sp*cr-cy*sr; u[2]=cp*cr;
}
void fpv_acro_reset(fpv_flight *s,const float origin[3],float yaw){
    fpv_flight_reset(s,origin,yaw);s->acro=1;s->q[0]=cosf(yaw*.5f);s->q[3]=sinf(yaw*.5f);
    s->mass_kg=.85f;
}
void fpv_acro_set_mass(fpv_flight *s,float grams){
    if(isfinite(grams))s->mass_kg=clamp(grams,300.f,1500.f)/1000.f;
}
static float stick(float x){return isfinite(x)?clamp(x,-1,1):0;}
static float rate(float x,float maximum){x=stick(x);return (.65f*x+.35f*x*x*x)*maximum;}
void fpv_acro_step(fpv_flight *s,const fpv_controls *in,float dt){
    if(!s->acro||!isfinite(dt)||dt<=0)return;
    dt=clamp(dt,0,.1f);int n=(int)ceilf(dt*240.f);float h=dt/n;
    float desired[3]={rate(in->roll,7.330383f),rate(in->pitch,7.330383f),rate(in->yaw,5.235988f)};
    float throttle=isfinite(in->lift)?(stick(in->lift)+1)*.5f:0;
    float mass=isfinite(s->mass_kg)&&s->mass_kg>=.3f&&s->mass_kg<=1.5f?s->mass_kg:.85f;
    s->limited=0;
    for(int k=0;k<n;k++){
        float response=1-expf(-h/0.03f),motor_response=1-expf(-h/.04f);
        for(int i=0;i<3;i++)s->omega[i]+=(desired[i]-s->omega[i])*response;
        s->motor+=(throttle-s->motor)*motor_response;
        float w=s->q[0],x=s->q[1],y=s->q[2],z=s->q[3];
        float a=s->omega[0]*h*.5f,b=s->omega[1]*h*.5f,c=s->omega[2]*h*.5f;
        s->q[0]=w-x*a-y*b-z*c;s->q[1]=x+w*a+y*c-z*b;
        s->q[2]=y+w*b+z*a-x*c;s->q[3]=z+w*c+x*b-y*a;
        float norm=sqrtf(s->q[0]*s->q[0]+s->q[1]*s->q[1]+s->q[2]*s->q[2]+s->q[3]*s->q[3]);
        for(int i=0;i<4;i++)s->q[i]/=norm;
        float f[3],u[3];fpv_flight_basis(s,f,u);
        float speed=sqrtf(s->v[0]*s->v[0]+s->v[1]*s->v[1]+s->v[2]*s->v[2]);
        /* Fixed motor/propeller force and air resistance, divided by mass.
         * 750 g exactly reproduces the approved translation model. 850 g
         * adds a modest load: less thrust per kg and less drag deceleration.
         * Gravity stays 9.81 m/s^2 for every mass; rates remain body rates. */
        float thrust=(.75f*6.f*9.81f/mass)*s->motor*s->motor;
        float drag=(.75f/mass)*(.12f+.0065f*speed);
        for(int i=0;i<3;i++){
            s->v[i]+=(u[i]*thrust-drag*s->v[i]-(i==2?9.81f:0))*h;
            s->p[i]+=s->v[i]*h;
        }
    }
    float f[3],u[3];fpv_flight_basis(s,f,u);
    s->yaw=atan2f(f[1],f[0]);s->pitch=asinf(clamp(-f[2],-1,1));
    s->roll=atan2f(2*(s->q[0]*s->q[1]+s->q[2]*s->q[3]),1-2*(s->q[1]*s->q[1]+s->q[2]*s->q[2]));
    /* Deliberately no distance or height clamp relative to the player's car.
     * The host supplies map streaming and a separate scenery collision query.
     * This is a rate/force model, not Betaflight or a calibrated airframe. */
}
void fpv_flight_step(fpv_flight *s,const fpv_controls *in,float dt){
    int n,i; float h;
    if(!isfinite(dt)||dt<=0)return;
    dt=clamp(dt,0,.05f); n=(int)ceilf(dt*120.f); h=dt/(float)n;
    s->limited=0;
    for(i=0;i<n;i++){
        float f[3],u[3],thrust,vz,k=1.f-expf(-h*8.f),dist;
        s->pitch+=(clamp(in->pitch,-1,1)*.55f-s->pitch)*k;
        s->roll+=(clamp(in->roll,-1,1)*.55f-s->roll)*k;
        s->yaw+=clamp(in->yaw,-1,1)*2.4f*h;
        s->yaw=remainderf(s->yaw,6.283185307f);
        fpv_flight_basis(s,f,u);
        /* Centred lift stick commands zero vertical speed, assisted hover. */
        vz=clamp(in->lift,-1,1)*6.f;
        thrust=clamp((9.81f+(vz-s->v[2])*4.f)/fmaxf(u[2],.4f),0,30);
        s->v[0]+=(u[0]*thrust-.9f*s->v[0])*h;
        s->v[1]+=(u[1]*thrust-.9f*s->v[1])*h;
        s->v[2]+=(u[2]*thrust-9.81f)*h;
        for(int j=0;j<3;j++){s->v[j]=clamp(s->v[j],-14,14);s->p[j]+=s->v[j]*h;}
        dist=hypotf(s->p[0]-s->origin[0],s->p[1]-s->origin[1]);
        if(dist>70){
            float nx=(s->p[0]-s->origin[0])/dist,ny=(s->p[1]-s->origin[1])/dist;
            float outward=s->v[0]*nx+s->v[1]*ny;
            s->p[0]=s->origin[0]+nx*70;s->p[1]=s->origin[1]+ny*70;
            if(outward>0){s->v[0]-=nx*outward;s->v[1]-=ny*outward;}s->limited=1;
        }
        if(s->p[2]<s->origin[2]+.8f){s->p[2]=s->origin[2]+.8f;s->v[2]=fmaxf(0,s->v[2]);s->limited=1;}
        if(s->p[2]>s->origin[2]+35){s->p[2]=s->origin[2]+35;s->v[2]=fminf(0,s->v[2]);s->limited=1;}
    }
}
