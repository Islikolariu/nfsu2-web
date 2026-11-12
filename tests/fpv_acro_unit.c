#include "fpv_flight.h"
#include "fpv_camera.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void step(fpv_flight *s,fpv_controls in,int count,float dt){for(int i=0;i<count;i++)fpv_acro_step(s,&in,dt);}
static void chase(fpv_flight *s){
    const float o[3]={100,200,15};fpv_acro_reset(s,o,0);
    fpv_acro_set_mass(s,750); /* approved reference translation model */
    s->q[0]=cosf(.5235988f);s->q[2]=sinf(.5235988f);s->motor=sqrtf(1.f/3.f);
}
int main(void){
    fpv_flight s,t;const float o[3]={100,200,15};float f[3],u[3];fpv_controls in={0};
    fpv_acro_reset(&s,o,0);in.lift=-1;step(&s,in,120,1.f/120);
    assert(s.p[2]<14&&s.v[2]<-8); /* raw zero throttle falls: no height hold */
    fpv_acro_reset(&s,o,0);in.roll=.2f;step(&s,in,36,1.f/120);in.roll=0;
    step(&s,in,120,1.f/120);float bank=s.roll;step(&s,in,240,1.f/120);
    assert(fabsf(bank)>.2f&&fabsf(s.roll-bank)<.001f); /* neutral holds bank, no auto-level */
    for(int axis=0;axis<2;axis++){
        fpv_acro_reset(&s,o,0);in=(fpv_controls){0,0,0,-1};
        if(axis==0)in.roll=1;else in.pitch=1;
        int inverted=0,full_turn=0;
        for(int i=0;i<240;i++){
            fpv_acro_step(&s,&in,1.f/240);fpv_flight_basis(&s,f,u);
            if(u[2]<-.95f)inverted=1;
            if(inverted&&u[2]>.99f)full_turn=1;
        }
        assert(inverted&&full_turn); /* bank/flip really passes 180 and 360 degrees */
        in.roll=in.pitch=0;step(&s,in,120,1.f/240);float held_q[4];
        for(int j=0;j<4;j++)held_q[j]=s.q[j];
        step(&s,in,480,1.f/240);
        float qdot=0;for(int j=0;j<4;j++)qdot+=held_q[j]*s.q[j];
        assert(fabsf(qdot)>.99999f); /* releasing either axis cannot level attitude */
    }
    fpv_acro_reset(&s,o,0);s.q[0]=0;s.q[1]=1;in.lift=0;step(&s,in,24,1.f/120);
    fpv_flight_basis(&s,f,u);assert(u[2]<-.99f&&s.v[2]<-2); /* inverted thrust, no Euler clamp */
    chase(&s);chase(&t);in=(fpv_controls){0,0,0,2*sqrtf(1.f/3.f)-1};
    step(&s,in,2400,1.f/120);step(&t,in,600,1.f/30);
    assert(s.p[0]-o[0]>500&&s.v[0]>40&&!s.limited);
    assert(fabsf(s.p[0]-t.p[0])<.2f&&fabsf(s.p[2]-t.p[2])<.2f);
    printf("Chase model: %.1f m from spawn, %.1f km/h, 30/120 Hz displacement difference %.4f m\n",s.p[0]-o[0],s.v[0]*3.6f,fabsf(s.p[0]-t.p[0]));
    in=(fpv_controls){.8f,-.7f,.4f,.2f};step(&s,in,30000,1.f/120);
    float norm=0;for(int i=0;i<4;i++)norm+=s.q[i]*s.q[i];assert(fabsf(norm-1)<.00001f);
    fpv_flight_basis(&s,f,u);float dot=0,fn=0,un=0;
    for(int i=0;i<3;i++){assert(isfinite(s.p[i])&&isfinite(s.v[i]));dot+=f[i]*u[i];fn+=f[i]*f[i];un+=u[i]*u[i];}
    assert(fabsf(dot)<.00001f&&fabsf(fn-1)<.00001f&&fabsf(un-1)<.00001f);
    fpv_acro_reset(&s,o,0);in=(fpv_controls){NAN,INFINITY,NAN,NAN};step(&s,in,120,1.f/120);
    assert(isfinite(s.p[0])&&isfinite(s.p[2]));
    /* Fixed motor force and drag, varying mass: gravity is never scaled.
     * A modest added load at the same low throttle descends faster, keeps
     * body attitude and velocity, and can still climb at sufficient throttle. */
    fpv_acro_reset(&s,o,0);fpv_acro_reset(&t,o,0);fpv_acro_set_mass(&s,750);fpv_acro_set_mass(&t,850);
    float hover=sqrtf(1.f/6.f);s.motor=t.motor=hover;in=(fpv_controls){0,0,0,2*hover-1};
    step(&s,in,240,1.f/120);step(&t,in,240,1.f/120);
    assert(fabsf(s.v[2])<.001f&&t.v[2]<-1.5f&&t.p[2]<s.p[2]-1.5f);
    printf("Mass test: same low throttle, 750 g vz=%.3f m/s, 850 g vz=%.3f m/s after 2s\n",s.v[2],t.v[2]);
    fpv_acro_reset(&s,o,0);fpv_acro_reset(&t,o,0);fpv_acro_set_mass(&s,300);fpv_acro_set_mass(&t,1500);
    in=(fpv_controls){0,0,0,-1};step(&s,in,1,1.f/240);step(&t,in,1,1.f/240);
    assert(fabsf(s.v[2]-t.v[2])<1e-6f&&fabsf(s.v[2]+9.81f/240)<1e-6f);
    in=(fpv_controls){.4f,.2f,.1f,-1};step(&s,in,120,1.f/120);step(&t,in,120,1.f/120);
    assert(t.v[2]<s.v[2]);for(int i=0;i<4;i++)assert(fabsf(s.q[i]-t.q[i])<1e-6f);
    fpv_acro_set_mass(&s,9999);assert(s.mass_kg==1.5f);fpv_acro_set_mass(&s,NAN);assert(s.mass_kg==1.5f);
    fpv_acro_reset(&s,o,0);assert(fabsf(s.mass_kg-.85f)<1e-6f);
    float look[3], yaw=0;
    assert(fabsf(fpv_camera_view(&s,look,&yaw))<1e-5f&&look[2]>s.p[2]+7.f);
    s.q[0]=0;s.q[1]=1;
    assert(fabsf(fpv_camera_view(&s,look,&yaw))>3.f&&look[2]<s.p[2]-7.f);
    s.p[0]=o[0]+110.f;s.p[2]=o[2]-2.f;s.v[0]=8.f;s.v[2]=-5.f;
    fpv_camera_limit(&s,o);
    assert(fabsf(s.p[0]-o[0]-100.f)<1e-4f&&s.p[2]==o[2]);
    assert(fabsf(s.v[0])<1e-5f&&s.v[2]==0.f);
    puts("PASS: raw throttle/gravity, no auto-level after roll AND pitch release, full 360-degree rolls/flips, inverted flight, no car tether, chase speed, timestep stability and quaternion basis");
    return 0;
}
