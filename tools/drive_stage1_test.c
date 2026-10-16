/* Synthetic gameplay adapter regressions; no retail assets. */
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "drive_stage1.h"
#ifdef NDEBUG
#error assertions required
#endif
static DrvCar car(float mass) {
    DrvCar c={0};c.mass=mass;c.gear_count=4;c.rear_share=.9f;
    c.final_front=c.final_rear=4;c.clutch_k=.05f;c.clutch_rate=2.5f;
    c.ratio[0]=-3;c.ratio[2]=3;c.ratio[3]=2;c.ratio[4]=1.4f;c.ratio[5]=1;
    c.idle_rpm=1000;c.shift_rpm=6000;c.max_rpm=7000;c.engine_inertia=.0003f;
    for(int k=0;k<4;k++)c.wheel_radius[k]=.32f;
    for(int k=0;k<2;k++){c.theta_max[k]=12;c.mu_peak[k]=1.3f;c.mu_slide[k]=1.1f;}
    for(int k=0;k<9;k++)c.torque_primary[k]=.3f;
    for(int k=0;k<3;k++)c.engine_brake[k]=.03f;
    assert(drv_car_finalize(&c)==0);return c;
}
typedef struct { Stage1Drive drive;float p[3],v[2],h,s;int tick; } Trace;
static void init(Trace *t,float mass){memset(t,0,sizeof *t);DrvCar c=car(mass);assert(stage1_drive_init(&t->drive,&c)==0);}
static void step(Trace *t,float pedal,int hb,unsigned contact){
    PhysRideState ride={.contact_mask=contact};
    stage1_drive_step(&t->drive,t->p,t->v,&t->h,&t->s,pedal,0,hb,NULL,NULL,&ride);
    assert(isfinite(t->s)&&isfinite(t->drive.drivetrain.omega_e));
    assert(t->drive.drivetrain.omega_e>=t->drive.drivetrain.car.idle_rpm*DRV_KW);
    ++t->tick;
}
/* Serialize invented tuning into the permitted independent parser's framing.
 * This checks offset mapping and validation without opening a game file. */
static void parser_test(void){
    unsigned char bytes[16+0x890]={0};
    uint32_t tag=0x34600,size=8+0x890;
    memcpy(bytes,&tag,4);memcpy(bytes+4,&size,4);memset(bytes+8,0x11,8);
    unsigned char *row=bytes+16;strcpy((char *)row,"SYNTH");
    strcpy((char *)row+0x40,"CARS\\SYNTH\\GEOMETRY.BIN");
    DrvCar c=car(1.2f),out;
#define PUT(field,offset) memcpy(row+(offset),&(field),sizeof(field))
    PUT(c.mass,0x220);
    for(int k=0;k<4;k++){
        PUT(c.wheel_radius[k],0x130+0x30*k);
        uint32_t index=k;PUT(index,0x138+0x30*k);
    }
    for(int a=0;a<2;a++){
        PUT(c.theta_max[a],0x1e0+0x20*a);
        PUT(c.mu_peak[a],0x1ec+0x20*a);PUT(c.mu_slide[a],0x1f0+0x20*a);
    }
    PUT(c.launch_fraction,0x2c0);PUT(c.final_front,0x2c8);PUT(c.final_rear,0x2cc);
    PUT(c.rear_share,0x2d0);PUT(c.clutch_k,0x2d4);
    uint32_t count=c.gear_count;PUT(count,0x2d8);PUT(c.clutch_rate,0x2dc);
    PUT(c.ratio,0x2e0);PUT(c.idle_rpm,0x300);PUT(c.shift_rpm,0x304);
    PUT(c.max_rpm,0x308);PUT(c.engine_inertia,0x30c);
    PUT(c.torque_primary,0x310);PUT(c.torque_secondary,0x334);PUT(c.engine_brake,0x358);
    assert(stage1_car_read(bytes,sizeof bytes,"SYNTH",&out));
    assert(!memcmp(&out,&c,sizeof c));
    assert(!stage1_car_read(bytes,sizeof bytes-1,"SYNTH",&out));
    assert(!stage1_car_read(bytes,sizeof bytes,"MISSING",&out));
    float bad=NAN;PUT(bad,0x30c);
    assert(!stage1_car_read(bytes,sizeof bytes,"SYNTH",&out));
    PUT(c.engine_inertia,0x30c);row[0x138]=3;
    assert(!stage1_car_read(bytes,sizeof bytes,"SYNTH",&out));
#undef PUT
}
static void tests(void){
    Trace a;init(&a,1.2f);
    for(int k=0;k<600;k++)step(&a,1,0,15);
    printf("launch 10 s: %.2f km/h gear %d rpm %.0f\n",PHYS_KMH(a.s),stage1_drive_gear(&a.drive),stage1_drive_rpm(&a.drive));
    assert(a.s>20/3.6f/PHYS_TICKRATE&&a.p[0]>20);
    assert(stage1_drive_gear(&a.drive)>=2);
    float launch=a.s;
    for(int k=0;k<120;k++)step(&a,0,0,15);
    assert(a.s<launch&&a.s>0);
    float coast=a.s;
    for(int k=0;k<60;k++)step(&a,-1,0,15);
    assert(a.s<coast);
    for(int k=0;k<600;k++)step(&a,-1,0,15);
    assert(a.s<-.02f&&stage1_drive_gear(&a.drive)==-1);
    for(int k=0;k<300;k++)step(&a,1,0,15);
    assert(a.s>0&&stage1_drive_gear(&a.drive)>0);
    for(int k=0;k<600;k++)step(&a,1,1,15);
    assert(a.s==0&&a.v[0]==0&&a.v[1]==0);
    stage1_drive_reset(&a.drive,a.v,a.h);
    assert(stage1_drive_gear(&a.drive)==0&&a.drive.drivetrain.feedback==0);
    assert(a.drive.drivetrain.engage==0&&a.drive.drivetrain.wheel[0].omega==0);
    Trace air;init(&air,1.2f);
    for(int k=0;k<120;k++)step(&air,1,0,0);
    assert(air.s==0&&air.p[0]==0);
    /* A paused/zero-tick frame never calls the adapter: all state remains exact. */
    Stage1Drive snapshot=a.drive;PhysClock clock={0};assert(phys_clock_steps(&clock,0)==0);
    assert(!memcmp(&snapshot,&a.drive,sizeof snapshot));
    /* A fresh car replaces every engine/wheel/clutch state. */
    DrvCar c=car(1.8f);assert(stage1_drive_init(&a.drive,&c)==0);
    assert(a.drive.drivetrain.car.mass==1.8f&&stage1_drive_gear(&a.drive)==0);
    /* No engine torque means no hidden arcade propulsion. */
    for(int k=0;k<9;k++)c.torque_primary[k]=0;
    c.clutch_k=0; /* disconnect the idle-clamped rotating engine as well */
    assert(stage1_drive_init(&a.drive,&c)==0);memset(a.v,0,sizeof a.v);a.s=0;
    for(int k=0;k<300;k++)step(&a,1,0,15);
    assert(a.s==0);
    /* Every axle class uses actual shaft torque, and steering stays live. */
    const float shares[]={0,.5f,1};
    for(unsigned i=0;i<sizeof shares/sizeof *shares;i++){
        Trace t;init(&t,1.2f);c=car(1.2f);c.rear_share=shares[i];
        assert(stage1_drive_init(&t.drive,&c)==0);
        for(int k=0;k<600;k++)step(&t,1,0,15);
        assert(t.s>.05f&&stage1_drive_gear(&t.drive)>=2);
        float oldh=t.h;PhysRideState ride={.contact_mask=15};
        stage1_drive_step(&t.drive,t.p,t.v,&t.h,&t.s,1,.5f,0,NULL,NULL,&ride);
        assert(t.h>oldh);
        /* Collision impulses enter through changed host velocity; no teleport
           or stale velocity copy restores the previous body motion. */
        t.v[0]=t.v[1]=0;t.s=0;
        step(&t,0,1,15);assert(t.s==0);
        t.v[0]=.2f;t.v[1]=.1f;
        float airh=t.h;
        for(int k=0;k<30;k++)step(&t,1,0,0);
        assert(t.h==airh&&t.v[1]==.1f);
    }
    Trace reference;init(&reference,1.2f);
    for(int k=0;k<600;k++)step(&reference,k<420?1:0,0,15);
    int rates[]={30,60,90,144,240};
    for(unsigned r=0;r<sizeof rates/sizeof *rates;r++){
        Trace t;init(&t,1.2f);clock=(PhysClock){0};
        for(int frame=0;frame<10*rates[r];frame++){
            int n=phys_clock_steps(&clock,1.0/rates[r]);
            for(int k=0;k<n;k++)step(&t,t.tick<420?1:0,0,15);
        }
        assert(t.tick==reference.tick&&t.s==reference.s&&t.p[0]==reference.p[0]);
        assert(!memcmp(&t.drive,&reference.drive,sizeof t.drive));
    }
    /* Refuse missing/invalid data instead of substituting invented car tuning. */
    DrvCar out;assert(!stage1_car_read(NULL,0,"SYNTH",&out));
    c=car(1.2f);c.ratio[2]=NAN;assert(stage1_drive_init(&a.drive,&c)<0);
    /* Finite samples whose binary32 sum overflows are rejected at entry, and a
       rejected init leaves a previously valid state untouched. */
    Trace good;init(&good,1.2f);
    for(int k=0;k<60;k++)step(&good,1,0,15);
    Stage1Drive saved=good.drive;
    c=car(1.2f);
    for(int k=0;k<9;k++)c.torque_primary[k]=c.torque_secondary[k]=FLT_MAX;
    assert(drv_car_finalize(&c)<0);
    assert(stage1_drive_init(&good.drive,&c)<0);
    assert(!memcmp(&good.drive,&saved,sizeof saved));
    Drivetrain dt;DrvCar ok=car(1.2f);assert(drv_init(&dt,&ok,NULL)==0);
    Drivetrain dsaved=dt;DrvCar bad=ok;
    bad.torque_primary[4]=bad.torque_secondary[4]=FLT_MAX;
    assert(drv_init(&dt,&bad,NULL)<0&&!memcmp(&dt,&dsaved,sizeof dt));
    DrvCar before=bad;assert(drv_car_finalize(&bad)<0&&!memcmp(&bad,&before,sizeof bad));
    /* Non-finite curve coordinates never reach an integer conversion. */
    float tq[DRV_TORQUE_SAMPLES]={1,2,3,4,5,6,7,8,9};
    assert(drv_curve(tq,9,0,1,NAN)==1&&drv_curve(tq,9,0,1,INFINITY)==9);
    assert(drv_curve(tq,9,0,1,-INFINITY)==1&&drv_curve(tq,9,0,0,0)==1);
}
int main(void){parser_test();tests();phys_selftest();puts("Stage 1 gameplay adapter tests passed");return 0;}
