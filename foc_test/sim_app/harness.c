/* PC end-to-end test of foc_app.c (+foc_voltage.c) against a PMSM dq model */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "foc_app.h"
#include "foc_config.h"
#include "foc_board.h"
#include "foc_encoder_mt6701.h"

/* ---------- model ---------- */
#define PP 11
static const double R=13.0, L=0.5e-3, LAM=0.005, J=2e-5, B=1e-5, VB=12.0;
static double Tcog=0.002, Fcoul=0.001, enc_off=2.3; static int enc_sign=-1;
static double th_m=1.234, w=0, id=0, iq=0, load=0;
static double da=.5,db=.5,dc=.5; static int en=0; static const int CPR=16384;
static double t=0;
#define DT 5e-5
#define KE_TRUE (LAM*PP)   /* Uq per mech rad/s (amplitude-invariant), same as sim.c old scenario */

static void step(double dt){
  double va=(da-.5)*VB, vb=(db-.5)*VB, vc=(dc-.5)*VB; if(!en) va=vb=vc=0;
  double cm=(va+vb+vc)/3; va-=cm; vb-=cm; vc-=cm;
  double al=(2.0/3)*(va-0.5*vb-0.5*vc), be=(2.0/3)*(0.8660254*(vb-vc));
  double te=th_m*PP, c=cos(te), s=sin(te);
  double vd=al*c+be*s, vq=-al*s+be*c, we=w*PP;
  for(int k=0;k<10;k++){ double h=dt/10;
    double did=(vd-R*id+we*L*iq)/L, diq=(vq-R*iq-we*L*id-we*LAM)/L;
    id+=did*h; iq+=diq*h;
    double T=1.5*PP*LAM*iq - B*w - load - Tcog*sin(6*PP*th_m) - Fcoul*tanh(w/0.05);
    w+=T/J*h; th_m+=w*h; }
}

/* ---------- stubs ---------- */
mt6701f_t g_enc;
int inject_bad_reads=0;
volatile uint32_t g_isr_cycles_last, g_isr_cycles_max, g_isr_count;
static void (*isr_cb)(void)=0; static int isr_on=0;
static void tick(void);
void FocBoard_Init(void){ en=0; da=db=dc=.5; isr_on=0; }
void FocBoard_StartIsr(void(*cb)(void)){ isr_cb=cb; isr_on=1; }
void FocBoard_StopIsr(void){ isr_on=0; }
void hal_pwm_set(float a,float b,float c){da=a;db=b;dc=c;}
void hal_pwm_enable(int on){en=on;}
uint32_t HAL_GetTick(void){ return (uint32_t)(t*1000.0+1e-6); }
static int g_stub_enc_ok=1;
bool FocEnc_Init(void){ return g_stub_enc_ok; }
void MT6701F_ClearFault(mt6701f_t *e){ e->fault=false; e->consecutive_err=0; }
static uint32_t model_raw(void){ double a=fmod(enc_sign*th_m+enc_off,2*M_PI); if(a<0)a+=2*M_PI; return (uint32_t)(a/(2*M_PI)*CPR)%CPR; }
uint32_t hal_encoder_raw(void){
  if(inject_bad_reads>0){ inject_bad_reads--; if(++g_enc.consecutive_err>=3) g_enc.fault=true; return g_enc.last_raw; }
  g_enc.consecutive_err=0; g_enc.last_raw=(uint16_t)model_raw(); return g_enc.last_raw;
}
static void one_step(void){ if(isr_on) isr_cb(); step(DT); t+=DT; tick(); }
void hal_delay_ms(uint32_t ms){ for(uint32_t i=0;i<ms*20;i++) one_step(); }

/* ---------- monitors ---------- */
static double max_i=0, max_dth=0, th_ref=0, max_uq_win=0; static int win_steps=0;
static void tick(void){
  double im=sqrt(id*id+iq*iq); if(im>max_i) max_i=im;
  double d=fabs(th_m-th_ref); if(d>max_dth) max_dth=d;
  if(win_steps>0){ double u=fabs(g_foc.uq); if(u>max_uq_win) max_uq_win=u; win_steps--; }
}
static int step_ctr=0;
static void run(double sec){
  int n=(int)(sec/DT+0.5);
  for(int i=0;i<n;i++){ one_step(); if(++step_ctr%20==0) FocApp_Loop(); }
}
static int fails=0;
static void check(int ok,const char*fmt,...) __attribute__((format(printf,2,3)));
#include <stdarg.h>
static void check(int ok,const char*fmt,...){
  printf("%s  ",ok?"PASS":"FAIL"); va_list a; va_start(a,fmt); vprintf(fmt,a); va_end(a); printf("\n"); if(!ok) fails++;
}
static const char* stn(int s){ static const char*n[]={"INIT","ENC_FAIL","PP_FAIL","ALIGN_FAIL","RUNNING","FAULT","MEASURING_KE","STOPPED"}; return n[s]; }

int main(int argc,char**argv){
  if(argc>1) enc_sign=atoi(argv[1]);
  if(argc>2) enc_off=atof(argv[2]);
  printf("model: PP=%d R=%.1f L=%.1fmH LAM=%.4f Ke_true=%.4f Vs/rad J=%.1e enc_sign=%+d off=%.2f\n",PP,R,L*1e3,LAM,KE_TRUE,J,enc_sign,enc_off);
  printf("Kt=%.4f Nm/A, torque@I_LIMIT=%.4f Nm\n",1.5*PP*LAM,1.5*PP*LAM*FOC_I_LIMIT);

  /* 0 init failure: commands must not drive the motor */
  printf("\n--- 0 Init failure ---\n");
  g_stub_enc_ok=0; FocApp_Init();
  check(g_app_state==FOC_APP_ENC_FAIL,"state ENC_FAIL after encoder init failure (got %s)",stn(g_app_state));
  g_cmd.stop=1; FocApp_Loop(); g_cmd.mode=3; g_cmd.target=1.0f; g_cmd.apply=1; FocApp_Loop();
  g_cmd.clear_fault=1; FocApp_Loop(); g_cmd.measure_ke=1; FocApp_Loop();
  check(g_app_state==FOC_APP_ENC_FAIL && !isr_on,"stop/apply/clear/measure ignored: state=%s isr_on=%d",stn(g_app_state),isr_on);
  g_stub_enc_ok=1;

  /* 1 boot */
  printf("\n--- 1 Boot ---\n");
  FocApp_Init();
  printf("boot time %.2f s (sim), state=%s pp=%d dir=%d elec_off=%.3f pp_check=%.3f\n",t,stn(g_app_state),g_detected_pp,g_foc.dir,g_foc.elec_offset,g_foc.pp_check);
  check(g_app_state==FOC_APP_RUNNING && g_detected_pp==PP,"state RUNNING, pp==%d (got %d)",PP,g_detected_pp);
  check(g_foc.dir==enc_sign,"dir=%d expected %d",g_foc.dir,enc_sign);
  th_ref=th_m; max_dth=0; run(0.5);
  check(max_dth<0.01,"max |pos-pos_start| over 0.5 s = %.5f rad (<0.01), end vel=%.3f",max_dth,w);

  /* 2 position step */
  printf("\n--- 2 Position step +1 rad ---\n");
  double tgt=g_foc.pos+1.0; g_cmd.mode=3; g_cmd.target=(float)tgt; g_cmd.apply=1; max_i=0; run(2.0);
  double err=tgt-g_foc.pos;
  check(fabs(err)<0.005,"|err|=%.5f rad (<0.005), vel=%.3f",fabs(err),g_foc.vel);
  check(max_i<FOC_I_LIMIT*1.2,"max phase current %.3f A (limit %.2f, +20%% = %.3f); iq_ss=%.3f",max_i,FOC_I_LIMIT,FOC_I_LIMIT*1.2,iq);

  /* 3 disturbance */
  printf("\n--- 3 Load disturbance ---\n");
  double tl=0.5*1.5*PP*LAM*FOC_I_LIMIT; load=tl; double e0=0,emax=0;
  for(int k=0;k<1000;k++){ run(0.001); double e=fabs(g_foc.target-g_foc.pos); if(e>emax)emax=e; }
  e0=fabs(g_foc.target-g_foc.pos);
  printf("load=%.4f Nm; peak err %.4f rad, err after 1 s %.5f rad, iq=%.3f, uq=%.3f\n",tl,emax,e0,iq,g_foc.uq);
  check(e0<0.01,"steady error under load after 1 s: %.5f rad (<0.01)",e0);
  run(2.0); printf("  info: err after further 2 s (3 s total) = %.5f rad\n",fabs(g_foc.target-g_foc.pos));

  /* 4 Ke */
  printf("\n--- 4 Ke measurement ---\n");
  load=0; g_cmd.measure_ke=1; run(0.002); int to=0;
  check(g_app_state==FOC_APP_MEASURING_KE,"entered MEASURING_KE (state=%s)",stn(g_app_state));
  double t0=t; while(g_app_state!=FOC_APP_RUNNING && t-t0<5.0) run(0.001); to=(g_app_state!=FOC_APP_RUNNING);
  check(!to,"returned to RUNNING after %.2f s",t-t0);
  double ke_err=(g_measured_ke-KE_TRUE)/KE_TRUE*100;
  printf("measured %.5f true %.5f (%.1f%%); g_foc.ke=%.5f\n",g_measured_ke,KE_TRUE,ke_err,g_foc.ke);
  check(fabs(ke_err)<10,"Ke within 10%% (%.1f%%)",ke_err);
  { /* info: two-point Ke (cancels Coulomb friction) */
    double u[2],ww[2]; g_cmd.mode=0; g_cmd.apply=1; g_cmd.target=2.0f; run(1.5); u[0]=g_foc.uq; ww[0]=g_foc.vel;
    g_cmd.target=4.0f; g_cmd.apply=1; run(1.5); u[1]=g_foc.uq; ww[1]=g_foc.vel;
    printf("  info: two-point Ke=(%.3f-%.3f)/(%.2f-%.2f)=%.5f (true %.5f)\n",u[1],u[0],ww[1],ww[0],(u[1]-u[0])/(ww[1]-ww[0]),KE_TRUE);
    g_cmd.target=0.0f; g_cmd.apply=1; run(1.0); g_cmd.mode=3; g_cmd.target=g_foc.pos; g_cmd.apply=1; run(0.5); }
  run(1.0);
  printf("after: pos=%.4f target=%.4f vel=%.3f\n",g_foc.pos,g_foc.target,g_foc.vel);

  /* 5 fault */
  printf("\n--- 5 Encoder fault ---\n");
  double pos_b=g_foc.pos, th_b=th_m; inject_bad_reads=5; run(0.01);
  check(g_app_state==FOC_APP_FAULT,"state=%s (expect FAULT)",stn(g_app_state));
  check(en==0 && isr_on==0,"EN=%d ISR=%d (expect 0,0)",en,isr_on);
  printf("pending bad reads after fault: %d\n",inject_bad_reads);
  run(0.1);
  g_cmd.clear_fault=1; run(0.002);
  check(g_app_state==FOC_APP_RUNNING,"after clear_fault state=%s",stn(g_app_state));
  run(0.5);
  printf("rotor moved %.4f rad, g_foc.pos before %.4f after %.4f (target %.4f)\n",th_m-th_b,pos_b,g_foc.pos,g_foc.target);
  check(fabs(th_m-th_b)<0.02,"physical jump %.4f rad (<0.02)",fabs(th_m-th_b));
  check(fabs(g_foc.pos-pos_b)<0.02,"g_foc.pos frame jump %.4f rad (<0.02)",fabs(g_foc.pos-pos_b));

  /* 6 stop / resume */
  printf("\n--- 6 Stop/resume ---\n");
  double old_t=g_foc.target; g_cmd.stop=1; run(0.002);
  check(g_app_state==FOC_APP_STOPPED && en==0,"state=%s EN=%d",stn(g_app_state),en);
  th_m+=0.5; w=0; run(0.1);
  double pos_left=enc_sign*(th_m-th_m)+0; (void)pos_left;
  printf("rotor turned +0.5 rad by hand (frame delta %+.2f); old g_foc.pos(stale)=%.4f, old target %.4f\n",enc_sign*0.5,g_foc.pos,old_t);
  double newt=old_t+1.0; double th_s=th_m;
  g_cmd.mode=3; g_cmd.target=(float)newt; g_cmd.apply=1;
  /* single Loop pass */
  { int s0=step_ctr; while((step_ctr+1)%20!=0) { one_step(); step_ctr++; } one_step(); step_ctr++; FocApp_Loop(); (void)s0; }
  printf("after ONE Loop pass: state=%s apply=%d g_foc.target=%.4f g_foc.pos=%.4f (cmd target %.4f, old target %.4f)\n",stn(g_app_state),g_cmd.apply,g_foc.target,g_foc.pos,newt,old_t);
  printf("  => resume_hold then apply %s in the same pass\n",(g_cmd.apply==0&&fabs(g_foc.target-newt)<1e-4)?"CONSUMED (new target used directly)":(g_cmd.apply?"still pending":"LOST/other"));
  run(2.0);
  printf("after 2 s: g_foc.pos=%.4f target=%.4f err=%.4f, rotor moved %.4f rad\n",g_foc.pos,g_foc.target,g_foc.target-g_foc.pos,th_m-th_s);
  check(fabs(g_foc.target-g_foc.pos)<0.01,"reached applied target, err %.5f",g_foc.target-g_foc.pos);

  /* 7 mode switch */
  printf("\n--- 7 Mode switch glitch ---\n");
  run(0.2);
  max_uq_win=0; win_steps=60; g_cmd.mode=0; g_cmd.target=0; g_cmd.apply=1; double pre=g_foc.uq;
  run(0.002); double u1=max_uq_win; /* window collected over the following 2 ms */
  printf("POS->VOLT(0): uq before %.3f, max|Uq| in 3 ms after %.3f V, max|iq| %.3f\n",pre,u1,fabs(iq));
  run(0.1);
  max_uq_win=0; win_steps=60; g_cmd.mode=3; g_cmd.target=(float)g_foc.pos; g_cmd.apply=1; double p1=g_foc.pos; max_i=0;
  run(0.002); double u2=max_uq_win;
  printf("VOLT->POS(cur): max|Uq| in 3 ms after %.3f V (target=%.4f)\n",u2,p1);
  run(0.5);
  printf("settled: err %.5f rad, max I %.3f A\n",g_foc.target-g_foc.pos,max_i);
  check(u1<FOC_VOLTAGE_LIMIT && u2<FOC_VOLTAGE_LIMIT && u1<1.0 && u2<1.0,"max|Uq| %.3f / %.3f V bounded (<1 V, no glitch)",u1,u2);

  /* 7b: under 50% load, switch POS->VOLT(0)->POS */
  load=tl; run(2.0); double pe=g_foc.target-g_foc.pos; double uqh=g_foc.uq;
  max_uq_win=0; win_steps=60; g_cmd.mode=0; g_cmd.target=0; g_cmd.apply=1; run(0.003); double v1=max_uq_win;
  run(0.05); max_uq_win=0; win_steps=60; g_cmd.mode=3; g_cmd.target=(float)g_foc.pos; g_cmd.apply=1; run(0.003); double v2=max_uq_win;
  run(1.0);
  printf("7b under load: hold uq=%.3f err=%.4f; max|Uq| after ->VOLT %.3f, after ->POS %.3f; final err %.4f\n",uqh,pe,v1,v2,g_foc.target-g_foc.pos);
  load=0;
  printf("\n%s (%d failures)\n",fails?"SOME FAILED":"ALL PASSED",fails);
  return fails?1:0;
}
