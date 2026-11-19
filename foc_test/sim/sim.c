#include "foc_voltage.h"
#include <stdio.h>
#include <math.h>
/* Basit PMSM modeli (dq), enkoder ters yönde ve ofsetli bağlı */
static double th_m=1.234, w=0, id=0, iq=0; static int PP=7; static double R=0.5; static const double L=0.0005,LAM=0.01,J=2e-5,B=1e-5,VB=12;
static double Tcog=0, Fcoul=0, enc_off=0.7; static int enc_sign=-1; /* kogging genligi, Coulomb surtunme, enkoder ofset/yon */
static double da=.5,db=.5,dc=.5; static int en=0; static int CPR=4096; static double load=0, visc=0;
uint32_t hal_encoder_raw(void){ double a=fmod(enc_sign*th_m+enc_off,2*M_PI); if(a<0)a+=2*M_PI; return (uint32_t)(a/(2*M_PI)*CPR)%CPR; }
void hal_pwm_set(float a,float b,float c){da=a;db=b;dc=c;}
void hal_pwm_enable(int on){en=on;}
static void step(double dt){
  double va=(da-.5)*VB, vb=(db-.5)*VB, vc=(dc-.5)*VB; if(!en) va=vb=vc=0;
  double cm=(va+vb+vc)/3; va-=cm; vb-=cm; vc-=cm;
  double al=(2.0/3)*(va-0.5*vb-0.5*vc), be=(2.0/3)*(0.8660254*(vb-vc));
  double te=th_m*PP, c=cos(te), s=sin(te);
  double vd=al*c+be*s, vq=-al*s+be*c, we=w*PP;
  for(int k=0;k<10;k++){ double h=dt/10;
    double did=(vd-R*id+we*L*iq)/L, diq=(vq-R*iq-we*L*id-we*LAM)/L;
    id+=did*h; iq+=diq*h; double T=1.5*PP*LAM*iq - B*w - load - visc*w - Tcog*sin(6*PP*th_m) - Fcoul*tanh(w/0.05); w+=T/J*h; th_m+=w*h; }
}
static double t=0; void hal_delay_ms(uint32_t ms){ for(uint32_t i=0;i<ms*20;i++){ step(5e-5); t+=5e-5; } }
static void old_scenario(void){
  foc_t m={.pole_pairs=7,.cpr=CPR,.vbus=12,.phase_resistance=0.5f,.phase_inductance=0.0005f,.ke=0,.i_limit=3,.voltage_limit=6,
    .align_voltage=0.8f,.fast_dt=5e-5f,.ctrl_div=20,.vel_kp=0.05f,.vel_ki=1.0f,.vel_lpf_tc=0.005f,.pos_kp=20,.vel_limit=50,.latency_s=7.5e-5f};
  int r=foc_align(&m); printf("align=%d dir=%d pp_check=%.3f\n",r,m.dir,m.pp_check);
  #define RUN(sec) for(int i=0;i<(int)((sec)/5e-5);i++){ foc_update(&m); step(5e-5); }
  foc_set_target(&m,FOC_MODE_VOLTAGE,2.0f); RUN(1.0);
  printf("VOLTAGE Uq=2: vel=%.1f rad/s  id=%.3f iq=%.3f  -> Ke~%.4f (gercek %.4f)\n",m.vel,id,iq,m.uq/m.vel,1.5*0+LAM*PP);
  m.ke=m.uq/m.vel;
  foc_set_target(&m,FOC_MODE_VELOCITY,30.0f); RUN(1.0); printf("VEL 30: vel=%.2f id=%.3f iq=%.3f\n",m.vel,id,iq);
  load=0.02; RUN(1.0); printf("VEL 30 + yuk: vel=%.2f iq=%.3f (Iq_tahmin=%.3f)\n",m.vel,iq,(m.uq-m.ke*m.vel)/0.5);
  load=0; foc_set_target(&m,FOC_MODE_POSITION,10.0f); RUN(3.0); printf("POS 10: pos=%.3f vel=%.2f\n",m.pos,m.vel);
  foc_set_target(&m,FOC_MODE_VELOCITY,-60.0f); double imax=0; for(int i=0;i<40000;i++){foc_update(&m);step(5e-5); if(fabs(iq)>imax)imax=fabs(iq);} 
  printf("VEL -60 (ani yon degisimi): vel=%.2f  max|iq|=%.2f A (limit 3)\n",m.vel,imax);
  visc=0.003; foc_set_target(&m,FOC_MODE_CURRENT_EST,1.0f); load=0; RUN(1.0); printf("CURRENT_EST 1A: iq=%.3f id=%.3f vel=%.1f\n",iq,id,m.vel);
}

/* ---- Yeni senaryo: R=13, 14 bit mutlak enkoder, kutup cifti tespiti, kalibrasyon atlama ---- */
static foc_t make_foc(void){
  foc_t m={.cpr=16384,.vbus=12,.phase_resistance=13.0f,.phase_inductance=0,.ke=0,.i_limit=0.4f,.voltage_limit=6,
    .align_voltage=2.5f,.fast_dt=5e-5f,.ctrl_div=20,.vel_kp=0.5f,.vel_ki=10,.vel_lpf_tc=0.01f,.pos_kp=20,.vel_limit=20,.latency_s=80e-6f};
  return m;
}
static void new_scenario(void){
  const int pps[3]={7,11,14}; const double offs[3]={2.9,5.1,0.3}; const int sgn[3]={-1,1,-1};
  printf("\n=== Yeni senaryo: R=13, CPR=16384, cogging+Coulomb ===\n");
  for(int k=0;k<3;k++){
    PP=pps[k]; R=13.0; CPR=16384; enc_off=offs[k]; enc_sign=sgn[k]; Tcog=0.002; Fcoul=0.001;
    th_m=0.4+0.37*k; w=id=iq=0; load=visc=0; en=0; da=db=dc=.5;
    foc_t m=make_foc();
    int pp=foc_detect_pole_pairs(&m,8);
    printf("[PP=%d enc_sign=%+d off=%.1f] detect: ret=%d pp_est=%.3f  %s\n",PP,enc_sign,enc_off,pp,m.pp_est,pp==PP?"PASS":"FAIL");
    if(pp<=0) continue;
    m.pole_pairs=pp;
    int r=foc_align(&m);
    printf("  align=%d dir=%d (beklenen %d) pp_check=%.3f elec_offset=%.3f\n",r,m.dir,enc_sign,m.pp_check,m.elec_offset);
    float tgt=m.pos+3.0f; foc_set_target(&m,FOC_MODE_POSITION,tgt); RUN(3.0);
    printf("  POS +3 rad: pos=%.4f hedef=%.4f hata=%.4f rad (%.2f count) vel=%.3f\n",m.pos,tgt,m.pos-tgt,(m.pos-tgt)*16384/(2*M_PI),m.vel);
    /* guc dongusu: PWM kapali, rotor elle oynatilir, yeni foc_t, align YOK */
    int dir=m.dir; float off=m.elec_offset; foc_stop(&m); w=id=iq=0; th_m+=0.9;
    for(int i=0;i<4000;i++) step(5e-5);       /* 0.2 sn serbest, kogging ile oturur */
    foc_t m2=make_foc();
    foc_set_calibration(&m2,pp,dir,off); hal_pwm_enable(1);
    double th0=th_m; float p0=m2.pos; float tgt2=p0+2.0f;
    foc_set_target(&m2,FOC_MODE_POSITION,tgt2);
    for(int i=0;i<(int)(3.0/5e-5);i++){ foc_update(&m2); step(5e-5); }
    printf("  KALIBRASYON ATLA: pos=%.4f hedef=%.4f hata=%.4f rad, gercek rotor hareketi=%.4f rad (beklenen 2.0) %s\n",
      m2.pos,tgt2,m2.pos-tgt2,th_m-th0,(fabs(m2.pos-tgt2)<0.02&&fabs(th_m-th0-2.0)<0.03)?"PASS":"FAIL");
  }
}

int main(){ old_scenario(); new_scenario(); return 0; }
