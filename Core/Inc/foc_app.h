/* foc_app.h — uygulama katmani (CubeIDE Live Expressions ile izle/komut ver) */
#ifndef FOC_APP_H
#define FOC_APP_H

#include <stdint.h>
#include "foc_voltage.h"

typedef struct {
    int   mode;        /* foc_mode_t: 0 VOLTAGE, 1 CURRENT_EST, 2 VELOCITY, 3 POSITION */
    float target;
    int   apply;       /* 1 yaz -> mode/target uygulanir, otomatik 0 olur */
    int   stop;        /* 1 -> motoru durdur (EN kapali) */
    int   clear_fault; /* 1 -> enkoder hatasini temizle ve yeniden baslat */
    int   measure_ke;  /* 1 -> Ke olcumu (motor serbest doner!) */
} foc_cmd_t;

typedef enum {
    FOC_APP_INIT = 0,
    FOC_APP_ENC_FAIL,
    FOC_APP_PP_FAIL,
    FOC_APP_ALIGN_FAIL,
    FOC_APP_RUNNING,
    FOC_APP_FAULT,
    FOC_APP_MEASURING_KE,
    FOC_APP_STOPPED          /* g_cmd.stop sonrasi; apply ile devam eder */
} foc_app_state_t;

extern foc_t g_foc;
extern volatile foc_cmd_t g_cmd;
extern volatile foc_app_state_t g_app_state;
extern volatile int g_detected_pp;
extern volatile float g_measured_ke;

void FocApp_Init(void);   /* main'de MX_* sonrasi, bir kez (bloklayici, otomatik kutup tespitiyle ~21 sn, kayitli kalibrasyonla <1 sn) */
void FocApp_Loop(void);   /* while(1) icinde, bloklamaz */

#endif
