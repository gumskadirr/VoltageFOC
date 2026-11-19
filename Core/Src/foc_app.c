/* foc_app.c — FOC uygulama katmani (acilis, komutlar, hata yonetimi) */
#include "foc_app.h"
#include "foc_config.h"
#include "foc_board.h"
#include "foc_voltage.h"
#include "foc_encoder_mt6701.h"
#include "stm32f4xx.h"
#include <math.h>   /* __disable_irq / __enable_irq */

extern uint32_t HAL_GetTick(void);

foc_t g_foc;
volatile foc_cmd_t g_cmd;
volatile foc_app_state_t g_app_state = FOC_APP_INIT;
volatile int g_detected_pp = 0;
volatile float g_measured_ke = 0.0f;

#define KE_TEST_UQ1_V    2.0f     /* Ke olcumu 1. nokta [V] */
#define KE_TEST_UQ2_V    4.0f     /* Ke olcumu 2. nokta [V] */
#define KE_SPIN_MS       1500U    /* her noktada donus suresi */
#define KE_BRAKE_MS      1000U    /* hiz=0 ile fren suresi */

typedef enum { KE_IDLE = 0, KE_SPIN1, KE_SPIN2, KE_BRAKE } ke_phase_t;
static ke_phase_t s_ke_phase = KE_IDLE;
static uint32_t   s_ke_t0;
static float      s_ke_uq1, s_ke_w1;

/* Thread'den mod+hedef degisimi: ISR arada yeni hedefi eski modla gormesin
 * (ornegin konum [rad] degeri bir adim boyunca Uq [V] olarak uygulanmasin). */
static void set_target_atomic(foc_mode_t md, float tgt)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    foc_set_target(&g_foc, md, tgt);
    __set_PRIMASK(pm);
}

static void foc_isr(void)
{
    if (g_enc.fault) {                    /* enkoder hatasi: tork kes */
        foc_stop(&g_foc);
        g_app_state = FOC_APP_FAULT;
        FocBoard_StopIsr();
        return;
    }
    foc_update(&g_foc);
}

/* Mevcut konumda tut ve ISR'i baslat (ISR KAPALIYKEN cagir).
 * Guvenli yol: foc_set_calibration enkoder izlemeyi o anki ham aciyla yeniden kurar,
 * boylece g_foc.pos/count GUNCEL olur (align/tespit sonrasi rotor kaymis olabilir,
 * pos bayat kalmasin diye). Hiz/integral sifir, Uq=0. Sonra hedef = pos -> ziplama yok. */
static void start_holding(int keep_turns)
{
    float p_old = g_foc.pos;              /* durdurulmadan onceki cok turlu konum */
    foc_set_calibration(&g_foc, g_foc.pole_pairs, g_foc.dir, g_foc.elec_offset);
    if (keep_turns) {
        /* foc_set_calibration sayaci tek tur (0..2pi) aralığından kurar; tur sayisini
         * eski konumdan geri ekle. Rotor durukken pi'den az dondurulduysa kesin dogru. */
        int64_t turns = (int64_t)llroundf((p_old - g_foc.pos) / 6.2831853f);
        g_foc.count     += turns * (int64_t)g_foc.cpr;
        g_foc.count_prev = g_foc.count;
        g_foc.pos        = (float)g_foc.count * 6.2831853f / (float)g_foc.cpr;
    }
    foc_set_target(&g_foc, FOC_MODE_POSITION, g_foc.pos);
    hal_pwm_enable(1);
    /* Kalibrasyondaki (thread) enkoder hatalari ISR'yi hemen FAULT'a dusurmesin;
     * calisma sirasinda 3 ardisik hata yine FAULT verir. */
    MT6701F_ClearFault(&g_enc);
    FocBoard_StartIsr(foc_isr);
    g_app_state = FOC_APP_RUNNING;
}

void FocApp_Init(void)
{
    FocBoard_Init();
    if (!FocEnc_Init()) { g_app_state = FOC_APP_ENC_FAIL; return; }

    g_foc.cpr              = FOC_CPR;
    g_foc.vbus             = FOC_VBUS;
    g_foc.phase_resistance = FOC_PHASE_R;
    g_foc.phase_inductance = FOC_PHASE_L;
    g_foc.ke               = FOC_KE;
    g_foc.i_limit          = FOC_I_LIMIT;
    g_foc.voltage_limit    = FOC_VOLTAGE_LIMIT;
    g_foc.align_voltage    = FOC_ALIGN_VOLTAGE;
    g_foc.fast_dt          = FOC_PWM_DT_S;
    g_foc.ctrl_div         = FOC_CTRL_DIV;
    g_foc.vel_kp           = FOC_VEL_KP;
    g_foc.vel_ki           = FOC_VEL_KI;
    g_foc.vel_lpf_tc       = FOC_VEL_LPF_TC;
    g_foc.pos_kp           = FOC_POS_KP;
    g_foc.vel_limit        = FOC_VEL_LIMIT;
    g_foc.latency_s        = FOC_LATENCY_S;

#if FOC_USE_STORED_CALIB
    foc_set_calibration(&g_foc, FOC_CALIB_POLE_PAIRS, FOC_CALIB_DIR, FOC_CALIB_ELEC_OFFSET);
    g_detected_pp = FOC_CALIB_POLE_PAIRS;
#else
    int pp = FOC_POLE_PAIRS;
    if (pp == 0) {
        pp = foc_detect_pole_pairs(&g_foc, 8);
        if (pp <= 0) {
            g_app_state = FOC_APP_PP_FAIL;
            hal_pwm_enable(0);
            return;
        }
    }
    g_detected_pp = pp;
    g_foc.pole_pairs = pp;
    if (foc_align(&g_foc) != 0) {         /* foc_stop icerde cagrilabilir; EN'i kapat */
        g_app_state = FOC_APP_ALIGN_FAIL;
        hal_pwm_enable(0);
        return;
    }
#endif
    start_holding(0);
}

/* Durdurulmus / hatali durumdan yeniden basla (ISR kapali) */
static void resume_hold(void)
{
    start_holding(1);
}

static void ke_finish_hold(void)
{
    s_ke_phase = KE_IDLE;
    set_target_atomic(FOC_MODE_POSITION, g_foc.pos);
    g_app_state = FOC_APP_RUNNING;
}

void FocApp_Loop(void)
{
    /* Baslatma basarisizsa kalibrasyon yok: hicbir komut motoru surmesin
     * (ENC_FAIL'de cpr=0 -> resume yolunda sifira bolme). Cikis: reset. */
    if (g_app_state == FOC_APP_INIT || g_app_state == FOC_APP_ENC_FAIL ||
        g_app_state == FOC_APP_PP_FAIL || g_app_state == FOC_APP_ALIGN_FAIL) {
        g_cmd.stop = 0; g_cmd.apply = 0; g_cmd.clear_fault = 0; g_cmd.measure_ke = 0;
        return;
    }

    /* stop her durumda gecerli */
    if (g_cmd.stop) {
        g_cmd.stop = 0;
        FocBoard_StopIsr();
        foc_stop(&g_foc);
        s_ke_phase = KE_IDLE;
        if (g_app_state != FOC_APP_FAULT) g_app_state = FOC_APP_STOPPED;
        return;
    }

    if (g_cmd.clear_fault) {
        g_cmd.clear_fault = 0;
        if (g_app_state == FOC_APP_FAULT) {
            FocBoard_StopIsr();
            MT6701F_ClearFault(&g_enc);
            if (!g_enc.fault) resume_hold();   /* hala hata varsa FAULT'ta kal */
        }
    }

    if (g_app_state == FOC_APP_STOPPED && g_cmd.apply) {
        resume_hold();                         /* pozisyonda tutarak devam */
    }

    if (g_app_state == FOC_APP_RUNNING) {
        if (g_cmd.apply) {
            int md = g_cmd.mode;
            if (md >= FOC_MODE_VOLTAGE && md <= FOC_MODE_POSITION)
                set_target_atomic((foc_mode_t)md, g_cmd.target);
            g_cmd.apply = 0;
        }
        /*
         * UYARI: Ke olcumu motoru SERBESTCE dondurur (Uq = 2 V sonra 4 V, ~3 sn).
         * Iki noktali olcum: Ke = (Uq2-Uq1)/(w2-w1). Surtunmenin R*I dususu iki
         * noktada ayni oldugundan farkta sadeleşir (tek nokta Ke'yi ~%10 yuksek olcer).
         * Gimbal yukunu / mekanigi once sok; kablo sarilmasi ve carpma riski.
         */
        if (g_cmd.measure_ke) {
            g_cmd.measure_ke = 0;
            g_measured_ke = 0.0f;
            set_target_atomic(FOC_MODE_VOLTAGE, KE_TEST_UQ1_V);
            s_ke_t0 = HAL_GetTick();
            s_ke_phase = KE_SPIN1;
            g_app_state = FOC_APP_MEASURING_KE;
        }
    } else if (g_app_state == FOC_APP_MEASURING_KE) {
        g_cmd.apply = 0;                       /* olcum sirasinda komut yok */
        uint32_t el = HAL_GetTick() - s_ke_t0;
        if (s_ke_phase == KE_SPIN1 && el >= KE_SPIN_MS) {
            s_ke_uq1 = g_foc.uq; s_ke_w1 = g_foc.vel;     /* 1. nokta */
            set_target_atomic(FOC_MODE_VOLTAGE, KE_TEST_UQ2_V);
            s_ke_t0 = HAL_GetTick();
            s_ke_phase = KE_SPIN2;
        } else if (s_ke_phase == KE_SPIN2 && el >= KE_SPIN_MS) {
            float duq = g_foc.uq - s_ke_uq1, dw = g_foc.vel - s_ke_w1;
            if (s_ke_w1 > 1.0f && dw > 1.0f && duq > 0.5f) {
                g_measured_ke = duq / dw;      /* iki noktali egim */
                g_foc.ke = g_measured_ke;
            }                                  /* aksi halde g_measured_ke = 0 kalir (olcum basarisiz) */
            set_target_atomic(FOC_MODE_VELOCITY, 0.0f);   /* aktif fren */
            s_ke_t0 = HAL_GetTick();
            s_ke_phase = KE_BRAKE;
        } else if (s_ke_phase == KE_BRAKE && el >= KE_BRAKE_MS) {
            ke_finish_hold();                  /* bulunan konumda tut */
        }
    }
}
