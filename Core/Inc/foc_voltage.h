#ifndef FOC_VOLTAGE_H
#define FOC_VOLTAGE_H

/*
 * foc_voltage.h — Akım sensörsüz, enkoderli FOC (voltaj modlu FOC)
 *
 * Prensip:
 *   - Rotor açısı enkoderden okunur (komütasyon için).
 *   - Akım ölçülmediği için Id/Iq PI döngüsü YOK; Uq doğrudan verilir, Ud = 0.
 *   - İstenirse Iq "tahmini" olarak uygulanır:  Uq = Iq*R + Ke*w  (sanal akım kontrolü)
 *   - Uq, sanal akım limiti ile sınırlanır:  |Uq - Ke*w| <= Imax*R
 *
 * Kullanım:
 *   1) Aşağıdaki 4 HAL fonksiyonunu kendi MCU'na göre yaz.
 *   2) foc_t'yi doldur, foc_align() çağır (motor biraz döner, kesmeler kapalıyken!).
 *   3) PWM kesmesinde (ör. 10–20 kHz) foc_update() çağır.
 *   4) Ana döngüden foc_set_target() ile mod/hedef ver.
 *
 *   foc_t m = {
 *       .pole_pairs = 7, .cpr = 4096, .vbus = 12.0f,
 *       .phase_resistance = 0.5f,   // faz direnci (faz-faz ölçümünün yarısı)
 *       .phase_inductance = 0.0f,   // bilinmiyorsa 0
 *       .ke = 0.0f,                 // ölçülene kadar 0 (aşağıya bkz.)
 *       .i_limit = 3.0f,            // sanal akım limiti [A]
 *       .voltage_limit = 6.0f,      // mutlak Uq limiti [V]
 *       .align_voltage = 1.0f,      // hizalama voltajı, ~ (0.3..0.5)*Imax*R
 *       .fast_dt = 1.0f/20000.0f, .ctrl_div = 20,   // 20 kHz FOC, 1 kHz kontrol
 *       .vel_kp = 0.2f, .vel_ki = 2.0f, .vel_lpf_tc = 0.005f,
 *       .pos_kp = 20.0f, .vel_limit = 50.0f,
 *       .latency_s = 1.5f/20000.0f,
 *   };
 *   if (foc_align(&m) != 0) { hata: motor dönmedi / enkoder sayılmıyor }
 *   foc_set_target(&m, FOC_MODE_VOLTAGE, 1.0f);   // 1 V Uq ile dön
 *
 * Ke ölçümü (önerilir): FOC_MODE_VOLTAGE'de yüksüz, sabit bir Uq ile dönerken
 *   Ke ≈ m.uq / m.vel   [V / (rad/s mekanik)]
 *   (Yüksüzken akım küçük olduğundan Uq neredeyse tamamen geri-EMK'dır.)
 *   DİKKAT: Komut verdiğin değeri değil, UYGULANAN m.uq'yu kullan. ke = 0 iken
 *   sanal akım limiti Uq'yu ±(i_limit*R) aralığına kırpar; ölçüm için bu yeterli,
 *   ama Ke girilene kadar hız/konum modunda bu hız sınırlı kalır.
 *   Alternatif: datasheet KV (rpm/V) biliniyorsa  m.ke = foc_ke_from_kv(KV);
 */

/* KV (rpm/V, hat-hat tepe) -> Ke [V/(rad/s mek)], faz tepe geri-EMK.
 * SimpleFOC'un estimateBEMF'i ile aynı dönüşüm: 60 / (2π·√3·KV). */
static inline float foc_ke_from_kv(float kv_rpm_per_volt)
{
    return 5.51329f / kv_rpm_per_volt;
}
#include <stdint.h>

/* ---- Kullanıcının yazacağı HAL fonksiyonları ---- */
uint32_t hal_encoder_raw(void);                       /* 0 .. cpr-1 arası enkoder sayacı */
void     hal_pwm_set(float da, float db, float dc);   /* 0..1 duty, 3 faz (merkez hizalı PWM) */
void     hal_pwm_enable(int on);                      /* sürücü EN / gate enable */
void     hal_delay_ms(uint32_t ms);

typedef enum {
    FOC_MODE_VOLTAGE = 0,   /* hedef = Uq [V] (tork ~ voltaj, en basit ve sağlam mod) */
    FOC_MODE_CURRENT_EST,   /* hedef = Iq [A] tahmini; R ve Ke gerekir */
    FOC_MODE_VELOCITY,      /* hedef = hız [rad/s mekanik] */
    FOC_MODE_POSITION       /* hedef = konum [rad mekanik, çok turlu] */
} foc_mode_t;

typedef struct {
    /* --- Parametreler --- */
    int     pole_pairs;
    int32_t cpr;              /* mekanik tur başına sayım (x4 sonrası) */
    float   vbus;             /* DC bara voltajı [V] */
    float   phase_resistance; /* [ohm] */
    float   phase_inductance; /* [H], 0 olabilir */
    float   ke;               /* [V/(rad/s mek)], 0 olabilir */
    float   i_limit;          /* sanal akım limiti [A] (R>0 ise aktif) */
    float   voltage_limit;    /* mutlak |Uq| limiti [V] */
    float   align_voltage;    /* [V] */
    float   fast_dt;          /* foc_update çağrı periyodu [s] */
    int     ctrl_div;         /* kontrol döngüsü = fast / ctrl_div */
    float   vel_kp, vel_ki;   /* hız PI: çıkış Volt */
    float   vel_lpf_tc;       /* hız filtresi zaman sabiti [s] */
    float   pos_kp;           /* konum P: çıkış rad/s */
    float   vel_limit;        /* konum modunda max hız [rad/s] */
    float   latency_s;        /* açı gecikme kompanzasyonu [s] */

    /* --- Komut (ana döngüden yazılır) --- */
    volatile foc_mode_t mode;
    volatile float      target;

    /* --- Durum (salt okunur) --- */
    int8_t  dir;              /* enkoder yönü (+1/-1), align bulur */
    float   elec_offset;      /* elektriksel sıfır [rad], align bulur */
    float   pp_check;         /* ~1.0 olmalı; değilse pole_pairs veya cpr yanlış */
    uint32_t prev_raw;
    int32_t turn_count;       /* 0..cpr-1 */
    int64_t count;            /* çok turlu toplam sayım */
    int64_t count_prev;
    float   pos;              /* [rad] */
    float   vel;              /* filtreli hız [rad/s] */
    float   vel_integ;
    float   uq, ud;           /* uygulanan voltajlar [V] */
    int     ctrl_cnt;
    float   pp_est;           /* foc_detect_pole_pairs ham tahmini (tam sayiya yakin olmali) */
} foc_t;

int  foc_align(foc_t *m);                              /* 0 = OK, <0 = hata */
void foc_update(foc_t *m);                             /* PWM kesmesinde çağır */
void foc_set_target(foc_t *m, foc_mode_t mode, float target);
void foc_stop(foc_t *m);

/* Kutup cifti otomatik tespiti (bloklayici, thread baglami, ISR KAPALI olmali).
 * m->cpr, m->vbus, m->align_voltage dolu olmali; pole_pairs bilinmesi gerekmez.
 * Donus: kutup cifti (>0), -1 = motor donmedi, -2 = belirsiz (tam sayi degil).
 * pole_pairs alanini YAZMAZ; cagiran atar. PWM sonunda 0 V'ta, EN acik kalir. */
int  foc_detect_pole_pairs(foc_t *m, int elec_revs);

/* Mutlak enkoderle hizalamayi atla: onceden bulunmus degerleri yukle. */
void foc_set_calibration(foc_t *m, int pole_pairs, int dir, float elec_offset);

#endif
