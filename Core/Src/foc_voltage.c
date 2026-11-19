/*
 * foc_voltage.c — Akım sensörsüz, enkoderli FOC
 */
#include "foc_voltage.h"
#include <math.h>
#include <stdlib.h>

#define TWO_PI     6.28318530718f
#define SQRT3_2    0.86602540378f
#define INV_SQRT3  0.57735026919f

static float wrap_2pi(float a)
{
    a = fmodf(a, TWO_PI);
    if (a < 0.0f) a += TWO_PI;
    return a;
}

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ---------------- Enkoder ---------------- */
static void encoder_reset(foc_t *m)
{
    uint32_t raw = hal_encoder_raw();
    m->prev_raw   = raw;
    m->turn_count = (m->dir > 0) ? (int32_t)raw : (int32_t)((m->cpr - (int32_t)raw) % m->cpr);
    m->count      = m->turn_count;
    m->count_prev = m->count;
}

/* Her çağrıda yarım turdan az dönmüş olmalı (20 kHz'de sorun değil) */
static void encoder_update(foc_t *m)
{
    uint32_t raw = hal_encoder_raw();
    int32_t d = (int32_t)raw - (int32_t)m->prev_raw;
    if (d >  m->cpr / 2) d -= m->cpr;
    if (d < -m->cpr / 2) d += m->cpr;
    m->prev_raw = raw;
    d *= m->dir;

    m->count += d;
    m->turn_count += d;
    if (m->turn_count >= m->cpr) m->turn_count -= m->cpr;
    else if (m->turn_count < 0)  m->turn_count += m->cpr;
}

static float mech_angle(const foc_t *m)   /* 0..2π, tek tur */
{
    return (float)m->turn_count * TWO_PI / (float)m->cpr;
}

/* ------------- Inverse Park + SVPWM ------------- */
static void apply_voltage(foc_t *m, float uq, float ud, float theta)
{
    /* Lineer bölge sınırı: |U| <= Vbus/√3 */
    float vmax = m->vbus * INV_SQRT3;
    float mag = sqrtf(uq * uq + ud * ud);
    if (mag > vmax) { float k = vmax / mag; uq *= k; ud *= k; }

    float s = sinf(theta), c = cosf(theta);
    float ua = ud * c - uq * s;              /* inverse Park */
    float ub = ud * s + uq * c;

    float va = ua;                           /* inverse Clarke */
    float vb = -0.5f * ua + SQRT3_2 * ub;
    float vc = -0.5f * ua - SQRT3_2 * ub;

    /* Min-max (common-mode) enjeksiyonu = SVPWM ile eşdeğer */
    float vmx = fmaxf(va, fmaxf(vb, vc));
    float vmn = fminf(va, fminf(vb, vc));
    float off = -0.5f * (vmx + vmn);

    float inv = 1.0f / m->vbus;
    hal_pwm_set(clampf((va + off) * inv + 0.5f, 0.0f, 1.0f),
                clampf((vb + off) * inv + 0.5f, 0.0f, 1.0f),
                clampf((vc + off) * inv + 0.5f, 0.0f, 1.0f));
}

/* ---------------- Hizalama ----------------
 * 1) d-ekseni boyunca voltaj → rotor elektriksel 0'a kilitlenir
 * 2) 1 elektriksel tur ileri sür → enkoder yönünü ve pole_pairs'i doğrula
 * 3) Geri dön, otur, elektriksel ofseti kaydet
 * DİKKAT: Bloklayıcıdır; foc_update çağıran kesme kapalıyken çalıştır.
 */
int foc_align(foc_t *m)
{
    const int steps = 500;

    m->dir = 1;
    m->uq = m->ud = 0.0f;
    hal_pwm_enable(1);
    encoder_reset(m);

    apply_voltage(m, 0.0f, m->align_voltage, 0.0f);
    hal_delay_ms(700);
    encoder_update(m);
    int64_t c0 = m->count;

    for (int i = 0; i <= steps; i++) {
        apply_voltage(m, 0.0f, m->align_voltage, TWO_PI * (float)i / steps);
        hal_delay_ms(2);
        encoder_update(m);
    }
    int64_t moved = m->count - c0;

    for (int i = steps; i >= 0; i--) {
        apply_voltage(m, 0.0f, m->align_voltage, TWO_PI * (float)i / steps);
        hal_delay_ms(2);
    }
    hal_delay_ms(700);

    int64_t expected = m->cpr / m->pole_pairs;
    /* hata durumunda da gorunsun (teshis): ~1.0 olmali */
    m->pp_check = (float)llabs(moved) * (float)m->pole_pairs / (float)m->cpr;
    if (llabs(moved) < expected / 4) {        /* motor dönmedi veya enkoder saymıyor */
        foc_stop(m);
        return -1;
    }
    m->dir = (moved > 0) ? 1 : -1;
    m->pp_check = (float)llabs(moved) * (float)m->pole_pairs / (float)m->cpr;

    /* Yön belli oldu: sayaçları yeniden kur, rotor şu an elektriksel 0'da */
    encoder_reset(m);
    m->elec_offset = wrap_2pi(mech_angle(m) * (float)m->pole_pairs);

    apply_voltage(m, 0.0f, 0.0f, 0.0f);
    m->vel = 0.0f;
    m->vel_integ = 0.0f;
    m->pos = (float)m->count * TWO_PI / (float)m->cpr;
    m->mode = FOC_MODE_VOLTAGE;
    m->target = 0.0f;

    if (m->pp_check < 0.8f || m->pp_check > 1.2f)
        return -2;                            /* pole_pairs veya cpr hatalı olabilir */
    return 0;
}

/* ---------------- Kutup cifti tespiti ----------------
 * Alan (d-ekseni voltaji) elec_revs elektriksel tur doner; rotor mekanik olarak
 * elec_revs/pp tur doner. pp = elec_revs * cpr / |sayim farki|.
 * elec_revs secimi (uygulamada 8):
 *   - Kucuk elec_revs: pp buyukse (30-40) mekanik hareket az kalir (8 tur ve pp=40
 *     icin ~3300 sayim; 1 tur olsa ~410), sayim hatasi / kogging pp'yi bozar.
 *   - Buyuk elec_revs: pp kucukse (2-3) rotor cok tur doner (8 tur, pp=2 -> 4 mekanik
 *     tur), sure uzar (tur basi 1 sn) ve kablo/yuk sarilir.
 *   8 tur: pp ~2..40 arasinda iyi sonuc verir (sure ~10 sn).
 * DIKKAT: Bloklayici; foc_update kesmesi kapaliyken calistir.
 */
int foc_detect_pole_pairs(foc_t *m, int elec_revs)
{
    const int steps = 500;                    /* elektriksel tur basina adim */

    if (elec_revs < 1) return -1;
    m->dir = 1;
    m->uq = m->ud = 0.0f;
    m->pp_est = 0.0f;
    hal_pwm_enable(1);
    encoder_reset(m);

    apply_voltage(m, 0.0f, m->align_voltage, 0.0f);   /* rotoru elek. 0'a kilitle */
    hal_delay_ms(700);
    encoder_update(m);
    int64_t c0 = m->count;

    int total = steps * elec_revs;
    for (int i = 0; i <= total; i++) {                /* yavas ileri tarama */
        float th = TWO_PI * (float)(i % steps) / (float)steps;
        apply_voltage(m, 0.0f, m->align_voltage, th);
        hal_delay_ms(2);
        encoder_update(m);                            /* cok turlu takip icin her adimda */
    }
    hal_delay_ms(500);
    encoder_update(m);
    int64_t c1 = m->count;

    for (int i = total; i >= 0; i--) {                /* geri don (kablo/yuk icin) */
        float th = TWO_PI * (float)(i % steps) / (float)steps;
        apply_voltage(m, 0.0f, m->align_voltage, th);
        hal_delay_ms(2);
        encoder_update(m);
    }
    hal_delay_ms(500);
    encoder_update(m);
    apply_voltage(m, 0.0f, 0.0f, 0.0f);               /* 0 V; EN'e dokunma */

    int64_t dc = llabs(c1 - c0);
    int64_t min_dc = (int64_t)m->cpr * elec_revs / 100;   /* pp<=100 varsayimi */
    if (min_dc < 16) min_dc = 16;
    if (dc < min_dc) return -1;                       /* motor donmedi */

    float pp = (float)elec_revs * (float)m->cpr / (float)dc;
    m->pp_est = pp;
    long r = lroundf(pp);
    if (fabsf(pp - (float)r) > 0.25f || r < 1) return -2;  /* belirsiz */
    return (int)r;
}

/* Mutlak enkoder: elek. aci = mech*pp - offset guc dongusunden sonra da gecerli */
void foc_set_calibration(foc_t *m, int pole_pairs, int dir, float elec_offset)
{
    m->pole_pairs  = pole_pairs;
    m->dir         = (dir >= 0) ? 1 : -1;
    m->elec_offset = elec_offset;
    encoder_reset(m);
    m->pos         = (float)m->count * TWO_PI / (float)m->cpr;
    m->vel         = 0.0f;
    m->vel_integ   = 0.0f;
    m->ctrl_cnt    = 0;
    m->uq = m->ud  = 0.0f;
    m->mode        = FOC_MODE_VOLTAGE;
    m->target      = 0.0f;
}

/* -------------- Yavaş döngü: hız/konum/tork -------------- */
static void control_step(foc_t *m, float dt)
{
    int64_t dc = m->count - m->count_prev;
    m->count_prev = m->count;
    m->pos = (float)m->count * TWO_PI / (float)m->cpr;

    float v_raw = (float)dc * TWO_PI / (float)m->cpr / dt;
    float a = dt / (m->vel_lpf_tc + dt);
    m->vel += a * (v_raw - m->vel);

    float uq = 0.0f, ud = 0.0f;
    float tgt = m->target;
    float bemf = m->ke * m->vel;

    if (m->mode == FOC_MODE_VOLTAGE) {
        uq = tgt;
    } else if (m->mode == FOC_MODE_CURRENT_EST) {
        uq = tgt * m->phase_resistance + bemf;           /* Uq = R*Iq + geri-EMK */
    } else {                                             /* VELOCITY / POSITION */
        float vref = tgt;
        if (m->mode == FOC_MODE_POSITION)
            vref = clampf(m->pos_kp * (tgt - m->pos), -m->vel_limit, m->vel_limit);
        float err = vref - m->vel;
        m->vel_integ = clampf(m->vel_integ + m->vel_ki * err * dt,
                              -m->voltage_limit, m->voltage_limit);
        uq = m->vel_kp * err + m->vel_integ + m->ke * vref;  /* Ke ileri besleme */
    }

    /* Sanal akım limiti: |(Uq - Ke*w)/R| <= Imax  → sensörsüz aşırı akım koruması */
    if (m->phase_resistance > 0.0f && m->i_limit > 0.0f) {
        float du = m->i_limit * m->phase_resistance;
        uq = clampf(uq, bemf - du, bemf + du);
    }
    uq = clampf(uq, -m->voltage_limit, m->voltage_limit);

    /* Dekuplaj: Ud = -we*L*Iq_tahmin → yüksek hızda Id≈0 kalır (L ve Ke girilmişse) */
    if (m->phase_resistance > 0.0f && m->phase_inductance > 0.0f && m->ke > 0.0f) {
        float iq_est = (uq - bemf) / m->phase_resistance;
        ud = -(m->vel * (float)m->pole_pairs) * m->phase_inductance * iq_est;
    }

    m->uq = uq;
    m->ud = ud;
}

/* -------------- Hızlı döngü: PWM kesmesinde -------------- */
void foc_update(foc_t *m)
{
    encoder_update(m);

    if (++m->ctrl_cnt >= m->ctrl_div) {
        m->ctrl_cnt = 0;
        control_step(m, m->fast_dt * (float)m->ctrl_div);
    }

    /* Elektriksel açı + gecikme kompanzasyonu (yüksek hızda faz kaymasını azaltır) */
    float we = m->vel * (float)m->pole_pairs;
    float th = mech_angle(m) * (float)m->pole_pairs - m->elec_offset + we * m->latency_s;
    apply_voltage(m, m->uq, m->ud, wrap_2pi(th));
}

void foc_set_target(foc_t *m, foc_mode_t mode, float target)
{
    if (mode != m->mode) m->vel_integ = 0.0f;
    m->target = target;
    m->mode = mode;
}

void foc_stop(foc_t *m)
{
    m->uq = m->ud = 0.0f;
    m->target = 0.0f;
    hal_pwm_set(0.5f, 0.5f, 0.5f);
    hal_pwm_enable(0);
}
