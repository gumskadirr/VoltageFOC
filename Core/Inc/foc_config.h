/* foc_config.h — kullanici ayarlari (tum ayarlanabilir degerler burada) */
#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* ---- Besleme / motor ---- */
#define FOC_VBUS            12.0f   /* DC bara voltaji [V] */

/* Faz direnci [ohm]: 26 ohm iki uc arasi olcum / 2 = 13 ohm (yildiz baglanti varsayimi).
 * Bu tek define; yanlissa sanal akim limiti de yanlis olur. */
#define FOC_PHASE_R         13.0f
#define FOC_PHASE_L         0.0f    /* [H], bilinmiyor -> 0 (dekuplaj kapali) */

/* Ke [V/(rad/s mek)]: bilinmiyorsa 0. Olcum: FOC_MODE_VOLTAGE'de yuksuz, sabit Uq ile
 * donerken Ke = g_foc.uq / g_foc.vel  (veya g_cmd.measure_ke = 1 ile otomatik; sonuc
 * g_measured_ke). Cikan degeri buraya yaz. */
#define FOC_KE              0.0f

/* ---- Limitler ---- */
#define FOC_I_LIMIT         0.4f    /* sanal akim limiti [A] (I*R = 5.2 V) */
#define FOC_VOLTAGE_LIMIT   6.0f    /* mutlak |Uq| limiti [V] */
#define FOC_ALIGN_VOLTAGE   2.5f    /* hizalama / kutup tespiti voltaji [V] (~0.19 A) */

/* ---- Enkoder / zamanlama ---- */
#define FOC_CPR             16384   /* MT6701 14 bit */
#define FOC_CTRL_DIV        20      /* 20 kHz / 20 = 1 kHz kontrol dongusu */
#define FOC_LATENCY_S       80e-6f  /* aci gecikme kompanzasyonu [s] */

/* ---- Kazanclar (BASLANGIC DEGERLERI, motora gore ayarla) ----
 * Simulasyonda (pp=11, R=13): KP=0.5/KI=10 -> yuk bozucusunda tepe hata ~0.096 rad,
 * ~1.5 sn'de soner; adimda akim sanal limitte kalir.
 * Daha sert gimbal icin denenmis: KP=1.0/KI=30 -> tepe hata ~0.053 rad, <1 sn'de soner,
 * ama adim aninda gercek akim limiti ~%25 asar (geçici, L nedeniyle). Motorun akim
 * sinirini bildikten sonra gec. */
#define FOC_VEL_KP          0.5f    /* hiz P [V/(rad/s)] */
#define FOC_VEL_KI          10.0f   /* hiz I */
#define FOC_VEL_LPF_TC      0.01f   /* hiz filtresi zaman sabiti [s] */
#define FOC_POS_KP          20.0f   /* konum P [rad/s / rad] */
#define FOC_VEL_LIMIT       20.0f   /* konum modunda max hiz [rad/s] */

/* ---- Kalibrasyon ---- */
/* 1 = hizalamayi atla, asagidaki kayitli degerleri kullan (mutlak enkoder sayesinde
 * guc dongusunden sonra gecerli). Degerleri ilk acilistaki g_foc.pole_pairs, g_foc.dir,
 * g_foc.elec_offset'ten oku. */
#define FOC_USE_STORED_CALIB    0
#define FOC_CALIB_POLE_PAIRS    0
#define FOC_CALIB_DIR           1
#define FOC_CALIB_ELEC_OFFSET   0.0f   /* [rad] */

/* 0 = otomatik tespit; sifirdan farkli = bu degeri kullan, tespiti atla */
#ifndef FOC_POLE_PAIRS   /* PC testi -DFOC_POLE_PAIRS=0 ile otomatik tespiti calistirir */
#define FOC_POLE_PAIRS          7   /* 19.11 kart testinde tespit: pp_est = 7.06 */
#endif

#endif
