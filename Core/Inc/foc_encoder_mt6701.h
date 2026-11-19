#ifndef FOC_ENCODER_MT6701_H
#define FOC_ENCODER_MT6701_H
#include <stdbool.h>
#include <stdint.h>
#include "mt6701_fast.h"

extern mt6701f_t g_enc;

/* Sensoru baslat; ilk gecerli okuma icin en fazla 10 deneme. false = sensor yok */
bool FocEnc_Init(void);

/* FOC arayuzu: 0 .. 16383 */
uint32_t hal_encoder_raw(void);

#endif
