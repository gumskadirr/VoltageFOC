/* FOC icin MT6701 adaptoru (SPI2, CS=PB12) */
#include "foc_encoder_mt6701.h"

mt6701f_t g_enc;

bool FocEnc_Init(void)
{
    /* PB12 = (1U<<12): HAL bagimliligi olmadan GPIO_PIN_12 */
    MT6701F_Init(&g_enc, SPI2, GPIOB, (1U << 12), MT6701F_BR_DIV4);   /* 10.5 MHz; 19.11 testinde crc_err=0 (once gorulen hatalar kablo temassizligiydi) */
    if (g_enc.flags & MT6701F_FLAG_BAD_SPI_MODE) return false;  /* CubeMX SPI2 ayarini kontrol et */

    for (int i = 0; i < 10; i++) {          /* sinirli tekrar */
        (void)MT6701F_Read(&g_enc);
        if (g_enc.valid_once) {
            MT6701F_ClearFault(&g_enc);     /* baslangic hatalarini sil */
            return true;
        }
    }
    return false;
}

/* foc_update() (PWM ISR) basina TAM BIR kez cagrilir.
 * Hata olursa son gecerli degeri doner. FOC katmani her ISR'de
 * g_enc.fault kontrol edip true ise foc_stop() cagirmali.
 * valid_once olmadan deger anlamsizdir (FocEnc_Init false donerse surmeyin). */
uint32_t hal_encoder_raw(void)
{
    (void)MT6701F_Read(&g_enc);
    return (uint32_t)g_enc.last_raw;
}
