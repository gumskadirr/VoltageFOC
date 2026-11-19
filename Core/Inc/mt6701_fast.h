/* MT6701 SSI hizli surucu (STM32F407, CMSIS) - FOC ISR icin */
#ifndef MT6701_FAST_H
#define MT6701_FAST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef MT6701F_HOST_TEST
/* PC testi: STM32 basligi yok, sadece decode kullanilir */
typedef struct { uint32_t dummy; } SPI_TypeDef;
typedef struct { uint32_t dummy; } GPIO_TypeDef;
#else
#include "stm32f4xx.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* SPI CR1 BR[2:0] degerleri, APB1 = 42 MHz icin SCK */
#define MT6701F_BR_DIV4   1U   /* 10.5 MHz  */
#define MT6701F_BR_DIV8   2U   /* 5.25 MHz  */
#define MT6701F_BR_DIV16  3U   /* 2.625 MHz */

/* Sinirli bekleme dongusu limiti (tick kullanilmaz).
 * 500 tur >= ~2000 CPU cevrimi (~12 us); 1 bayt: DIV4'te ~0.8 us, DIV16'da ~3 us.
 * DIV16'dan yavas bir BR kullanirsan bu limiti buyut. */
#ifndef MT6701F_SPIN_LIMIT
#define MT6701F_SPIN_LIMIT 500U
#endif
/* CSN dusus -> ilk CLK >= 100 ns : 20 NOP @168 MHz ~ 120 ns */
#define MT6701F_CS_SETUP_NOPS 20
/* Son CLK -> CSN yukselis >= 0.5 Tclk (10.5 MHz'de ~48 ns) */
#define MT6701F_CS_HOLD_NOPS  10

#define MT6701F_DEFAULT_MAX_CONSEC 3U

/* flags */
#define MT6701F_FLAG_FIELD_WARN   (1U << 0) /* Mg[1:0]=1 veya 2 (alan zayif/guclu) */
#define MT6701F_FLAG_FIELD_UNDEF  (1U << 1) /* Mg[1:0]=3 (tanimsiz) */
#define MT6701F_FLAG_BAD_SPI_MODE (1U << 2) /* CR1 CPOL!=1 veya CPHA!=0 */
#define MT6701F_FLAG_BUTTON       (1U << 3) /* Mg[2] buton */

typedef enum {
    MT6701F_OK = 0,
    MT6701F_OK_FIELD_WARN,
    MT6701F_ERR_CRC,
    MT6701F_ERR_LOT,
    MT6701F_ERR_TIMEOUT
} mt6701f_result_t;

typedef struct {
    SPI_TypeDef  *spi;
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;
    volatile uint16_t last_raw;     /* son GECERLI aci, 0..16383 */
    volatile uint8_t  last_status;  /* son gecerli Mg[3:0] */
    volatile uint8_t  flags;        /* son okumanin bayraklari */
    uint32_t reads;
    uint32_t crc_err;
    uint32_t lot_err;
    uint32_t timeout_err;
    uint32_t field_warn;
    uint8_t  consecutive_err;
    uint8_t  max_consecutive;       /* varsayilan 3 */
    volatile bool fault;            /* MT6701F_ClearFault'a kadar kalici */
    bool valid_once;                /* ilk gecerli okumadan sonra true */
} mt6701f_t;

void MT6701F_Init(mt6701f_t *e, SPI_TypeDef *spi, GPIO_TypeDef *cs_port,
                  uint16_t cs_pin, uint32_t br_bits);
mt6701f_result_t MT6701F_Read(mt6701f_t *e);
void MT6701F_ClearFault(mt6701f_t *e);

/* Saf fonksiyonlar (yan etkisiz, PC'de test edilebilir) */
mt6701f_result_t MT6701F_Decode(const uint8_t rx[4], uint16_t *angle, uint8_t *status);
uint8_t MT6701F_Crc6(uint32_t bits18);

#ifdef __cplusplus
}
#endif
#endif
