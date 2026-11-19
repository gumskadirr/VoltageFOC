/* MT6701 SSI hizli surucu - register seviyesi, HAL yok */
#include "mt6701_fast.h"

/* SPI2 CR1: CPOL=1, CPHA=0 (mod 2), 8-bit, soft NSS CubeMX'te ayarli olmali */

#define CS_LOW(e)   ((e)->cs_port->BSRR = ((uint32_t)(e)->cs_pin) << 16)
#define CS_HIGH(e)  ((e)->cs_port->BSRR = (uint32_t)(e)->cs_pin)


#define NOP_N(n) do { for (int _i = 0; _i < (n); _i++) { __NOP(); } } while (0)

/* Sinirli bekleme: bayrak set olunca 1, limit dolarsa 0 */
static inline bool wait_sr(SPI_TypeDef *s, uint32_t mask, bool want_set)
{
    uint32_t n = MT6701F_SPIN_LIMIT;
    while (n--) {
        bool set = (s->SR & mask) != 0U;
        if (set == want_set) return true;
    }
    return false;
}

void MT6701F_Init(mt6701f_t *e, SPI_TypeDef *spi, GPIO_TypeDef *cs_port,
                  uint16_t cs_pin, uint32_t br_bits)
{
    e->spi = spi; e->cs_port = cs_port; e->cs_pin = cs_pin;
    e->last_raw = 0; e->last_status = 0; e->flags = 0;
    e->reads = 0; e->crc_err = 0; e->lot_err = 0;
    e->timeout_err = 0; e->field_warn = 0;
    e->consecutive_err = 0;
    e->max_consecutive = MT6701F_DEFAULT_MAX_CONSEC;
    e->fault = false; e->valid_once = false;

    CS_HIGH(e);                                   /* CS bosta yuksek */
    spi->CR1 &= ~SPI_CR1_SPE;                     /* BR degisimi icin SPE=0 */
    spi->CR1 = (spi->CR1 & ~SPI_CR1_BR_Msk) |
               ((br_bits << SPI_CR1_BR_Pos) & SPI_CR1_BR_Msk);
    if (((spi->CR1 & SPI_CR1_CPOL) == 0U) || ((spi->CR1 & SPI_CR1_CPHA) != 0U) ||
        ((spi->CR1 & SPI_CR1_DFF) != 0U) || ((spi->CR1 & SPI_CR1_LSBFIRST) != 0U) ||
        ((spi->CR1 & SPI_CR1_MSTR) == 0U)) {
        e->flags |= MT6701F_FLAG_BAD_SPI_MODE;    /* mod 2, 8-bit, MSB once, master olmali */
    }
    spi->CR1 |= SPI_CR1_SPE;
}

void MT6701F_ClearFault(mt6701f_t *e)
{
    e->consecutive_err = 0;
    e->fault = false;
}

static void note_failure(mt6701f_t *e)
{
    if (e->consecutive_err < 255U) e->consecutive_err++;
    if (e->consecutive_err >= e->max_consecutive) e->fault = true;
}

mt6701f_result_t MT6701F_Read(mt6701f_t *e)
{
    SPI_TypeDef *s = e->spi;
    uint8_t rx[4];
    uint8_t uflags = (uint8_t)(e->flags & MT6701F_FLAG_BAD_SPI_MODE);
    bool ok = true;
    uint16_t angle = 0;
    uint8_t status = 0;
    mt6701f_result_t res;

    e->reads++;

    /* eski RXNE / OVR temizle */
    if (s->SR & SPI_SR_RXNE) { (void)s->DR; }
    if (s->SR & SPI_SR_OVR)  { (void)s->DR; (void)s->SR; }

    CS_LOW(e);
    NOP_N(MT6701F_CS_SETUP_NOPS);

    /* Bayt bayt: yaz, RXNE bekle. 10.5 MHz'de tum cerceve ~3 us tel suresi */
    for (int i = 0; i < 4; i++) {
        if (!wait_sr(s, SPI_SR_TXE, true)) { ok = false; break; }
        *(volatile uint8_t *)&s->DR = 0xFFU;
        if (!wait_sr(s, SPI_SR_RXNE, true)) { ok = false; break; }
        rx[i] = (uint8_t)s->DR;
    }
    if (ok && !wait_sr(s, SPI_SR_BSY, false)) ok = false;

    NOP_N(MT6701F_CS_HOLD_NOPS);
    CS_HIGH(e);

    if (!ok) {
        e->timeout_err++;
        e->flags = uflags;
        note_failure(e);
        return MT6701F_ERR_TIMEOUT;
    }

    res = MT6701F_Decode(rx, &angle, &status);

    if (status & 0x4U) uflags |= MT6701F_FLAG_BUTTON;
    if (res == MT6701F_OK_FIELD_WARN) {
        e->field_warn++;
        uflags |= ((status & 0x3U) == 3U) ? MT6701F_FLAG_FIELD_UNDEF
                                          : MT6701F_FLAG_FIELD_WARN;
    }
    e->flags = uflags;

    if (res == MT6701F_ERR_CRC) { e->crc_err++; note_failure(e); return res; }
    if (res == MT6701F_ERR_LOT) { e->lot_err++; note_failure(e); return res; }

    e->last_raw = angle;
    e->last_status = status;
    e->consecutive_err = 0;
    e->valid_once = true;
    return res;
}
