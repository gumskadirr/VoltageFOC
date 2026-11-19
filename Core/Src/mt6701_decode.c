/* MT6701 saf decode + CRC6 (donanimdan bagimsiz) */
#include "mt6701_fast.h"

/* CRC-6, poly x^6+x+1, init 0. 6 bitlik adim tablosu: T[x] = x'i 6 kez kaydir */
static const uint8_t k_crc6_tab[64] = {
    0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09, 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
    0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39, 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
    0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A, 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
    0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A, 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02
};

uint8_t MT6701F_Crc6(uint32_t bits18)
{
    /* 18 bit = 3 x 6 bit, MSB once */
    uint32_t r = k_crc6_tab[(bits18 >> 12) & 0x3FU];
    r = k_crc6_tab[(r ^ (bits18 >> 6)) & 0x3FU];
    r = k_crc6_tab[(r ^ bits18) & 0x3FU];
    return (uint8_t)r;
}

mt6701f_result_t MT6701F_Decode(const uint8_t rx[4], uint16_t *angle, uint8_t *status)
{
    uint32_t raw32 = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
                     ((uint32_t)rx[2] << 8)  |  (uint32_t)rx[3];
    uint32_t packet = (raw32 >> 7) & 0xFFFFFFU;   /* bit31 dummy */
    uint16_t a  = (uint16_t)((packet >> 10) & 0x3FFFU);
    uint8_t  mg = (uint8_t)((packet >> 6) & 0xFU);
    uint8_t  crc = (uint8_t)(packet & 0x3FU);

    if (angle)  *angle = a;     /* hata durumunda da tani icin yazilir */
    if (status) *status = mg;

    /* Tamami sifir cerceve (DO hatti hep 0: sensor yanit vermedi / hat kopuk)
     * CRC'den gecer (crc6(0)=0) ve aci=0 gibi gorunur -> hata say. Bedeli: rotor
     * tam 0 sayimindayken (1/16384) o okuma atilir, son gecerli aci kullanilir. */
    if (packet == 0U) return MT6701F_ERR_CRC;
    if (MT6701F_Crc6(packet >> 6) != crc) return MT6701F_ERR_CRC;
    if (mg & 0x8U)                         return MT6701F_ERR_LOT;
    if ((mg & 0x3U) != 0U)                 return MT6701F_OK_FIELD_WARN;
    return MT6701F_OK;
}
