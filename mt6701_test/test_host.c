/* PC birim testi: bagimsiz referans CRC (uzun bolme) ile */
#include <stdio.h>
#include <stdint.h>
#include "mt6701_fast.h"

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { if (fails < 20) { printf("FAIL: " __VA_ARGS__); printf("\n"); } fails++; } } while (0)

/* Klasik: 18 bit + 6 sifir, 0b1000011'e bol */
static uint8_t ref_crc(uint32_t bits18)
{
    uint32_t v = bits18 << 6;           /* 24 bit */
    for (int i = 23; i >= 6; i--)
        if ((v >> i) & 1U) v ^= (0x43U << (i - 6));
    return (uint8_t)(v & 0x3FU);
}

static uint32_t build_packet(uint16_t angle, uint8_t mg)
{
    uint32_t b18 = ((uint32_t)angle << 4) | (mg & 0xFU);
    return (b18 << 6) | ref_crc(b18);
}

static void to_bytes(uint32_t packet, int dummy, uint8_t rx[4])
{
    uint32_t raw = ((uint32_t)(dummy & 1) << 31) | (packet << 7);
    rx[0] = raw >> 24; rx[1] = raw >> 16; rx[2] = raw >> 8; rx[3] = raw;
}

static mt6701f_result_t expect_for(uint8_t mg)
{
    if (mg & 8) return MT6701F_ERR_LOT;
    if (mg & 3) return MT6701F_OK_FIELD_WARN;
    return MT6701F_OK;
}

int main(void)
{
    uint8_t rx[4]; uint16_t a; uint8_t st;

    /* 1) tum acilar, Mg=0, iki dummy degeri */
    for (int d = 0; d < 2; d++)
        for (uint32_t ang = 0; ang < 16384; ang++) {
            to_bytes(build_packet((uint16_t)ang, 0), d, rx);
            mt6701f_result_t r = MT6701F_Decode(rx, &a, &st);
            if (ang == 0) { CHECK(r == MT6701F_ERR_CRC, "all-zero packet must be rejected d=%d r=%d", d, r); continue; }
            CHECK(r == MT6701F_OK && a == ang && st == 0, "angle %u d=%d r=%d a=%u", (unsigned)ang, d, r, a);
        }

    /* 2) tum Mg degerleri */
    const uint16_t angs[] = {0, 1, 0x1555, 0x2AAA, 8192, 16383};
    for (unsigned k = 0; k < sizeof angs / sizeof angs[0]; k++)
        for (uint8_t mg = 0; mg < 16; mg++)
            for (int d = 0; d < 2; d++) {
                to_bytes(build_packet(angs[k], mg), d, rx);
                mt6701f_result_t r = MT6701F_Decode(rx, &a, &st);
                if (angs[k] == 0 && mg == 0) { CHECK(r == MT6701F_ERR_CRC, "all-zero packet rejected"); continue; }
                CHECK(r == expect_for(mg) && a == angs[k] && st == mg, "mg %u ang %u r=%d", mg, angs[k], r);
            }

    /* 2b) hat hep 0 / hep 1 */
    { uint8_t z[4] = {0,0,0,0}, o[4] = {0xFF,0xFF,0xFF,0xFF};
      CHECK(MT6701F_Decode(z, &a, &st) == MT6701F_ERR_CRC, "all-zero frame rejected");
      CHECK(MT6701F_Decode(o, &a, &st) == MT6701F_ERR_CRC, "all-ones frame rejected"); }

    /* 3) tek bit cevirme: 24 paket biti, asla OK/yanlis aci olmamali */
    for (unsigned k = 0; k < sizeof angs / sizeof angs[0]; k++)
        for (int bit = 0; bit < 24; bit++) {
            uint32_t p = build_packet(angs[k], 0) ^ (1U << bit);
            to_bytes(p, 0, rx);
            mt6701f_result_t r = MT6701F_Decode(rx, &a, &st);
            CHECK(r == MT6701F_ERR_CRC, "flip bit %d ang %u r=%d", bit, angs[k], r);
        }
    for (uint32_t ang = 0; ang < 16384; ang++)
        for (int bit = 0; bit < 24; bit++) {
            to_bytes(build_packet((uint16_t)ang, 0) ^ (1U << bit), 1, rx);
            mt6701f_result_t r = MT6701F_Decode(rx, &a, &st);
            CHECK(!(r == MT6701F_OK && a != ang), "wrong-angle accepted ang %u bit %d", (unsigned)ang, bit);
            CHECK(r == MT6701F_ERR_CRC, "flip not CRC ang %u bit %d", (unsigned)ang, bit);
        }

    /* 4) tablo CRC == bit-bit orijinal == uzun bolme, 2^18 giris */
    for (uint32_t x = 0; x < (1U << 18); x++) {
        uint32_t reg = 0;
        for (int i = 17; i >= 0; i--) {
            uint8_t msb = ((reg >> 5) & 1) ^ ((x >> i) & 1);
            reg = (reg << 1) & 0x3F;
            if (msb) reg ^= 0x03;
        }
        uint8_t t = MT6701F_Crc6(x);
        CHECK(t == reg && t == ref_crc(x), "crc mismatch x=%u tab=%u bit=%u ref=%u", (unsigned)x, t, (unsigned)reg, ref_crc(x));
    }

    printf("%s: %d checks, %d failures\n", fails ? "FAIL" : "PASS", checks, fails);
    return fails ? 1 : 0;
}
