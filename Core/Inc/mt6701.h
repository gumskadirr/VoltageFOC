#ifndef MT6701_H
#define MT6701_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Manyetik Alan Durum Tanımları
typedef enum {
    MT6701_MAG_NORMAL = 0,
    MT6701_MAG_TOO_STRONG = 1,
    MT6701_MAG_TOO_WEAK = 2,
    MT6701_LOSS_OF_TRACK = 3
} mt6701_mag_status_t;

// Sürücü Yapılandırma Yapısı
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} mt6701_t;

// Okunan Veri Yapısı
typedef struct {
    uint16_t raw_angle;         // 14-bit ham açı verisi (0 - 16383)
    float angle_degrees;        // Hesaplanan açı (0.0° - 360.0°)
    mt6701_mag_status_t status; // Manyetik alan durumu
    bool push_button;           // Buton basılma durumu (Mg[2])
    bool loss_of_track;         // İzleme kaybı (Mg[3])
    bool crc_valid;             // CRC-6 doğrulama sonucu
} mt6701_data_t;

// Fonksiyon Prototipleri
bool MT6701_Init(mt6701_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
bool MT6701_ReadData(mt6701_t *dev, mt6701_data_t *data);

#endif
