#include <mt6701.h>

/**
 * @brief MT6701 CRC-6 Hesaplama Fonksiyonu ($X^6 + X + 1$ Polinomu)
 * @param bitstream: D[13:0] ve Mg[3:0] içeren 18-bitlik veri paketi
 * @return Hesaplanan 6-bitlik CRC değeri
 */
static uint8_t mt6701_calculate_crc6(uint32_t bitstream) {
    uint32_t reg = 0;
    // 18 bitlik veri işlenir (MSB'den LSB'ye)[cite: 25]
    for (int i = 17; i >= 0; i--) {
        // Gelen bit register MSB'si ile XOR'lanır (augmented olmayan CRC; eski hali 6 sıfır bit eklemeden yanlış sonuç veriyordu)
        uint8_t msb = ((reg >> 5) & 1) ^ ((bitstream >> i) & 1);
        reg = (reg << 1) & 0x3F;
        if (msb) {
            reg ^= 0x03; // Polinom: x^6 + x + 1 (0x03 karşılığı)
        }
    }
    return (uint8_t)(reg & 0x3F);
}

bool MT6701_Init(mt6701_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    if (!dev || !hspi) return false;

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;

    // CSN pinini başlangıçta High (Pasif) yap
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    return true;
}

bool MT6701_ReadData(mt6701_t *dev, mt6701_data_t *data) {
    // İlk düşen kenarda DO henüz geçerli değil (1 boş bit) → 24 bit veri için 25 clock gerekir, 4 byte okunur
    uint8_t tx_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx_data[4] = {0};

    // 1. Adım: CSN pinini Low çekerek SSI transferini başlat[cite: 24, 25]
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    // Min 100ns bekleme (TL parametresi)[cite: 24]
    // Not: HAL fonksiyonunun overhead süresi bu gecikmeyi fazlasıyla karşılar.

    // 2. Adım: 24-bit veriyi SPI üzerinden al (3 Byte)
    if (HAL_SPI_TransmitReceive(dev->hspi, tx_data, rx_data, 4, 100) != HAL_OK) {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
        return false;
    }

    // 3. Adım: CSN pinini High çekerek transferi sonlandır[cite: 24, 25]
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    // 4. Adım: 32 bit = [31] boş bit, [30:7] 24-bit paket (D13..D0, Mg3..Mg0, CRC5..CRC0), [6:0] artık
    uint32_t raw32 = ((uint32_t)rx_data[0] << 24) | ((uint32_t)rx_data[1] << 16) |
                     ((uint32_t)rx_data[2] << 8)  | rx_data[3];
    uint32_t raw_packet = (raw32 >> 7) & 0xFFFFFF;

    // Alanları Ayıkla
    uint16_t angle_raw = (raw_packet >> 10) & 0x3FFF; // İlk 14 bit (D13 - D0)[cite: 25]
    uint8_t status_raw  = (raw_packet >> 6) & 0x0F;   // Sonraki 4 bit (Mg3 - Mg0)[cite: 25]
    uint8_t rx_crc      = raw_packet & 0x3F;          // Son 6 bit (CRC5 - CRC0)[cite: 25]

    // 5. Adım: Açı Hesaplama Formülü: \theta = (D / 16384) * 360°[cite: 25]
    data->raw_angle = angle_raw;
    data->angle_degrees = ((float)angle_raw / 16384.0f) * 360.0f;

    // 6. Adım: Manyetik Durum Analizi (Mg[3:0])
    uint8_t mg_field = status_raw & 0x03; // Mg[1:0]
    if (mg_field == 0)      data->status = MT6701_MAG_NORMAL;
    else if (mg_field == 1) data->status = MT6701_MAG_TOO_STRONG; // Mıknatıs çok yakın[cite: 9, 25]
    else                    data->status = MT6701_MAG_TOO_WEAK;   // Mıknatıs çok uzak[cite: 9, 25]

    data->push_button   = (status_raw >> 2) & 0x01; // Mg[2]: Buton basıldı mı?[cite: 25]
    data->loss_of_track = (status_raw >> 3) & 0x01; // Mg[3]: İzleme kaybı var mı?[cite: 25]
    if (data->loss_of_track) {
    	data->status = MT6701_LOSS_OF_TRACK;    }

    // 7. Adım: CRC-6 Doğrulama (D[13:0] ve Mg[3:0] toplam 18 bit)[cite: 25]
    uint32_t crc_payload = (raw_packet >> 6) & 0x3FFFF;
    uint8_t calc_crc = mt6701_calculate_crc6(crc_payload);

    data->crc_valid = (calc_crc == rx_crc);

    return true;
}
