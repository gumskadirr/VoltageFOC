# VoltageFOC

STM32F407 üzerinde, akım sensörü olmadan, MT6701 14 bit mutlak manyetik enkoderle gimbal BLDC motoru süren **voltaj modlu FOC** yazılımı.

- 20 kHz PWM ve FOC kesmesi (TIM1, merkez hizalı, SVPWM), 1 kHz hız/konum kontrolü
- MT6701 SSI okuma: SPI2 register seviyesinde, 10,5 MHz, CRC-6 ve durum bitleri kontrolü
- Sensörsüz aşırı akım koruması için sanal akım limiti (Uq, geri EMK etrafında sınırlanır)
- Açılışta otomatik kutup çifti tespiti ve hizalama; mutlak enkoder sayesinde kalibrasyon kaydedilebilir
- Konum, hız, voltaj ve tahmini akım modları; iki noktalı Ke ölçümü
- PC'de çalışan birim testleri ve motor simülasyonu

## Donanım
| | |
|---|---|
| MCU | STM32F407VG, 168 MHz |
| Enkoder | MT6701 (SSI), PB12 CSN / PB13 CLK / PB14 DO |
| Sürücü | 3PWM girişli sürücü, PE9 / PE11 / PE13 (TIM1 CH1–3), EN: PE7 (isteğe bağlı) |
| Besleme | 12 V |

## Derleme
STM32CubeIDE ile **File → Import → Existing Projects into Workspace** üzerinden içe aktarılıp derlenir. Proje adı ve klasör adı `VoltageFOC` olmalıdır.

## Dokümantasyon
- [Kullanım: bağlantı, ilk çalıştırma, komutlar](FOC_README.md)
- [Teknik doküman (Markdown)](docs/TEKNIK_DOKUMAN.md) · [PDF](docs/VoltageFOC_Teknik_Dokuman.pdf)
- MT6701 datasheet: üreticinin (MagnTek) sitesinden indirilebilir; depoya eklenmedi.

## Lisanslar
`Drivers/` altındaki STM32 HAL ve CMSIS dosyaları kendi lisanslarıyla (STMicroelectronics, Arm) gelir; ilgili `LICENSE.txt` dosyalarına bakın.
