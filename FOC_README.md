# FOC (akım sensörsüz, MT6701 enkoderli) — kullanım

## Dosyalar
| Dosya | Görev |
|---|---|
| `Core/Src/foc_app.c` + `Core/Inc/foc_app.h` | Açılış sırası, komutlar, Ke ölçümü, hata yönetimi |
| `Core/Inc/foc_config.h` | **Tüm ayarlar** (Vbus, R, limitler, kazançlar, kalibrasyon) |
| `Core/Src/foc_voltage.c/.h` | FOC çekirdeği (SVPWM, hizalama, kutup çifti tespiti, hız/konum) |
| `Core/Src/foc_board.c/.h` | TIM1 PWM (PE9/PE11/PE13, 20 kHz), EN (PE7), 20 kHz kesme |
| `Core/Src/mt6701_fast.c`, `mt6701_decode.c`, `foc_encoder_mt6701.c` | MT6701 hızlı SSI okuma (SPI2, 10.5 MHz) |
| `foc_test/` | FOC çekirdeği ve uygulama katmanı PC simülasyonları (firmware'e derlenmez) |
| `mt6701_test/` | Enkoder çözümleme birim testi (firmware'e derlenmez) |
| `docs/` | Teknik doküman (Markdown ve PDF) |

## Bağlantı
- PE9 → sürücü IN1 (A), PE11 → IN2 (B), PE13 → IN3 (C), GND ortak.
- PE7 → sürücünün EN girişi (varsa). EN bağlı değilse motor durduğu her durumda üç faz toprağa çekilir ve motor elle çevrilmeye direnç gösterir.
- DRV8313 gibi entegrelerde nSLEEP ve nRESET yüksekte olmalı.
- MT6701: PB13 CLK, PB14 DO, PB12 CSN, 3,3 V ve GND. PB15 (MOSI) boş kalır.
- TIM1, PE7 ve PB12 `.ioc` dosyasında tanımlı; **TIM1 kesmesini (NVIC) CubeMX'te açma**, kesme fonksiyonu `foc_board.c`'de.

## İlk çalıştırma
1. Gimbal yükünü sök, motoru serbest bırak. Mümkünse akım sınırlı güç kaynağı (≈0,5 A).
2. Derle, debug ile çalıştır. Açılışta motor ~3,5 s kısa bir hizalama hareketi yapar (`FOC_POLE_PAIRS` = 0 ise kutup çifti tespitiyle ~21 s).
3. Live Expressions'a ekle: `g_app_state`, `g_detected_pp`, `g_foc`, `g_enc`, `g_isr_cycles_max`, `g_cmd`.
4. `g_app_state` = `FOC_APP_RUNNING` olmalı; motor bulunduğu yerde tutar.
   - `ENC_FAIL`: enkoder okunamadı / SPI ayarı yanlış. `PP_FAIL`: motor dönmedi veya kutup çifti belirsiz (`g_foc.pp_est`'e bak). `ALIGN_FAIL`: hizalama başarısız (`g_foc.pp_check` 1,0'a yakın olmalı).
5. `g_isr_cycles_max` / 168 = µs cinsinden en kötü ISR süresi; 50 µs'nin belirgin şekilde altında olmalı.
6. `g_enc.crc_err` sıfıra yakın kalmalı; artıyorsa enkoder kablolarını kontrol et.

## Komut verme (Live Expressions'tan yaz)
- Konum: `g_cmd.mode = 3`, `g_cmd.target = <rad>`, `g_cmd.apply = 1`
- Hız: `mode = 2`, `target = <rad/s>`; Voltaj: `mode = 0`, `target = <V>`
- Durdur: `g_cmd.stop = 1` (apply ile devam). Hata temizle: `g_cmd.clear_fault = 1`.
- Ke ölç: `g_cmd.measure_ke = 1` (motor ~4 s serbest döner!) → `g_measured_ke`'yi `FOC_KE`'ye yaz.

## Kalibrasyonu kaydet (açılışta motoru döndürmemek için)
İlk açılıştan sonra `g_foc.pole_pairs`, `g_foc.dir`, `g_foc.elec_offset` değerlerini oku, `foc_config.h`'de
`FOC_USE_STORED_CALIB 1` yapıp bu üç değeri yaz. Enkoder mutlak olduğu için güç döngüsünden sonra da geçerlidir
(mıknatıs veya enkoder yerinden oynarsa tekrar kalibre et).

## PC testleri (isteğe bağlı, Linux/WSL gcc)
```
cd mt6701_test      && ./build.sh
cd foc_test/sim     && gcc -O2 -I../../Core/Inc sim.c ../../Core/Src/foc_voltage.c -lm -o sim && ./sim
cd foc_test/sim_app && gcc -O2 -DMT6701F_HOST_TEST -Isim_app_stub -I../../Core/Inc -DFOC_POLE_PAIRS=0 harness.c ../../Core/Src/foc_app.c ../../Core/Src/foc_voltage.c -lm -o harness && ./harness
```
