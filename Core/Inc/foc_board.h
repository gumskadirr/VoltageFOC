/* foc_board.h — STM32F407 kart katmanı (TIM1 PWM, EN pini, FOC kesmesi)
 *
 * Donanım:
 *   PE9  = TIM1_CH1 (Faz A PWM, AF1)
 *   PE11 = TIM1_CH2 (Faz B PWM, AF1)
 *   PE13 = TIM1_CH3 (Faz C PWM, AF1)
 *   PE7  = EN (sürücü enable, GPIO çıkış, aktif yüksek)
 *   TIM1 saati 168 MHz (APB2 84 MHz x2), merkez hizalı mod 1, ARR = 4200 -> 20 kHz
 *
 * CubeMX'te TIM1 ve PE7/PE9/PE11/PE13 AÇILMAMALI (bu dosya registerlarla kurar).
 * TIM1_UP_TIM10_IRQHandler bu modülde tanımlıdır.
 */
#ifndef FOC_BOARD_H
#define FOC_BOARD_H

#include <stdint.h>

#define FOC_PWM_FREQ_HZ   20000U
#define FOC_PWM_DT_S      (1.0f / (float)FOC_PWM_FREQ_HZ)

/* GPIO + TIM1'i kurar, sayacı başlatır. Çıkışlar %50 duty değil, 0 duty; EN = düşük.
 * Update kesmesi KAPALI başlar (kalibrasyon thread bağlamında yapılır). */
void FocBoard_Init(void);

/* Her PWM periyodunda bir kez (20 kHz) çağrılacak fonksiyonu ayarlar ve kesmeyi açar. */
void FocBoard_StartIsr(void (*cb)(void));
void FocBoard_StopIsr(void);

/* ISR süre ölçümü (DWT CYCCNT), çevrim cinsinden; g_isr_cycles_max = en kötü durum. */
extern volatile uint32_t g_isr_cycles_last;
extern volatile uint32_t g_isr_cycles_max;
extern volatile uint32_t g_isr_count;

/* foc_voltage.h'nin istediği HAL fonksiyonları (bu modül sağlar): */
/* void     hal_pwm_set(float da, float db, float dc);  0..1 duty            */
/* void     hal_pwm_enable(int on);                      EN pini + çıkışlar  */
/* void     hal_delay_ms(uint32_t ms);                   HAL_Delay           */
/* (hal_encoder_raw() foc_encoder_mt6701.c'de)                              */

#endif
