/* foc_board.c — STM32F407 kart katmani (register seviyesi, CMSIS) */
#include "stm32f4xx.h"
#include "foc_board.h"

extern void HAL_Delay(uint32_t);

/* Merkez hizali modda periyot = 2*ARR timer clock: 168e6/(2*4200) = 20 kHz */
#define FOC_PWM_ARR 4200U

volatile uint32_t g_isr_cycles_last = 0;
volatile uint32_t g_isr_cycles_max  = 0;
volatile uint32_t g_isr_count       = 0;

static void (* volatile s_isr_cb)(void) = 0;

void FocBoard_Init(void)
{
    /* Saatleri ac, geri oku (errata gecikmesi) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    /* PE7 (EN): once LOW yaz, sonra cikis yap */
    GPIOE->BSRR = GPIO_BSRR_BR7;
    GPIOE->OTYPER  &= ~GPIO_OTYPER_OT7;
    GPIOE->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7;
    GPIOE->PUPDR   &= ~GPIO_PUPDR_PUPD7;
    GPIOE->MODER   = (GPIOE->MODER & ~GPIO_MODER_MODER7) | GPIO_MODER_MODER7_0;

    /* PE9/11/13: AF1 (TIM1_CH1..3), push-pull, very high speed, pull yok */
    const uint32_t pins[3] = {9U, 11U, 13U};
    for (int i = 0; i < 3; i++) {
        uint32_t p = pins[i];
        GPIOE->AFR[1]  = (GPIOE->AFR[1] & ~(0xFUL << ((p - 8U) * 4U))) | (1UL << ((p - 8U) * 4U));
        GPIOE->OTYPER &= ~(1UL << p);
        GPIOE->OSPEEDR |= (3UL << (p * 2U));
        GPIOE->PUPDR  &= ~(3UL << (p * 2U));
        GPIOE->MODER   = (GPIOE->MODER & ~(3UL << (p * 2U))) | (2UL << (p * 2U));
    }

    /* TIM1: once sayaci durdur */
    TIM1->CR1  = 0;
    TIM1->PSC  = 0;
    TIM1->ARR  = FOC_PWM_ARR;
    TIM1->RCR  = 1;                       /* update 2 yarim periyotta bir = PWM periyodunda bir */
    TIM1->CR1  = TIM_CR1_CMS_0 | TIM_CR1_ARPE;   /* merkez hizali mod 1, ARR preload */

    /* PWM mod 1 (OCxM=110) + preload */
    TIM1->CCMR1 = (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE)
                | (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE);
    TIM1->CCMR2 = (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE);
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    /* CC1E..3E acik, polarite aktif yuksek, CCxNE = 0 */
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    /* MOE=1, dead-time 0, OSSR/OSSI 0, break yok (surucu IC halleder) */
    TIM1->BDTR  = TIM_BDTR_MOE;
    TIM1->DIER  = 0;                      /* UIE kapali */

    /* Preload'lari yukle, bayragi temizle, sayaci baslat */
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR  = 0;
    TIM1->CR1 |= TIM_CR1_CEN;

    /* NVIC: onceligi ayarla, ENABLE ETME */
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0);

    /* DWT cevrim sayaci */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void FocBoard_StartIsr(void (*cb)(void))
{
    s_isr_cb = cb;
    TIM1->SR = ~(uint32_t)TIM_SR_UIF;               /* yalnizca UIF'i temizle */
    TIM1->DIER |= TIM_DIER_UIE;
    NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

void FocBoard_StopIsr(void)
{
    TIM1->DIER &= ~TIM_DIER_UIE;
    NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
}

/* Not: TIM10 ayni vektoru paylasir; kullanmiyoruz, bayragi sadece TIM1 icin isleriz. */
void TIM1_UP_TIM10_IRQHandler(void)
{
    uint32_t t0 = DWT->CYCCNT;

    if (TIM1->SR & TIM_SR_UIF) {
        TIM1->SR = ~(uint32_t)TIM_SR_UIF;           /* write-0-to-clear, RMW yok */
        void (*cb)(void) = s_isr_cb;
        if (cb) cb();
    }

    uint32_t dt = DWT->CYCCNT - t0;       /* 32-bit tasma guvenli */
    g_isr_cycles_last = dt;
    if (dt > g_isr_cycles_max) g_isr_cycles_max = dt;
    g_isr_count++;
}

/* Preload nedeniyle yeni CCR degerleri bir sonraki update olayinda etkili olur. */
void hal_pwm_set(float da, float db, float dc)
{
    float d[3] = {da, db, dc};
    uint32_t c[3];
    for (int i = 0; i < 3; i++) {
        float x = d[i];
        if (!(x > 0.0f)) x = 0.0f;        /* NaN de 0 olur */
        if (x > 1.0f) x = 1.0f;
        c[i] = (uint32_t)(x * (float)FOC_PWM_ARR + 0.5f);
    }
    TIM1->CCR1 = c[0];
    TIM1->CCR2 = c[1];
    TIM1->CCR3 = c[2];
}

void hal_pwm_enable(int on)
{
    if (on) {
        GPIOE->BSRR = GPIO_BSRR_BS7;
    } else {
        TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
        GPIOE->BSRR = GPIO_BSRR_BR7;
    }
}

/* Sadece thread baglaminda cagrilmali (HAL_Delay SysTick'e bagli, ISR'da kilitlenir). */
void hal_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
