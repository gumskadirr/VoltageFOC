#ifndef STUB_STM32F4XX_H
#define STUB_STM32F4XX_H
#include <stdint.h>
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __set_PRIMASK(uint32_t v) { (void)v; }
#endif
