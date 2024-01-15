#ifndef SYSTEM_CH32X035_H
#define SYSTEM_CH32X035_H

#ifndef ZEPHYR_INCLUDE_ARCH_RISCV_IRQ_H_

#include <zephyr/kernel.h>

static inline void Delay_Init(void) {}
static inline void Delay_Us(uint32_t us) { k_busy_wait(us); }

#endif // ZEPHYR_INCLUDE_ARCH_RISCV_IRQ_H_

#endif // SYSTEM_CH32X035_H
