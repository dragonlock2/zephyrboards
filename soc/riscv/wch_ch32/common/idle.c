#include <zephyr/kernel.h>

// inspired by soc/riscv-privileged/common/idle.c

static inline void riscv_idle(unsigned int key) {
    sys_trace_idle();
    irq_unlock(key);
    __asm__ volatile ("wfi"); // technically not atomic
}

void arch_cpu_idle(void) {
    riscv_idle(MSTATUS_IEN);
}

void arch_cpu_atomic_idle(unsigned int key) {
    riscv_idle(key);
}
