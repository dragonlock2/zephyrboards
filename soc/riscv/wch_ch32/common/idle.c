#include <zephyr/kernel.h>

// inspired by soc/riscv-privileged/common/idle.c

static inline void riscv_idle(unsigned int key) {
    sys_trace_idle();
    irq_unlock(key);
    // should "wfi" here but seems to cause lost interrupts, see wch_ch32/common/irq.c
}

void arch_cpu_idle(void) {
    riscv_idle(MSTATUS_IEN);
}

void arch_cpu_atomic_idle(unsigned int key) {
    riscv_idle(key);
}
