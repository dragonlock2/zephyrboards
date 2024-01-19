#include <zephyr/kernel.h>

void arch_irq_enable(unsigned int irq) {
    NVIC_EnableIRQ(irq);
}

void arch_irq_disable(unsigned int irq) {
    NVIC_DisableIRQ(irq);
}

void __soc_handle_irq(unsigned long mcause) {
    (void) mcause; // do nothing
}

__attribute__((naked))
bool __soc_is_irq(void) {
    __asm__ (
        "csrr a0, mcause \n"
        "srli a0, a0, 31 \n"
        "ret             \n"
    );
}

/*
 * CH32 is not RISC-V compliant for interrupt handling since it likely pulls some behavior from
 * ARM. Unlike normal RISC-V, CH32 relies on mret to exit the interrupt context. Since Zephyr does
 * a context switch inside _isr_wrapper by simply jumping, the next thread remains in an interrupt
 * context, effectively disabling same or lower priority interrupts until the next context switch.
 * Normal RISC-V lacks this issue as the context switch will re-enable interrupts as needed. To
 * work around this issue, we need to mret before context switching.
 * 
 * It should be noted that FreeRTOS does its context switching with the mret, so this issue doesn't
 * exist for it. However, it doesn't support interrupt nesting that Zephyr's method would allow.
 * 
 * Like ARM, clearing an interrupt is decentralized and needs access to the source peripheral to prevent
 * a retrigger. This means we can't add our mret to __soc_handle_irq which is called before our interrupt
 * handler which would clear the IRQ. To avoid patches, the cleanest place to do the mret is instead in
 * sys_trace_isr_exit. However, this does mean we can't use the trace subsystem.
 */
__attribute__((naked))
void sys_trace_isr_exit_user(void) {
    __asm__ (
        "csrw mepc, ra \n"
        "mret          \n"
    );
}

#if !(defined(CONFIG_TRACING) && defined(CONFIG_TRACING_USER) && defined(CONFIG_TRACING_ISR))
#error "please don't use tracing subsytem"
#endif
