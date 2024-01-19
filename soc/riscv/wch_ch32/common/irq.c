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
 * RISC-V port does a context switch w/o mret (unlike FreeRTOS), so we must do a manual
 * mret to exit the interrupt level before that. The extra mret from when the ISR actually
 * returns appears to not cause issues. Since __soc_handle_irq is called before our installed
 * handlers which would clear the interrupt source, doing an mret there would cause a retrigger
 * of the interrupt. Thus, we must do our mret workaround after calling our handler but before
 * the context switch.
 * 
 * To avoid patches, the cleanest place to do this is in the call sys_trace_isr_exit immediately
 * after our handler is called. However, this does mean we can't use the trace subsystem.
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

// TODO make __soc_handle_irq work? clear interrupt level w/o mret?
