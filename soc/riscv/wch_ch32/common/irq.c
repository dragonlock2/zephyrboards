#include <zephyr/kernel.h>

void __soc_handle_irq(unsigned long mcause) {
    (void) mcause; // do nothing
}

__attribute__((naked))
bool __soc_is_irq(void) {
    __asm__ (
        "csrr t0, mcause          \n"
        "li a0, 0                 \n"
        "bge t0, x0, is_exception \n"
        "li a0, 1                 \n"
    "is_exception:                \n"
        "ret                      \n"
    );
}
