#include <zephyr/kernel.h>

extern void __reset(void);

__attribute__((naked, section(".vectors.reset")))
void __start(void) {
    __asm__ (
        ".option push             \n"
        ".option norelax          \n"
        "la gp, __global_pointer$ \n" // set gp
        ".option pop              \n"
        : : : "gp"
    );

#if CONFIG_SOC_CH32X035
    __asm__ (
        "li t0, 0x1f       \n"
        "csrw 0xbc0, t0    \n" // from startup, exact effect unknown
        "li t0, 0x88       \n"
        "csrw mstatus, t0  \n" // enable MPIE/MIE
        "li t0, 0x0        \n"
        "csrw 0x804, t0    \n" // disable interrupt nesting and hardware stacking (INTSYSCR)
        : : : "t0"
    );
#else
    #error "define cpu specific settings for soc!"
#endif

    __asm__ (
        "mv t0, %0         \n"
        "li t1, 0xfffffffc \n"
        "and t0, t0, t1    \n"
        "ori t0, t0, 0x2   \n"
        "csrw mtvec, t0    \n"
        : : "r" (&_isr_wrapper) : "t0", "t1"
    );

    __reset();
    while (1);
}
