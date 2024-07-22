#include <zephyr/kernel.h>
#include <zephyr/llext/symbol.h>

extern "C" void run(void) {
    // TODO need to manually export symbols to be available (or pass in api block)
    // TODO os rgb, jiggle
    printk("hi\r\n");
}

// avoid conversion error
struct llext_symbol Z_GENERIC_SECTION(".exported_sym") __used symbol_run = {"run", (void*) &run};
