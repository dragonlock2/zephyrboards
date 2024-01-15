# TODO move to hal_wch
set(WCH_SDK_DIR ${ZEPHYR_BASE}/../modules/hal/wch/${SOC_SERIES}/EVT/EXAM/SRC)
set_property(GLOBAL PROPERTY CSTD gnu17) # support asm

file(GLOB WCH_SDK_SRCS
    ${WCH_SDK_DIR}/Peripheral/src/*.c
    ${WCH_SDK_DIR}/Core/*.c
)

zephyr_sources(
    idle.c
    irq.c
    startup.c
    timer.c
    ${WCH_SDK_SRCS}
)

zephyr_include_directories(
    .
    ${WCH_SDK_DIR}/Peripheral/inc
    ${WCH_SDK_DIR}/Core
)
