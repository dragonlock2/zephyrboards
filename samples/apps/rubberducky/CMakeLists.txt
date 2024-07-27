cmake_minimum_required(VERSION 3.20.0)

include_directories(src) # override header :)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rubberducky)

target_include_directories(app PRIVATE
    inc
)

target_sources(app PRIVATE
    src/main.cpp
    src/error.cpp
    src/hid.cpp
    src/payload.cpp
    src/rgb.cpp
)

function(add_payload PAYLOAD_NAME)
    add_llext_target(${PAYLOAD_NAME}
        OUTPUT ${ZEPHYR_BINARY_DIR}/payloads/${PAYLOAD_NAME}.elf
        SOURCES payloads/${PAYLOAD_NAME}.cpp
    )
    llext_include_directories(${PAYLOAD_NAME} inc)
    add_dependencies(app ${PAYLOAD_NAME}) # TODO try EDK
endfunction()

add_payload(example)
