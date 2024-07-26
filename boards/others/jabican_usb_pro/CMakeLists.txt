cmake_minimum_required(VERSION 3.20.0)

# patches to account for differences of MK22F1M0AVLH12
add_custom_target(
    apply_zephyr_patch
    COMMAND git apply ${BOARD_DIR}/zephyr.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
)
add_custom_target(
    apply_hal_nxp_patch
    COMMAND git apply ${BOARD_DIR}/hal_nxp.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}/../modules/hal/nxp
)
add_dependencies(version_h apply_zephyr_patch)
add_dependencies(version_h apply_hal_nxp_patch)

add_custom_target(
    remove_zephyr_patch ALL
    COMMAND git apply -R ${BOARD_DIR}/zephyr.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}
    DEPENDS zephyr_final
)
add_custom_target(
    remove_hal_nxp_patch ALL
    COMMAND git apply -R ${BOARD_DIR}/hal_nxp.patch
    WORKING_DIRECTORY $ENV{ZEPHYR_BASE}/../modules/hal/nxp
    DEPENDS zephyr_final
)
