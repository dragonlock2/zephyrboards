set(ZB_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "zephyrboards root directory")

zephyr_include_directories(include)

list(APPEND SYSCALL_INCLUDE_DIRS ${ZB_DIR}/include/zephyrboards/drivers)
set(SYSCALL_INCLUDE_DIRS ${SYSCALL_INCLUDE_DIRS} PARENT_SCOPE)

add_subdirectory(samples)
add_subdirectory(drivers)
