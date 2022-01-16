set(ZB_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "zephyrboards root directory")

zephyr_include_directories(include)

add_subdirectory(samples)
add_subdirectory(drivers)
