cmake_minimum_required(VERSION 3.10)


project(
    SOEM
    LANGUAGES C
)


# ============================================================
# Include paths
# ============================================================

include_directories(
    osal
)

include_directories(
    osal/linux
)

include_directories(
    oshw
)

include_directories(
    oshw/linux
)

include_directories(
    soem
)



# ============================================================
# Build SOEM static library
# ============================================================

add_library(
    soem

    STATIC

    osal/linux/osal.c

    oshw/linux/nicdrv.c

    oshw/linux/oshw.c


    soem/ethercatbase.c

    soem/ethercatcoe.c

    soem/ethercatconfig.c

    soem/ethercatdc.c

    soem/ethercatfoe.c

    soem/ethercatmain.c

    soem/ethercatprint.c

    soem/ethercatsoe.c

    soem/ethercateoe.c
)



# ============================================================
# Public include
# ============================================================

target_include_directories(
    soem

    PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}

    ${CMAKE_CURRENT_SOURCE_DIR}/osal

    ${CMAKE_CURRENT_SOURCE_DIR}/osal/linux

    ${CMAKE_CURRENT_SOURCE_DIR}/oshw

    ${CMAKE_CURRENT_SOURCE_DIR}/oshw/linux

    ${CMAKE_CURRENT_SOURCE_DIR}/soem
)



# ============================================================
# Linux dependencies
# ============================================================

target_link_libraries(
    soem

    PUBLIC

    pthread

    rt
)
