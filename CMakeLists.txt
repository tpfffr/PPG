cmake_minimum_required(VERSION 3.20.0)

# 1. Tell Zephyr about the custom YAML binding we made
list(APPEND dts_root ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(PPG)

# 2. Compile all the driver files
target_sources(app PRIVATE
    src/main.c
    src/ble.c
    src/max32664c.c
    src/max32664c_init.c
    src/max32664c_worker.c
    src/max32664c_interrupt.c
    src/max32664c_acc.c
    src/max32664c_bl.c
)
