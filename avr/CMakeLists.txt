cmake_minimum_required(VERSION 3.10)
project(truetape64 C)

SET(CMAKE_C_STANDARD 99)
SET(MCU "attiny2313")
SET(F_CPU "16000000")
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_C_COMPILER /usr/bin/avr-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/avr-g++)
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mmcu=${MCU} -DF_CPU=${F_CPU} -Os -I. -ffunction-sections -Wl,--gc-sections -Wl,--relax -std=c99")

add_executable(truetape64 main.c main.h)