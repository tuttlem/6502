cmake_minimum_required(VERSION 3.25)
project(6502 C)

set(CMAKE_C_STANDARD 11)

add_executable(6502 src/main.c src/vm/cpu.c src/vm/bus.c src/vm/memory.c src/vm/system.c)
