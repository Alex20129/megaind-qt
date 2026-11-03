cmake_minimum_required(VERSION 3.20)
project(MegaIndQt VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core)

add_library(megaind_core STATIC
	src/core/comm.c
	src/core/opto.c
	src/core/dout.c
	src/core/rs485.c
	src/core/rtc.c
	src/core/wdt.c
	src/core/thread.c
	src/core/megaind_core.c
)

set_property(TARGET megaind_core PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET megaind_core PROPERTY AUTOMOC OFF)
target_compile_definitions(megaind_core PRIVATE NO_CLI)
target_link_libraries(megaind_core pthread m)

add_library(MegaIndQt STATIC
	src/megaind_device.cpp
	include/megaind/megaind_device.hpp
)

set_property(TARGET MegaIndQt PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(MegaIndQt
	PUBLIC include
	PRIVATE src/core
)

target_link_libraries(MegaIndQt
	PUBLIC Qt6::Core
	PRIVATE megaind_core
)
