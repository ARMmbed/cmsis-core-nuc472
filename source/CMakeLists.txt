# mbed Microcontroller Library
# Copyright (c) 2014 ARM Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# this custom build file pulls in different startup assembly based on the
# toolchain being used


# !!! FIXME could also use if(TARGET_LIKE_GCC) / TARGET_LIKE_ARMCC / etc set by
# the yotta targets, maybe that'd be clearer
if(${CMAKE_C_COMPILER_ID} STREQUAL GNU)
    set(STARTUP_ASM "")
    set(STARTUP_SRC "startup_NUC472_442.c")
elseif(${CMAKE_C_COMPILER_ID} STREQUAL ARMCC)
    set(STARTUP_ASM "")
    set(STARTUP_SRC "startup_NUC472_442.c")
    
    # unfortunately, for armlink we have to pass the executable startup code as
    # an object file, not as part of an archive.
    add_library(cmsis-core-nuc472_startupcod OBJECT ${STARTUP_SRC} "mbed_overrides.c")

    # This is the "neatest" way of making sure the startup code gets linked
    # into every executable. Needless to say this is, despite its brevity,
    # not neat.
    macro (add_executable _name)
        _add_executable(${ARGV} $<TARGET_OBJECTS:cmsis-core-nuc472_startupcod>)
    Endmacro()
elseif(${CMAKE_C_COMPILER_ID} STREQUAL IAR)
    set(STARTUP_ASM "")
    set(STARTUP_SRC "startup_NUC472_442.c")
else()
    set(STARTUP_ASM "")
    set(STARTUP_SRC "")
    message(ERROR "Unsupported target toolchain")
endif()

enable_language(ASM)
set_property(SOURCE ${STARTUP_ASM} PROPERTY LANGUAGE ASM)

add_library(cmsis-core-nuc472
    ${STARTUP_SRC}
    "system_NUC472_442.c"
    "StdDriver/acmp.c"
    "StdDriver/adc.c"
    "StdDriver/can.c"
    "StdDriver/cap.c"
    "StdDriver/clk.c"
    "StdDriver/crypto.c"
    "StdDriver/eadc.c"
    "StdDriver/ebi.c"
    "StdDriver/emac.c"
    "StdDriver/fmc.c"
    "StdDriver/gpio.c"
    "StdDriver/i2c.c"
    "StdDriver/i2s.c"
    "StdDriver/pdma.c"
    "StdDriver/ps2.c"
    "StdDriver/pwm.c"
    "StdDriver/rtc.c"
    "StdDriver/sc.c"
    "StdDriver/scuart.c"
    "StdDriver/sd.c"
    "StdDriver/spi.c"
    "StdDriver/sys.c"
    "StdDriver/timer.c"
    "StdDriver/uart.c"
    "StdDriver/usbd.c"
    "StdDriver/wdt.c"
    "StdDriver/wwdt.c"
)
set_target_properties(cmsis-core-nuc472 PROPERTIES
    LINKER_LANGUAGE C
)

target_link_libraries(cmsis-core-nuc472
    cmsis-core
)
