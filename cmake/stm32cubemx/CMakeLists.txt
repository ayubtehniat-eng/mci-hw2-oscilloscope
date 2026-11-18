cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

# Symbols (macros): device selection only, no HAL
set(MX_Defines_Syms
    STM32F303xC
    $<$<CONFIG:Debug>:DEBUG>
)

# Include paths: our own headers plus the CMSIS headers only
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F3xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
)

# Application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/clock.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/systick.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/tim2.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f303xc.s
)

# CMSIS system file only (no HAL drivers)
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32f3xx.c
)

# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers object library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

# Add application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link the system object library and toolchain libraries
target_link_libraries(${CMAKE_PROJECT_NAME} STM32_Drivers ${TOOLCHAIN_LINK_LIBRARIES})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)
