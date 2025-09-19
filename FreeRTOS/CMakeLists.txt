cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(RTOS_Defines_Syms

        $<$<CONFIG:Debug>:DEBUG>
)

set(FreeRTOS_Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F
)

file(GLOB FreeRTOS_Src
        ${CMAKE_CURRENT_SOURCE_DIR}/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/portable/MemMang/heap_4.c
)


add_library(FreeRTOS STATIC)
target_sources(FreeRTOS PRIVATE ${FreeRTOS_Src})
target_include_directories(FreeRTOS PUBLIC ${FreeRTOS_Inc})
target_compile_definitions(FreeRTOS PUBLIC ${RTOS_Defines_Syms})

target_link_libraries(FreeRTOS PUBLIC STM32_Drivers)