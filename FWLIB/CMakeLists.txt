cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)

set(STD_Lib_Defines_Syms
		STM32F40_41xxx
		USE_STDPERIPH_DRIVER
		__FPU_USED
		__FPU_PRESENT
)



set(STD_Lib_Include_Dirs
		${CMAKE_CURRENT_SOURCE_DIR}/inc/
		${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/
)

file(GLOB STD_Lib_Drivers_Src
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/*.c
)

set(STD_Lib_Drivers_Src ${STD_Lib_Drivers_Src}
		${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f407xx.s
)

# Create STM32_Drivers static library
add_library(STM32_Drivers STATIC)
target_sources(STM32_Drivers PUBLIC ${STD_Lib_Drivers_Src})
target_compile_definitions(STM32_Drivers PUBLIC ${STD_Lib_Defines_Syms})
target_include_directories(STM32_Drivers PUBLIC ${STD_Lib_Include_Dirs})
# target_link_libraries(STM32_Drivers PUBLIC stm32stdlib)


# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
