


# specify cross compilers and tools
SET(CMAKE_C_COMPILER_WORKS   1)	# 省略编译器检查
SET(CMAKE_CXX_COMPILER_WORKS 1)	# 省略编译器检查

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

########################################################################################
#                                        编译选项                                        #
########################################################################################
SET(CMAKE_C_FLAGS_DEBUG   "-DDEBUG")
SET(CMAKE_C_FLAGS_RELEASE "-DRELEASE")

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

#请在此处添加头文件路径
include_directories(
    ${PROJECT_SOURCE_DIR}/Drivers/Mcu/GD32F303/GD32F30x_Firmware_Library_V220/CMSIS
    ${PROJECT_SOURCE_DIR}/Drivers/Mcu/GD32F303/GD32F30x_Firmware_Library_V220/CMSIS/GD/GD32F30x/Include
    ${PROJECT_SOURCE_DIR}/Drivers/Mcu/GD32F303/GD32F30x_Firmware_Library_V220/GD32F30x_standard_peripheral/Include
    ${PROJECT_SOURCE_DIR}/Drivers/Mcu/GD32F303/Inc
    ${PROJECT_SOURCE_DIR}/UserApp/Inc

)

ADD_DEFINITIONS(
    -DGD32F30X_HD
    -DUSE_STDPERIPH_DRIVER
)
#请在此处此处添加.c文件目录
file(GLOB_RECURSE SOURCES 
    "${PROJECT_SOURCE_DIR}/Drivers/Mcu/GD32F303/GD32F30x_Firmware_Library_V220/CMSIS/GD/GD32F30x/Source/GCC/startup_gd32f30x_hd.s"    
    "${PROJECT_SOURCE_DIR}/Drivers/Mcu/GD32F303/GD32F30x_Firmware_Library_V220/CMSIS/GD/GD32F30x/Source/*.c"
    "${PROJECT_SOURCE_DIR}/Drivers/Mcu/GD32F303/GD32F30x_Firmware_Library_V220/GD32F30x_standard_peripheral/Source/*.c"
    "${PROJECT_SOURCE_DIR}/Drivers/Mcu/GD32F303/*.c"
    "${PROJECT_SOURCE_DIR}/UserApp/*.c"

 
)

set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/Target/GD32F303/gd32f3xx_flash.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
