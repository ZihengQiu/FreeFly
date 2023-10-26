cmake_minimum_required (VERSION 3.15)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi.cmake)

project(freefly VERSION 1.0)

enable_language(C ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENfSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(EXECUTABLE ${PROJECT_NAME}.elf)

set(DEBUG 1)
set(ENABLE_SYSTEMVIEW 1)
set(USE_SELF_SYSCALL 1)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")

#add definitions
add_definitions(-DSTM32F40_41xxx)
add_definitions(-DUSE_STDPERIPH_DRIVER)
if(USE_SELF_SYSCALL)
	add_definitions(-DUSE_SELF_SYSCALL)
endif()

# set project dirs
set(PROJECT_SYSTEM_PATH ${PROJECT_SOURCE_DIR}/System)
set(PROJECT_APP_PATH ${PROJECT_SOURCE_DIR}/Application) 
set(FWLIB_DIR ${PROJECT_SYSTEM_PATH}/FWLIB)
set(OS_DIR ${PROJECT_SOURCE_DIR}/ucos2)

# add source files
aux_source_directory(${FWLIB_DIR}/Src FWLIB_SRC)
aux_source_directory(${PROJECT_APP_PATH}/Hardware/Src APP_HW_SRC)
aux_source_directory(${PROJECT_APP_PATH}/SystemConfig APP_SYSTEM_SRC)
file(GLOB_RECURSE APP_SYSTEMVIEW_SRC ${PROJECT_APP_PATH}/SystemView/*.c)
#  find .c and .s
file(GLOB_RECURSE OS_SRC ${OS_DIR}/Ports/*.c ${OS_DIR}/Ports/*.s ${OS_DIR}/Source/*.c)
add_executable(${EXECUTABLE}
		${FWLIB_SRC}
		${APP_HW_SRC}
		${PROJECT_APP_PATH}/main.c
		$<$<BOOL:${ENABLE_SYSTEMVIEW}>:${APP_SYSTEM_SRC}>
		${OS_SRC}
		${APP_SYSTEMVIEW_SRC}
		${PROJECT_SYSTEM_PATH}/CMSIS/Device/startup_stm32f401xe.s
		${PROJECT_SYSTEM_PATH}/CMSIS/Device/system_stm32f4xx.c
		${PROJECT_SYSTEM_PATH}/BSP/syscalls.c
		)

# add include files
target_include_directories(${EXECUTABLE} PRIVATE
		${FWLIB_DIR}/Inc
		${OS_DIR}/Ports
		${OS_DIR}/Source
		${OS_DIR}/Cfg
		${OS_DIR}/Cfg/Template
		${PROJECT_APP_PATH}
		${PROJECT_APP_PATH}/Hardware/Inc
		${PROJECT_APP_PATH}/SystemConfig
		$<$<BOOL:${ENABLE_SYSTEMVIEW}>:
			${PROJECT_APP_PATH}/SystemView/GenericConfig
			${PROJECT_APP_PATH}/SystemView/GenericFiles
			${PROJECT_APP_PATH}/SystemView/OSSpecific
		>
		${PROJECT_SYSTEM_PATH}/CMSIS/Core
		${PROJECT_SYSTEM_PATH}/CMSIS/Device
        )

target_compile_definitions(${EXECUTABLE} PRIVATE
        STM32F401xE
        )

target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -fdata-sections
        -ffunction-sections
		-fno-math-errno
        -Wall
		-O2
		$<$<BOOL:${DEBUG}>:-g>
        )

target_link_options(${EXECUTABLE} PRIVATE
        -T${CMAKE_SOURCE_DIR}/STM32F401RETx_FLASH.ld
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        # -specs=nano.specs
		-lc
		-lm
		-u_printf_float
		# if USE_SELF_SYSCALL = 0, then add -lnosys
		$<$<NOT:$<BOOL:${USE_SELF_SYSCALL}>>:-lnosys>
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        )

# post build commands
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE}
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)
