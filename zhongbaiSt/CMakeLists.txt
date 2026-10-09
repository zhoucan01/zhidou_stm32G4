cmake_minimum_required(VERSION 3.20)
project(zhongbaiSt_product_modbus C ASM)

set(PRODUCT_TARGET F1_Product_Modbus_v2_M1)
set(POWER_PRODUCT_TARGET F1_POWER_V004_Product_Modbus_v2_M1)
set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

set(APP_SOURCES
  Src/adc.c Src/commissioning.c Src/cordic.c Src/gpio.c Src/main.c Src/mc_api.c Src/mc_config.c
  Src/mc_interface.c Src/mc_math.c Src/mc_parameters.c Src/mc_tasks.c
  Src/motor_control_protocol.c Src/motorcontrol.c Src/regular_conversion_manager.c
  Src/stm32g4xx_hal_msp.c Src/stm32g4xx_it.c
  Src/stm32g4xx_mc_it.c Src/system_stm32g4xx.c Src/tim.c Src/ui_task.c Src/usart.c
  Src/user_interface.c Src/usrdug.c
)

set(HAL_SOURCES
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cordic.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c
  Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
)

set(MC_BASE MCSDK_v5.4.8-Full/MotorControl/MCSDK)
set(MC_SOURCES
  ${MC_BASE}/MCLib/Any/Src/bus_voltage_sensor.c
  ${MC_BASE}/MCLib/Any/Src/circle_limitation.c
  ${MC_BASE}/MCLib/Any/Src/digital_output.c
  ${MC_BASE}/MCLib/Any/Src/motor_power_measurement.c
  ${MC_BASE}/MCLib/Any/Src/ntc_temperature_sensor.c
  ${MC_BASE}/MCLib/Any/Src/open_loop.c
  ${MC_BASE}/MCLib/Any/Src/pid_regulator.c
  ${MC_BASE}/MCLib/Any/Src/pqd_motor_power_measurement.c
  ${MC_BASE}/MCLib/Any/Src/pwm_common.c
  ${MC_BASE}/MCLib/Any/Src/pwm_curr_fdbk.c
  ${MC_BASE}/MCLib/Any/Src/r_divider_bus_voltage_sensor.c
  ${MC_BASE}/MCLib/Any/Src/ramp_ext_mngr.c
  ${MC_BASE}/MCLib/Any/Src/revup_ctrl.c
  ${MC_BASE}/MCLib/Any/Src/speed_pos_fdbk.c
  ${MC_BASE}/MCLib/Any/Src/speed_torq_ctrl.c
  ${MC_BASE}/MCLib/Any/Src/state_machine.c
  ${MC_BASE}/MCLib/Any/Src/sto_pll_speed_pos_fdbk.c
  ${MC_BASE}/MCLib/Any/Src/virtual_speed_sensor.c
  ${MC_BASE}/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c
  ${MC_BASE}/UILibrary/Src/frame_communication_protocol.c
  ${MC_BASE}/UILibrary/Src/ui_irq_handler.c
  ${MC_BASE}/UILibrary/Src/usart_frame_communication_protocol.c
)

set(PRODUCT_INCLUDES
  Inc Drivers/STM32G4xx_HAL_Driver/Inc Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
  Drivers/CMSIS/Device/ST/STM32G4xx/Include Drivers/CMSIS/Include Drivers/CMSIS/DSP/Include
  ${MC_BASE}/MCLib/Any/Inc ${MC_BASE}/MCLib/G4xx/Inc ${MC_BASE}/UILibrary/Inc
  ${MC_BASE}/SystemDriveParams MDK-ARM/.cmsis/include MDK-ARM/RTE/_zhongbaiSt
)

function(add_product_target target)
  add_executable(${target}
    Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g431xx.s
    ${APP_SOURCES} Src/product_modbus.c ${HAL_SOURCES} ${MC_SOURCES}
  )
  target_include_directories(${target} PRIVATE ${PRODUCT_INCLUDES})
  target_compile_definitions(${target} PRIVATE
    ARM_MATH_CM4 USE_HAL_DRIVER STM32G431xx F1_PRODUCT_MODBUS F1_PRODUCT_MODBUS_M1 USRDUG_UI_UART ${ARGN})
  target_compile_options(${target} PRIVATE
    ${CPU_FLAGS} -O2 -g3 -ffunction-sections -fdata-sections -funsigned-char -Wall)
  target_link_options(${target} PRIVATE
    ${CPU_FLAGS} -T${CMAKE_SOURCE_DIR}/STM32G431RBTx_FLASH.ld
    --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage)
  target_link_libraries(${target} PRIVATE m)
  set_target_properties(${target} PROPERTIES SUFFIX ".elf")
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> ${CMAKE_BINARY_DIR}/${target}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> ${CMAKE_BINARY_DIR}/${target}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
    COMMENT "Generating ${target} HEX/BIN and reporting image size")
endfunction()

add_product_target(${PRODUCT_TARGET} F1_TEST_BOARD)
add_product_target(${POWER_PRODUCT_TARGET})

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_custom_target(flash-product-modbus
  COMMAND ${Python3_EXECUTABLE} -m pyocd flash -t stm32g431rbtx -f 100k -M under-reset
          ${CMAKE_BINARY_DIR}/${PRODUCT_TARGET}.hex
  DEPENDS ${PRODUCT_TARGET} USES_TERMINAL)
add_custom_target(flash-power-product-modbus
  COMMAND ${Python3_EXECUTABLE} -m pyocd flash -t stm32g431rbtx -f 100k -M under-reset
          ${CMAKE_BINARY_DIR}/${POWER_PRODUCT_TARGET}.hex
  DEPENDS ${POWER_PRODUCT_TARGET} USES_TERMINAL)
