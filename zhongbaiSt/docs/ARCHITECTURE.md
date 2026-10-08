# 产品固件结构

| 路径 | 职责 |
|---|---|
| `Src/product_modbus.c`、`Inc/product_modbus.h` | Modbus RTU、产品寄存器、工程扩展 |
| `Src/commissioning.c`、`Inc/commissioning.h` | M1 FAST 启动、工程运行参数、停止与故障确认 |
| `Src/main.c` | 外设初始化和 Product Modbus 主循环入口 |
| `Src/stm32g4xx_it.c` | UART 中断、零点检测和加热定时 |
| `Src/adc.c` | 两块板的 VBUS ADC 条件编译 |
| `Inc/pmsm_motor_parameters.h` | M1 电机模型 |
| `CMakeLists.txt` | F1_TEST 与 POWER V004 两个产品目标 |
| `tools/product_modbus.py` | Windows 串口工程控制和状态诊断 |

产品档位与工程运行是两个入口。`MISSION/MOTOR` 使用固件固定产品参数；`RUN` 使用保持寄存器 33..48 的工程参数。修改协议时必须同时更新固件、命令行工具和 `F1_PRODUCT_MODBUS.md`。

`Drivers` 与 `MCSDK_v5.4.8-Full` 是构建依赖，正常功能修改应集中在 `Src`、`Inc` 和 `tools`。CubeMX/MCSDK 重新生成可能覆盖串口、ADC、保护和手写逻辑，必须逐项检查差异。
