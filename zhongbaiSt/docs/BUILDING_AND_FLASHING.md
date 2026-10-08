# 编译和烧录

命令均从 `zhongbaiSt` 目录执行。正式构建路径是 Windows GCC/CMake；构建产物不提交 Git。

## 工具链定位

`cmake/arm-none-eabi.cmake` 会自动查找 Arm GNU 工具链，顺序为：

1. `ARM_GCC_ROOT` 环境变量；
2. STM32Cube / STM32CubeIDE 自带的 GCC（推荐，14.x）；
3. `PATH` 上的 `arm-none-eabi-gcc`。

Ninja 同样会被自动查找（`PATH`、`%LOCALAPPDATA%/at32-tools/ninja`、CMake 自带）。

**建议使用 GCC 14.x。** 部分厂商提供的 GCC 10.3 不支持 Windows 长路径，在本工程较深的 `MCSDK_v5.4.8-Full` 目录下会报：

```
fatal error: opening dependency file ...\xxx.c.obj.d: No such file or directory
```

该报错指向源码行号，实际原因是依赖文件路径达到 MAX_PATH（260 字符）。遇到此错误时更换 14.x 工具链（或用短路径构建目录）即可。

## POWER V004

```powershell
cmake --preset gcc-release
cmake --build --preset power-product-modbus
.\flash.cmd power-product
```

产物：`build/gcc-release/F1_POWER_V004_Product_Modbus_v2_M1.hex`。该目标使用 PC2/ADC2_IN8 采样 VBUS，RS485 位于 CN3。

## F1_TEST v1.5 实验板

```powershell
cmake --preset gcc-release
cmake --build --preset product-modbus
.\flash.cmd product
```

产物：`build/gcc-release/F1_Product_Modbus_v2_M1.hex`。该目标定义 `F1_TEST_BOARD`，使用 PA6/ADC2_IN3 采样 VBUS，RS485 位于 CN4。

## 同时编译两个目标

```powershell
.\build.cmd
```

`flash.cmd` 使用 pyOCD、STM32G431RBTx、100 kHz 和 connect-under-reset，并在烧录前打印文件路径和 SHA-256。烧录成功后用 FC04 地址 11、12 确认版本，再验证电机与加热。

两个 HEX 的 VBUS ADC 路由不同，不得混刷。Keil 工程配置保留用于查看和维护原工程；本产品分支的发布验证以 CMake 目标为准。
