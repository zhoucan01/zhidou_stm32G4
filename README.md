# STM32G431 产品 Modbus 固件

本分支用于中百 F1 产品板的 M1 无感 FOC、RS485 Modbus 和加热控制交付。工程由原始 `V1.1.3` 产品代码迁移而来，FOC 使用已验证的 M1 参数与 FAST 启动逻辑，非 FOC 控制保持 G4 上位机寄存器映射。

## 从这里开始

1. 进入 [`zhongbaiSt`](zhongbaiSt/) 目录。
2. 阅读 [`docs/INDEX.md`](zhongbaiSt/docs/INDEX.md) 选择板卡、编译和协议文档。
3. POWER V004 正式板使用 `power-product-modbus`；v1.5 实验板使用 `product-modbus`，两者不能混刷。

POWER V004 的最短构建与烧录流程：

```powershell
cd zhongbaiSt
cmake --preset gcc-release
cmake --build --preset power-product-modbus
.\flash.cmd power-product
```

Modbus 工程终端：

```powershell
.\tools\product-modbus.cmd COM14
```

当前产品协议版本：`0.2.4.0`，RS485 默认速率为 `1,000,000 baud`。完整协议、上位机使用和扩展命令见 [`F1_PRODUCT_MODBUS.md`](zhongbaiSt/docs/F1_PRODUCT_MODBUS.md)。

## 分支内容

| 路径 | 内容 |
|---|---|
| `zhongbaiSt/Src`, `Inc` | 产品应用、FOC、外设及 Modbus 源码 |
| `zhongbaiSt/Drivers` | STM32 HAL/CMSIS 依赖 |
| `zhongbaiSt/MCSDK_v5.4.8-Full` | 工程所需 MCSDK 5.4.8 源码 |
| `zhongbaiSt/MDK-ARM` | Keil 工程配置；不包含编译产物 |
| `zhongbaiSt/tools` | 产品 Modbus 命令行工具 |
| `zhongbaiSt/docs` | 产品使用、构建、架构及硬件事实 |

实验日志、历史 HEX/ELF、Motion Lab、CN2 ASCII 测试系列、原始硬件附件和旧 V1.1.3 说明未放在本交付分支的工作树中；需要追溯时使用 Git 历史。
