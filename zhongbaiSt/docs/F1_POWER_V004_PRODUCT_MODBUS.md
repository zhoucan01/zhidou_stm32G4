# F1_POWER_V004 产品 Modbus M1

本页只记录 POWER 板差异。完整的上位机操作、功能码、寄存器、工程扩展、波特率修改和编译方法见 [`F1_PRODUCT_MODBUS.md`](F1_PRODUCT_MODBUS.md)。

## 板卡与接口

| 项目 | 配置 |
|---|---|
| 板卡/MCU | `F1_POWER_V004` / STM32G431RBT6 |
| RS485 | CN3；PA2/PA3 USART2；PA4 `DE&~RE` |
| 通信 | Modbus RTU，从站 3，1,000,000 baud，8N1 |
| VBUS | PC2 / ADC2_IN8 |
| 加热 | PB13/`HEAT`；过零移相控制 |
| 上加热 | PB14 无硬件连接，命令和回读固定为 0 |

不要向此板烧录 `F1_Product_Modbus_v2_M1.hex`，它是 F1_TEST 目标并使用 PA6/ADC2_IN3 采样 VBUS。

## 编译、产物和烧录

```powershell
cmake --preset gcc-release
cmake --build --preset power-product-modbus
flash.cmd power-product
```

产物：`build/gcc-release/F1_POWER_V004_Product_Modbus_v2_M1.hex`。

当前产品协议版本为 `0.2.7.0`。上位机 FC04 地址 11 应读到 `0x0002`，地址 12 应读到 `0x0700`。烧录后应至少验证：

1. FC03 地址 0..4 能回读产品命令；
2. FC04 地址 0..13 能回读状态和版本；
3. 电机档位与实际方向/转速一致；
4. 下加热命令使 PB13 产生与零点同步的控制波形，并带来可测量的实际功率变化。
