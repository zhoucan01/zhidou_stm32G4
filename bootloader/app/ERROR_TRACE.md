# Error_Handler 来源定位

在 Keil 的 Watch 窗口添加 `g_error_trace`，重新下载带诊断的固件并复现问题。
停在 `Error_Handler()` 后展开此变量：

| 字段 | 含义 |
| --- | --- |
| `code` | 来源编号，下面按十六进制列出 |
| `reason` | 失败操作的文字说明；HAL 初始化会显示实际调用表达式 |
| `file`、`function`、`line` | 首次失败的源文件、函数和行号 |
| `detail` | HAL 返回值、队列结果或非法状态值 |
| `active_exception` | 0 为主循环；非 0 表示异常/中断号（IPSR） |
| `interrupt_mask` | 记录时 PRIMASK；1 表示当时已关闭可屏蔽中断 |

记录保留第一次错误，后续或嵌套错误不会覆盖。复位后重新开始记录。
诊断只写 RAM，不使用串口打印、不写 Flash，不改变原来的停机处理。

| `code` | 来源及应检查的内容 |
| --- | --- |
| `0x1000` | HAL 初始化/时钟配置失败；看 `reason` 中的 HAL 调用及 `detail` |
| `0x2001` | 上电启动 USART2 DMA/DWT 接收失败；看 `SystemCoreClock`、`DWT->CTRL`、`huart2` |
| `0x2002` | DMA/IDLE 接收事件回调中重新启动 DMA 失败 |
| `0x2003` | Main-loop RX abort/restart failed (HAL_BUSY is deferred); see detail/function |
| `0x2004` | 发送应答前停止串口接收失败 |
| `0x2005` | 应答发送后重新启动 DMA 失败 |
| `0x3001` | 数据 ACK 入队失败；`detail=0` 队列满，`-1` 参数非法 |
| `0x3002` | 控制/状态应答入队失败 |
| `0x3003` | 擦除前暂停接收失败 |
| `0x3004` | 擦除后恢复接收失败 |
| `0x3005` | 跳转 APP 前暂停接收失败；正常的 HAL_BUSY 不进入此错误 |
| `0x3006` | 升级状态机值非法；`detail` 为触发时的状态值 |
| `0xFFFF` | 未标记的直接调用；此时记录只能指出进入了 Error_Handler，不能还原调用者 |

HAL 返回值：`0=HAL_OK`、`1=HAL_ERROR`、`2=HAL_BUSY`、`3=HAL_TIMEOUT`。
注意队列接口使用不同约定，`detail=0` 在队列错误中表示队列满。

Bootloader 新增的 OTA 致命分支均使用 `ERROR_HANDLER_FROM` 并在调用点标注编号。
APP 的时钟、ADC、CORDIC、定时器、UART 初始化使用 `ERROR_CHECK_HAL`，保留实际失败返回值。
若重新用 CubeMX 生成代码，请检查生成区域中的这些标记是否仍保留。

USART2 error callbacks mark recovery pending. Main-loop recovery aborts old RX, discards partial frames and restarts DMA. HAL_BUSY is retried without entering Error_Handler. USART1 logs RX error counts.
