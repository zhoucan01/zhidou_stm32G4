#ifndef BOOTLOADER_MODBUS_PORT_H
#define BOOTLOADER_MODBUS_PORT_H

#include "modbus.h"
#include "stm32g4xx_hal.h"

/* USART2 DMA + IDLE 适配层：中断发布接收块，主循环负责组帧、校验和发送。
 * 业务通过此接口取帧、设置状态和登记应答；纯协议接口见 modbus.h。
 */
typedef struct
{
  uint32_t receive_errors, chunk_overflows, frame_overflows;
  uint32_t response_overflows, frame_errors, crc16_errors, timeouts, transmit_errors;
} modbus_stats_t;

/* Initialize once after USART2/DMA and the system clock. This enables the DWT cycle counter. */
HAL_StatusTypeDef Modbus_StartReceive(void);
/* 主循环安装版本读取器；offset/count 均在 1 KiB 版本区内，成功返回 1。
 * mode 为 0 Bootloader 或 1 APP。版本请求不写 Flash。
 */
void modbus_port_set_version_reader(int (*reader)(uint16_t, uint8_t *, uint8_t), uint8_t mode);
/* Call frequently from the main loop, never from an interrupt. Query and operation 0
   are answered here; operation 2 sends its confirmation before resetting. Operations
   1/3 are delivered to the upgrade logic, which is responsible for their replies.
   0xAA CRC16, length and timeout failures are answered; valid packets are not
   acknowledged automatically. UART IDLE is a chunk boundary, not a frame boundary. */
void modbusdeal(void);

/* Non-query control and data frames share a three-frame FIFO. A different type at its
   head returns 0. Return 1 on success, 0 when unavailable, -1 for invalid arguments/
   capacity. Take frames in order; a full FIFO drops new frames and updates statistics. */
int modbus_port_get_10(modbus_10_data_t *out);
int modbus_port_get_aa(modbus_aa_data_t *out, uint8_t *firmware, size_t capacity);
int modbus_port_take_query(void);
int modbus_port_rx_idle(void);
HAL_StatusTypeDef modbus_port_pause_rx(void);
/* Atomically pause only if the bus and all receive queues are idle. */
HAL_StatusTypeDef modbus_port_pause_rx_if_idle(void);
HAL_StatusTypeDef modbus_port_resume_rx(void);
int modbus_port_read_registers(uint16_t start, uint16_t count, uint16_t *out);
void modbus_port_set_ota_status(uint16_t status);
uint16_t modbus_port_get_ota_status(void);
void modbus_port_set_session_key(uint32_t key);
/* Queue an application-generated response: 1 queued, 0 full, -1 invalid.
   0xAA OK must only be sent after the application has actually written the data. */
int modbus_set_ack(uint8_t code, uint16_t packet);
int modbus_set_10_reply(uint8_t is_query);
int modbus_reply_pending(void);
/* Cumulative counters include lost DMA chunks, frames, responses and UART errors. */
int modbus_get_stats(modbus_stats_t *out);

#endif
