#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "modbus_port.h"
#include "usart.h"
#include "error_trace.h"
#include "boot_log.h"

CoreDebug_Type test_core_debug;
DWT_Type test_dwt;
uint32_t SystemCoreClock = 1000000u, test_primask, test_flags;
static DMA_HandleTypeDef dma;
static USART_TypeDef uart_regs = { USART_CR1_RE };
UART_HandleTypeDef huart2 = { &dma, &uart_regs, { 1000000u } };
UART_HandleTypeDef huart1;
static UART_HandleTypeDef other_uart;
static uint8_t *dma_buffer, transmitted[MODBUS_RESPONSE_CAPACITY];
static unsigned int tx_count, reset_count, fatal_count, tx_size;
static int fail_transmit;
static int fail_receive;
static int busy_until_abort;
static unsigned int receive_calls, abort_calls;
static int fail_log;
static unsigned int log_count, log_size;
static char log_text[8192];
static size_t log_used;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
  assert(uart == &huart2 && size == 1047u);
  ++receive_calls;
  if(fail_receive || busy_until_abort) return HAL_BUSY;
  dma_buffer = data;
  dma.counter = size;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
  assert(uart == &huart2 && test_primask == 1u);
  ++abort_calls;
  busy_until_abort = 0;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size, uint32_t timeout)
{
  if(uart == &huart1)
  {
    assert(test_primask == 0u && size <= 80u && timeout == 20u);
    if(fail_log) return HAL_ERROR;
    assert(log_used + size < sizeof(log_text));
    memcpy(log_text + log_used, data, size); log_used += size;
    log_text[log_used] = 0; log_size = size; ++log_count;
    return HAL_OK;
  }
  assert(uart == &huart2 && test_primask == 0u && (uart_regs.CR1 & USART_CR1_RE) == 0u);
  assert(timeout == 20u && size <= sizeof(transmitted));
  ++tx_count;
  if (fail_transmit)
    return HAL_ERROR;
  memcpy(transmitted, data, size);
  tx_size = size;
  return HAL_OK;
}

void Error_Handler(void) { ++fatal_count; }
void NVIC_SystemReset(void) { assert(tx_size == 8u && (uart_regs.CR1 & USART_CR1_RE)); ++reset_count; }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);

static void put16(uint8_t *p, unsigned value) { p[0] = (uint8_t)(value >> 8); p[1] = (uint8_t)value; }
static void seal(uint8_t *p, size_t size)
{
  uint16_t crc = modbus_crc16(p, size - 2u);
  p[size - 2u] = (uint8_t)crc;
  p[size - 1u] = (uint8_t)(crc >> 8);
}
static void feed(const uint8_t *p, size_t size)
{
  assert(size <= 1047u);
  memcpy(dma_buffer, p, size);
  dma.counter = (uint32_t)(1047u - size);
  HAL_UARTEx_RxEventCallback(&huart2, (uint16_t)size);
}
static void advance(uint32_t microseconds) { test_dwt.CYCCNT += microseconds; }
static void flush_reply(void) { advance(2000u); modbusdeal(); }

static int read_version(uint16_t offset, uint8_t *out, uint8_t count)
{
  unsigned int i;
  for(i = 0; i < count; ++i) out[i] = (uint8_t)(offset + i);
  return 1;
}

int main(void)
{
  uint8_t query[14] = {MODBUS_ADDRESS, 0x10, 0, 0xe6, 0, 3};
  uint8_t control[20] = {MODBUS_ADDRESS, 0x10, 0, 0xe6, 0, 4, 0, 0, 0, 8};
  uint8_t frame[1046] = {MODBUS_ADDRESS, 0xaa}, firmware[1024], reply[14];
  const uint8_t *found;
  size_t cursor, found_size;
  uint16_t regs[4];
  modbus_10_data_t cmd;
  modbus_aa_data_t packet;
  modbus_stats_t stats;
  modbus_t ctx = {0x12345678u, MODBUS_STATUS_DOWNLOADING};
  unsigned int i, before;

  assert(modbus_crc16((const uint8_t *)"123456789", 9) == 0x4b37u);
  assert(modbus_crc32((const uint8_t *)"123456789", 9) == 0xcbf43926u);
  test_dwt.CTRL = DWT_CTRL_NOCYCCNT_Msk;
  assert(Modbus_StartReceive() == HAL_ERROR);
  test_dwt.CTRL = 0;
  assert(Modbus_StartReceive() == HAL_OK);
  boot_log_start();
  assert(strstr(log_text, "USART2 OTA; USART1 log") != NULL);
  assert(test_core_debug.DEMCR && test_dwt.CTRL);
  assert(modbus_port_get_ota_status() == MODBUS_STATUS_FORBIDDEN);
  assert(modbus_build_10_reply(&ctx, 1, reply, sizeof(reply)) == 14);
  assert(reply[7] == 2 && reply[8] == 0x12 && modbus_crc16(reply, 14) == 0);

  seal(query, sizeof(query));
  cursor = 0;
  assert(modbus_find_frame(&ctx, query, sizeof(query), &cursor, &found, &found_size) == 1);
  assert(found == query && found_size == 14 && cursor == 14);
  assert(modbus_find_frame(&ctx, query, sizeof(query), &cursor, &found, &found_size) == 0);
  feed(query, 5); modbusdeal();
  advance(1000u);
  feed(query + 5, sizeof(query) - 5u); modbusdeal();
  assert(tx_count == 0 && modbus_reply_pending());
  flush_reply();
  assert(tx_count == 1 && tx_size == 14 && transmitted[7] == MODBUS_STATUS_FORBIDDEN);
  assert(modbus_crc16(transmitted, tx_size) == 0);
  assert(modbus_port_take_query() == 1 && modbus_port_take_query() == 0);

  put16(control + 10, 1); control[12] = 0x12; control[13] = 0x34;
  control[14] = 0x56; control[15] = 0x78; put16(control + 16, 1); seal(control, sizeof(control));
  feed(control, sizeof(control)); modbusdeal();
  assert(modbus_port_read_registers(0xe6, 4, regs) == 1);
  assert(regs[0] == 0 && regs[1] == 0x1234 && regs[2] == 0x5678 && regs[3] == 1);
  assert(modbus_port_get_10(&cmd) == 1 && cmd.operation == 1 && cmd.key == 0x12345678u);
  assert(modbus_set_10_reply(1) == 1);
  flush_reply();
  assert(tx_size == 14 && transmitted[7] == MODBUS_STATUS_FORBIDDEN);

  put16(frame + 2, sizeof(frame)); put16(frame + 4, 1); put16(frame + 10, 1024);
  put16(frame + 12, 2); put16(frame + 14, 1);
  for (i = 0; i < 1024u; ++i) frame[16 + i] = (uint8_t)i;
  frame[1040] = 0xcb; frame[1041] = 0xf4; frame[1042] = 0x39; frame[1043] = 0x26;
  seal(frame, sizeof(frame));
  feed(frame, 256); modbusdeal();
  feed(frame + 256, 500); modbusdeal();
  feed(frame + 756, 290); modbusdeal();
  assert(modbus_port_get_aa(&packet, firmware, 1023) == -1);
  assert(modbus_port_get_aa(&packet, firmware, sizeof(firmware)) == 1);
  assert(packet.firmware_size == 1024 && packet.packet == 1 && !memcmp(firmware, frame + 16, 1024));
  assert(modbus_check_aa_crc32(&packet, (const uint8_t *)"123456789", 9) == -1);
  assert(modbus_set_ack(MODBUS_ACK_OK, packet.packet) == 1);
  flush_reply(); assert(tx_size == 7 && transmitted[2] == 1);

  frame[1045] ^= 1u;
  feed(frame, sizeof(frame)); modbusdeal();
  advance(3500u); modbusdeal();
  flush_reply();
  assert(tx_size == 7 && transmitted[2] == MODBUS_ACK_FRAME_CRC && modbus_crc16(transmitted, 7) == 0);
  frame[1045] ^= 1u;

  feed(frame, 16); modbusdeal();
  advance(3499u); modbusdeal();
  modbus_get_stats(&stats); assert(stats.timeouts == 0);
  advance(1u); modbusdeal();
  assert(tx_size == 7 && transmitted[2] == MODBUS_ACK_RX_TIMEOUT);
  modbus_get_stats(&stats); assert(stats.timeouts == 1 && stats.crc16_errors == 1);

  put16(frame + 2, 1047); put16(frame + 10, 1025);
  feed(frame, 16); modbusdeal();
  advance(3500u); modbusdeal();
  assert(tx_size == 7 && transmitted[2] == MODBUS_ACK_TOO_LONG);
  put16(frame + 2, sizeof(frame)); put16(frame + 10, 1024); seal(frame, sizeof(frame));

  for (i = 0; i < 5u; ++i) feed(query, sizeof(query));
  before = tx_count; modbusdeal();
  modbus_get_stats(&stats); assert(stats.chunk_overflows == 1 && tx_count == before);
  HAL_UART_ErrorCallback(&other_uart);
  HAL_UART_ErrorCallback(&huart2);
  modbusdeal(); modbus_get_stats(&stats); assert(stats.receive_errors == 1);
  feed(query, sizeof(query)); modbusdeal();
  fail_transmit = 1; flush_reply();
  assert(modbus_reply_pending() && (uart_regs.CR1 & USART_CR1_RE));
  fail_transmit = 0; flush_reply();
  assert(!modbus_reply_pending());
  modbus_get_stats(&stats); assert(stats.transmit_errors == 1);

  for (i = 0; i < 4u; ++i) feed(frame, sizeof(frame));
  modbusdeal();
  modbus_get_stats(&stats); assert(stats.frame_overflows == 1);
  for (i = 0; i < 3u; ++i)
    assert(modbus_port_get_aa(&packet, firmware, sizeof(firmware)) == 1);
  assert(modbus_port_get_aa(&packet, firmware, sizeof(firmware)) == 0);

  for (i = 0; i < 4u; ++i) assert(modbus_set_ack(MODBUS_ACK_KEY, (uint16_t)i) == 1);
  assert(modbus_set_ack(MODBUS_ACK_KEY, 4) == 0);
  test_flags = UART_FLAG_RXNE; before = tx_count; flush_reply();
  assert(tx_count == before && modbus_reply_pending());
  test_flags = 0;
  for (i = 0; i < 4u; ++i)
  {
    flush_reply();
    assert(transmitted[2] == MODBUS_ACK_KEY && transmitted[4] == i);
  }
  modbus_get_stats(&stats); assert(stats.response_overflows == 1);

  feed(frame, 16); modbusdeal();
  HAL_UART_ErrorCallback(&huart2); modbusdeal();
  feed(frame + 16, sizeof(frame) - 16u); modbusdeal();
  assert(modbus_port_get_aa(&packet, firmware, sizeof(firmware)) == 0);
  advance(3500u); modbusdeal();

  put16(control + 16, 2); seal(control, sizeof(control));
  feed(control, sizeof(control)); modbusdeal();
  assert(reset_count == 0);
  fail_transmit = 1; flush_reply();
  assert(reset_count == 0 && modbus_reply_pending());
  fail_transmit = 0; flush_reply();
  assert(reset_count == 1 && tx_size == 8 && modbus_crc16(transmitted, 8) == 0);
  assert(modbus_port_take_query() == 1);
  advance(2000u);
  test_flags = UART_FLAG_RXNE;
  assert(modbus_port_pause_rx_if_idle() == HAL_BUSY);
  test_flags = 0;
  assert(modbus_port_pause_rx_if_idle() == HAL_OK);
  assert((uart_regs.CR1 & USART_CR1_RE) == 0u);
  assert(modbus_port_resume_rx() == HAL_OK && (uart_regs.CR1 & USART_CR1_RE));

  /* 后半帧到达时，即使主循环未在静默期间运行，也不能续接超时半帧。 */
  modbus_get_stats(&stats); before = stats.timeouts;
  feed(query, 5); modbusdeal();
  advance(10000u + 90u);
  feed(query + 5, sizeof(query) - 5u); modbusdeal();
  assert(modbus_port_take_query() == 0);
  modbus_get_stats(&stats); assert(stats.timeouts == before + 1u);

  /* 两块均已排队时，仍按接收时间戳判断；覆盖 CYCCNT 回绕。 */
  test_dwt.CYCCNT = 0xfffffff0u;
  feed(query, 5);
  advance(10000u + 90u);
  feed(query + 5, sizeof(query) - 5u); modbusdeal();
  assert(modbus_port_take_query() == 0);
  modbus_get_stats(&stats); assert(stats.timeouts == before + 2u);

  /* 1030 字节本身需要 10.3 ms，连续传输不能被误判为 3.5 ms 静默。 */
  feed(frame, 16); modbusdeal();
  advance(10300u);
  feed(frame + 16, sizeof(frame) - 16u); modbusdeal();
  assert(modbus_port_get_aa(&packet, firmware, sizeof(firmware)) == 1);
  modbus_get_stats(&stats); assert(stats.timeouts == before + 2u);

  /* 同样的长块前增加 3.5 ms 静默，旧帧应失效并登记 0x03 应答。 */
  feed(frame, 16); modbusdeal();
  advance(3500u + 10300u);
  feed(frame + 16, sizeof(frame) - 16u); modbusdeal();
  assert(modbus_port_get_aa(&packet, firmware, sizeof(firmware)) == 0);
  modbus_get_stats(&stats); assert(stats.timeouts == before + 3u);
  flush_reply();
  assert(tx_size == 7u && transmitted[2] == MODBUS_ACK_RX_TIMEOUT);
  feed(query, sizeof(query)); modbusdeal(); flush_reply();
  assert(modbus_port_take_query() == 1 && tx_size == 14u);
  {
    uint8_t request[8] = {MODBUS_ADDRESS, 0x11, 0, 32, 0, 32};
    modbus_port_set_version_reader(read_version, 0u);
    seal(request, sizeof(request));
    feed(request, 3); modbusdeal(); feed(request + 3, 5); modbusdeal(); flush_reply();
    assert(tx_size == 40u && transmitted[1] == 0x11u && transmitted[2] == 0u);
    assert(transmitted[4] == 32u && transmitted[5] == 32u && transmitted[6] == 32u);
    assert(transmitted[37] == 63u && modbus_crc16(transmitted, tx_size) == 0u);
    assert(modbus_port_take_query() == 1);
    before = tx_count;
    request[2] = 4; request[3] = 0; seal(request, sizeof(request));
    feed(request, sizeof(request)); modbusdeal(); flush_reply(); assert(tx_count == before);
    request[2] = 0; request[3] = 0; request[7] ^= 1u;
    feed(request, sizeof(request)); modbusdeal(); advance(3500u); modbusdeal();
    assert(tx_count == before);
  }
  assert(fatal_count == 0);
  {
    unsigned int before_logs = log_count;
    test_primask = 1u;
    boot_notice("Must skip locked context", 1, 2);
    test_primask = 0u;
    assert(log_count == before_logs);
    boot_notice("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789", 0xfffffffeu, 0xfffffffeu);
    assert(log_size == 79u && log_text[log_used - 1u] == '\n');
    fail_log = 1;
    boot_notice("Dropped log", BOOT_LOG_NONE, BOOT_LOG_NONE);
    assert(boot_log_drops == 1u && fatal_count == 0u);
    fail_log = 0;
    feed(query, sizeof(query)); modbusdeal(); flush_reply();
    assert(tx_size == 14u && modbus_crc16(transmitted, tx_size) == 0u);
  }
  /* Error callbacks leave RX recovery to the main loop. */
  {
    unsigned int calls = receive_calls, aborts = abort_calls;
    busy_until_abort = 1;
    HAL_UART_ErrorCallback(&huart2);
    assert(receive_calls == calls && abort_calls == aborts && fatal_count == 0u);
    modbusdeal();
    assert(abort_calls == aborts + 1u && receive_calls == calls + 1u);
    assert(fatal_count == 0u);
    feed(query, sizeof(query)); modbusdeal(); flush_reply();
    assert(tx_size == 14u && modbus_crc16(transmitted, tx_size) == 0u);
  }
  /* BUSY during RX event or recovery is deferred, never fatal. */
  fail_receive = 1;
  feed(query, sizeof(query));
  modbusdeal(); modbusdeal();
  assert(fatal_count == 0u);
  fail_receive = 0;
  modbusdeal();
  feed(query, sizeof(query)); modbusdeal(); flush_reply();
  assert(tx_size == 14u && modbus_crc16(transmitted, tx_size) == 0u);
  assert(g_error_trace.code == 0u);
  puts("modbus protocol and DMA tests passed");
  return 0;
}
