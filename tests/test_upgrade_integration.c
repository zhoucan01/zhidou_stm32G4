#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "modbus_port.h"
#include "upgrade.h"
#include "upgrade_flash.h"
#include "upgrade_platform.h"
#include "sha256.h"
#include "usart.h"
#include "boot_log.h"

CoreDebug_Type test_core_debug;
DWT_Type test_dwt;
uint32_t SystemCoreClock = 1000000u, test_primask, test_flags;
static DMA_HandleTypeDef dma;
static USART_TypeDef uart_regs = { USART_CR1_RE };
UART_HandleTypeDef huart2 = { &dma, &uart_regs, { 1000000u } };
UART_HandleTypeDef huart1;
static uint8_t flash[128u * 1024u], *dma_buffer, tx[14];
static unsigned int tx_size, tx_count, jumps, erase_count;
static int paused;
static char log_text[8192];
static size_t log_used;

uint32_t HAL_GetTick(void) { return test_dwt.CYCCNT / 1000u; }
void Error_Handler(void) { assert(!"Error_Handler"); }
void NVIC_SystemReset(void) { assert(!"unexpected reset"); }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
  assert(uart == &huart2 && size == 1047u); dma_buffer = data; dma.counter = size;
  paused = 0; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
  assert(uart == &huart2); paused = 1; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size, uint32_t timeout)
{
  if(uart == &huart1)
  {
    assert(size <= 80u && timeout == 20u && log_used + size < sizeof(log_text));
    memcpy(log_text + log_used, data, size); log_used += size; log_text[log_used] = 0;
    return HAL_OK;
  }
  assert(uart == &huart2 && timeout == 20u && size <= sizeof(tx));
  assert((uart_regs.CR1 & USART_CR1_RE) == 0u);
  memcpy(tx, data, size); tx_size = size; ++tx_count; return HAL_OK;
}
int Upgrade_FlashRead(uint32_t address, void *data, size_t size)
{
  if (address < UPGRADE_RECORD_ADDRESS || address > UPGRADE_FLASH_END ||
      size > UPGRADE_FLASH_END - address) return 0;
  memcpy(data, flash + address - UPGRADE_FLASH_BASE, size); return 1;
}
int Upgrade_FlashErase(uint32_t address, uint32_t size)
{
  assert(paused && !(uart_regs.CR1 & USART_CR1_RE));
  assert(address >= UPGRADE_RECORD_ADDRESS && address + size <= UPGRADE_FLASH_END);
  ++erase_count; memset(flash + address - UPGRADE_FLASH_BASE, 0xff, size); return 1;
}
int Upgrade_FlashProgram(uint32_t address, const uint8_t *data, size_t size)
{
  size_t i;
  assert(address >= UPGRADE_RECORD_ADDRESS && address + size <= UPGRADE_FLASH_END);
  assert(!(address & 7u) && !(size & 7u));
  for (i = 0; i < size; ++i)
  {
    uint8_t *p = flash + address - UPGRADE_FLASH_BASE + i;
    assert((*p & data[i]) == data[i]); *p &= data[i];
  }
  return 1;
}
void Upgrade_JumpToApp(uint32_t stack, uint32_t entry)
{
  assert(paused && !(uart_regs.CR1 & USART_CR1_RE));
  assert(stack == 0x20008000u && entry == MSP1_ADDR + 17u); ++jumps;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
static void put16(uint8_t *p, uint16_t x) { p[0] = (uint8_t)(x >> 8); p[1] = (uint8_t)x; }
static void put32(uint8_t *p, uint32_t x) { put16(p, (uint16_t)(x >> 16)); put16(p + 2, (uint16_t)x); }
static void put32le(uint8_t *p, uint32_t x)
{ p[0] = (uint8_t)x; p[1] = (uint8_t)(x >> 8); p[2] = (uint8_t)(x >> 16); p[3] = (uint8_t)(x >> 24); }
static void seal(uint8_t *p, size_t n)
{
  uint16_t crc = modbus_crc16(p, n - 2u);
  p[n - 2u] = (uint8_t)crc; p[n - 1u] = (uint8_t)(crc >> 8);
}
static void step(void) { modbusdeal(); Upgrade_Poll(); }
static void feed(const uint8_t *data, size_t size)
{
  assert(size <= 1047u && !paused);
  memcpy(dma_buffer, data, size); dma.counter = (uint32_t)(1047u - size);
  HAL_UARTEx_RxEventCallback(&huart2, (uint16_t)size); step();
}
static void wait_reply(unsigned int previous)
{
  test_dwt.CYCCNT += 2000u; step();
  assert(tx_count == previous + 1u && modbus_crc16(tx, tx_size) == 0);
}

int main(void)
{
  uint8_t query[14] = {MODBUS_ADDRESS, 0x10, 0, 0xe6, 0, 3};
  uint8_t control[20] = {MODBUS_ADDRESS, 0x10, 0, 0xe6, 0, 4, 0, 0, 0, 8};
  uint8_t image[UPGRADE_HEADER_SIZE + 71u] = {0}, frame[1046];
  sha256_ctx hash;
  uint32_t key = 0x12345678u;
  size_t i, offset, length;
  unsigned int number, before;

  memset(flash, 0xff, sizeof(flash));
  assert(Modbus_StartReceive() == HAL_OK); boot_log_start(); Upgrade_Init();
  assert(modbus_port_get_ota_status() == MODBUS_STATUS_DAMAGED);
  memcpy(image, "1.2", 3);
  for (i = UPGRADE_HEADER_SIZE; i < sizeof(image); ++i) image[i] = (uint8_t)i;
  put32le(image + UPGRADE_HEADER_SIZE, 0x20008000u);
  put32le(image + UPGRADE_HEADER_SIZE + 4u, MSP1_ADDR + 17u);
  sha256_init(&hash); sha256_update(&hash, image + UPGRADE_HEADER_SIZE, 71);
  sha256_final(&hash, image + 1024);

  seal(query, sizeof(query)); before = tx_count; feed(query, sizeof(query)); wait_reply(before);
  assert(tx_size == 14u && tx[7] == MODBUS_STATUS_DAMAGED);
  put16(control + 10, 1); put32(control + 12, key); put16(control + 16, 1);
  seal(control, sizeof(control)); before = tx_count; feed(control, sizeof(control));
  assert(erase_count == 2 && modbus_port_get_ota_status() == MODBUS_STATUS_DOWNLOADING);
  wait_reply(before); assert(tx_size == 8u && tx[1] == MODBUS_FUNCTION_CONTROL);

  for (offset = 0, number = 0; offset < sizeof(image); offset += length, ++number)
  {
    length = sizeof(image) - offset; if (length > 1024u) length = 1024u;
    frame[0] = MODBUS_ADDRESS; frame[1] = 0xaa; put16(frame + 2, (uint16_t)(length + 22u));
    put16(frame + 4, 1); put32(frame + 6, key); put16(frame + 10, (uint16_t)length);
    put16(frame + 12, 2); put16(frame + 14, (uint16_t)number);
    memcpy(frame + 16, image + offset, length);
    put32(frame + 16 + length, modbus_crc32(image + offset, length));
    seal(frame, length + 22u);
    before = tx_count; feed(frame, 13); feed(frame + 13, length + 9u); wait_reply(before);
    assert(tx_size == 7u && tx[2] == MODBUS_ACK_OK && tx[4] == number);
  }
  assert(number == 3 && !memcmp(flash + UPGRADE_APP_DATA_ADDRESS - UPGRADE_FLASH_BASE, image, sizeof(image)));
  assert(flash[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE] == 0x32 && jumps == 0);
  test_dwt.CYCCNT += 2000u; step(); assert(jumps == 1);
  assert(strstr(log_text, "USART2 OTA; USART1 log") != NULL);
  assert(strstr(log_text, "Erase OK; receiving") != NULL);
  assert(strstr(log_text, "RX packet/last 0/2") != NULL);
  assert(strstr(log_text, "SHA256 PASS") != NULL);
  assert(strstr(log_text, "Record saved; OTA complete") != NULL);
  assert(strstr(log_text, "Jump APP 08004800") != NULL);
  puts("DMA-to-Modbus-to-single-partition OTA integration passed");
  return 0;
}
