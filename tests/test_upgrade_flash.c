#include <windows.h>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32g4xx_hal.h"
#include "upgrade.h"
#include "upgrade_flash.h"

uint32_t test_flash_size = 128u * 1024u;
FLASH_TypeDef test_flash = { FLASH_ACR_DCEN };
static uint8_t *memory;
static int unlocked, erase_calls, program_calls, fail_erase, fail_program, fail_lock;

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { assert(!unlocked); unlocked = 1; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { assert(unlocked); unlocked = 0; return fail_lock ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error)
{
  assert(unlocked && erase->TypeErase == FLASH_TYPEERASE_PAGES && erase->Banks == FLASH_BANK_1);
  assert(erase->Page >= 7u && erase->Page + erase->NbPages <= 64u);
  ++erase_calls;
  if (fail_erase) { *error = erase->Page; return HAL_ERROR; }
  memset(memory + erase->Page * FLASH_PAGE_SIZE, 0xff, erase->NbPages * FLASH_PAGE_SIZE);
  *error = 0xffffffffu;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t value)
{
  unsigned int i;
  assert(unlocked && type == FLASH_TYPEPROGRAM_DOUBLEWORD && !(address & 7u));
  assert(address >= UPGRADE_RECORD_ADDRESS && address + 8u <= UPGRADE_FLASH_END);
  ++program_calls;
  if (fail_program) return HAL_ERROR;
  for (i = 0; i < 8u; ++i)
  {
    uint8_t byte = (uint8_t)(value >> (8u * i));
    uint8_t *dest = memory + address - UPGRADE_FLASH_BASE + i;
    if ((*dest & byte) != byte) return HAL_ERROR;
    *dest &= byte;
  }
  return HAL_OK;
}

int main(void)
{
  uint8_t data[16], readback[16];
  unsigned int i;
  memory = VirtualAlloc((void *)(uintptr_t)UPGRADE_FLASH_BASE, test_flash_size,
                        MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
  assert(memory == (uint8_t *)(uintptr_t)UPGRADE_FLASH_BASE);
  memset(memory, 0, test_flash_size);
  assert(!Upgrade_FlashErase(UPGRADE_FLASH_BASE, FLASH_PAGE_SIZE));
  assert(!Upgrade_FlashErase(UPGRADE_RECORD_ADDRESS + 1u, FLASH_PAGE_SIZE));
  assert(!Upgrade_FlashErase(UPGRADE_RECORD_ADDRESS, FLASH_PAGE_SIZE + 1u));
  assert(erase_calls == 0);
  assert(Upgrade_FlashErase(UPGRADE_RECORD_ADDRESS, FLASH_PAGE_SIZE));
  assert(memory[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE] == 0xff);
  assert(memory[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE - 1u] == 0);
  assert(Upgrade_FlashErase(UPGRADE_APP_DATA_ADDRESS, UPGRADE_IMAGE_CAPACITY));
  for (i = 0; i < sizeof(data); ++i) data[i] = (uint8_t)(i + 1u);
  assert(!Upgrade_FlashProgram(UPGRADE_APP_DATA_ADDRESS + 1u, data, 8));
  assert(!Upgrade_FlashProgram(UPGRADE_APP_DATA_ADDRESS, data, 7));
  assert(!Upgrade_FlashProgram(UPGRADE_FLASH_END - 8u, data, 16));
  assert(program_calls == 0);
  assert(Upgrade_FlashProgram(UPGRADE_APP_DATA_ADDRESS, data, sizeof(data)));
  assert(program_calls == 2 && (test_flash.ACR & FLASH_ACR_DCEN));
  assert(Upgrade_FlashRead(UPGRADE_APP_DATA_ADDRESS, readback, sizeof(readback)));
  assert(!memcmp(data, readback, sizeof(data)));
  memset(data, 0xff, sizeof(data));
  assert(!Upgrade_FlashProgram(UPGRADE_APP_DATA_ADDRESS, data, sizeof(data)));
  fail_program = 1;
  data[0] = 0;
  assert(!Upgrade_FlashProgram(UPGRADE_APP_DATA_ADDRESS + 16u, data, 8));
  assert(!unlocked && (test_flash.ACR & FLASH_ACR_DCEN));
  fail_program = 0; fail_erase = 1;
  assert(!Upgrade_FlashErase(UPGRADE_RECORD_ADDRESS, FLASH_PAGE_SIZE) && !unlocked);
  fail_erase = 0; fail_lock = 1;
  assert(!Upgrade_FlashErase(UPGRADE_RECORD_ADDRESS, FLASH_PAGE_SIZE) && !unlocked);
  fail_lock = 0; test_flash_size = 64u * 1024u;
  assert(!Upgrade_FlashErase(UPGRADE_RECORD_ADDRESS, FLASH_PAGE_SIZE));
  assert(!Upgrade_FlashRead(UPGRADE_RECORD_ADDRESS, readback, sizeof(readback)));
  assert(VirtualFree(memory, 0, MEM_RELEASE));
  puts("STM32 flash adapter boundary and failure tests passed");
  return 0;
}
