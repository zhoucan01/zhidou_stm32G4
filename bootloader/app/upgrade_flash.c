#include "upgrade_flash.h"
#include "upgrade.h"
#include "stm32g4xx_hal.h"
#include <string.h>

static int valid_range(uint32_t address, size_t length)
{
  return address >= UPGRADE_RECORD_ADDRESS && address <= UPGRADE_FLASH_END &&
         length <= UPGRADE_FLASH_END - address && FLASH_SIZE >= UPGRADE_FLASH_END - UPGRADE_FLASH_BASE;
}

int Upgrade_FlashRead(uint32_t address, void *data, size_t length)
{
  if(data == NULL || !valid_range(address, length))
    return 0;
  memcpy(data, (const void *)(uintptr_t)address, length);
  return 1;
}

int Upgrade_FlashErase(uint32_t address, uint32_t length)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error, i;
  uint8_t buffer[32];
  HAL_StatusTypeDef status, lock_status;
  if(!valid_range(address, length) || length == 0u ||
      (address - UPGRADE_FLASH_BASE) % FLASH_PAGE_SIZE != 0u || length % FLASH_PAGE_SIZE != 0u)
    return 0;
  if(HAL_FLASH_Unlock() != HAL_OK)
    return 0;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS | FLASH_FLAG_EOP);
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = (address - UPGRADE_FLASH_BASE) / FLASH_PAGE_SIZE;
  erase.NbPages = length / FLASH_PAGE_SIZE;
  status = HAL_FLASHEx_Erase(&erase, &page_error);
  lock_status = HAL_FLASH_Lock();
  if(status != HAL_OK || lock_status != HAL_OK)
    return 0;
  for(i = 0u; i < length; i += sizeof(buffer))
  {
    size_t j;
    if(!Upgrade_FlashRead(address + i, buffer, sizeof(buffer)))
      return 0;
    for(j = 0u; j < sizeof(buffer); ++j)
      if(buffer[j] != 0xffu)
        return 0;
  }
  return 1;
}

int Upgrade_FlashProgram(uint32_t address, const uint8_t *data, size_t length)
{
  size_t i;
  uint8_t verify[8];
  HAL_StatusTypeDef status = HAL_OK, lock_status;
  uint32_t cache_enabled;
  if(data == NULL || !valid_range(address, length) || length == 0u ||
      (address & 7u) != 0u || (length & 7u) != 0u)
    return 0;
  if(HAL_FLASH_Unlock() != HAL_OK)
    return 0;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS | FLASH_FLAG_EOP);
  for(i = 0u; i < length; i += 8u)
  {
    unsigned int j;
    uint64_t value = 0u;
    for(j = 0u; j < 8u; ++j)
      value |= (uint64_t)data[i + j] << (8u * j);
    if(value != UINT64_MAX)
    {
      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + (uint32_t)i, value);
      if(status != HAL_OK)
        break;
    }
  }
  lock_status = HAL_FLASH_Lock();
  cache_enabled = READ_BIT(FLASH->ACR, FLASH_ACR_DCEN);
  if(cache_enabled != 0u)
    __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  if(cache_enabled != 0u)
    __HAL_FLASH_DATA_CACHE_ENABLE();
  if(status != HAL_OK || lock_status != HAL_OK)
    return 0;
  for(i = 0u; i < length; i += sizeof(verify))
  {
    if(!Upgrade_FlashRead(address + (uint32_t)i, verify, sizeof(verify)) ||
        memcmp(verify, data + i, sizeof(verify)) != 0)
      return 0;
  }
  return 1;
}
