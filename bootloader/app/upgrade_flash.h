#ifndef UPGRADE_FLASH_H
#define UPGRADE_FLASH_H

#include <stddef.h>
#include <stdint.h>

/* 以下接口成功返回 1，参数越界或硬件操作失败返回 0；禁止访问 Bootloader 区。 */
int Upgrade_FlashRead(uint32_t address, void *data, size_t length);
/* 地址和长度均须按 STM32G4 Flash 页对齐，擦除后逐块读回验证。 */
int Upgrade_FlashErase(uint32_t address, uint32_t length);
/* 地址和长度均须为 8 字节对齐；调用方负责跨包缓存及末包补齐。 */
int Upgrade_FlashProgram(uint32_t address, const uint8_t *data, size_t length);

#endif
