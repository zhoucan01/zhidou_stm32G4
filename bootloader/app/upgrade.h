#ifndef UPGRADE_H
#define UPGRADE_H

#include <stdint.h>

#define UPGRADE_FLASH_BASE       0x08000000u
#define UPGRADE_BOOT_SIZE        0x00003800u
#define UPGRADE_RECORD_ADDRESS   0x08003800u
#define UPGRADE_APP_DATA_ADDRESS 0x08004000u
#define UPGRADE_HASH_ADDRESS     0x08004400u
#define MSP1_ADDR                0x08004800u
#define UPGRADE_FLASH_END        0x08020000u
#define UPGRADE_HEADER_SIZE      0x00000800u
#define UPGRADE_IMAGE_CAPACITY   (UPGRADE_FLASH_END - UPGRADE_APP_DATA_ADDRESS)
#define UPGRADE_APP_CAPACITY     (UPGRADE_FLASH_END - MSP1_ADDR)
#define UPGRADE_IMAGE_VALID      0x53484132u
#define UPGRADE_BOOT_WAIT_MS     4000u

/* The record is one little-endian doubleword: magic followed by raw APP length.
   The OTA stream is 1 KiB version (zero-padded), 1 KiB hash, then the raw APP.
   SHA-256 covers only the raw APP; the digest occupies the first 32 hash bytes. */
void Upgrade_Init(void);
void Upgrade_Poll(void);

#endif
