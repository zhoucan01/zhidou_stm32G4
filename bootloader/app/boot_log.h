#ifndef BOOT_LOG_H
#define BOOT_LOG_H
#include <stdint.h>

/* USART1 PB6/PB7, 115200 8N1. USART2/RS485 carries the binary OTA protocol.
 * Normal context only; no printf, Flash writes or transmission on USART2.
 */
#ifndef BOOT_LOG_ENABLED
#define BOOT_LOG_ENABLED 1
#endif
#define BOOT_LOG_NONE 0xffffffffu

#if BOOT_LOG_ENABLED
extern volatile uint32_t boot_log_drops;
void boot_log_start(void);
void boot_notice(const char *text, uint32_t first, uint32_t second);
#else
#define boot_log_start() ((void)0)
#define boot_notice(text, first, second) ((void)0)
#endif
#endif
