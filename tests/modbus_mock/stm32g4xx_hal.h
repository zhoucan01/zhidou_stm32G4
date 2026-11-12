#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>
typedef struct { uint32_t counter; } DMA_HandleTypeDef;
typedef struct { uint32_t CR1; } USART_TypeDef;
typedef struct { uint32_t BaudRate; } UART_InitTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; USART_TypeDef *Instance; UART_InitTypeDef Init; } UART_HandleTypeDef;
typedef struct { uint32_t DEMCR; } CoreDebug_Type;
typedef struct { uint32_t CTRL, CYCCNT; } DWT_Type;
typedef struct { uint32_t ACR; } FLASH_TypeDef;
typedef struct { uint32_t TypeErase, Banks, Page, NbPages; } FLASH_EraseInitTypeDef;
extern CoreDebug_Type test_core_debug;
extern DWT_Type test_dwt;
extern uint32_t SystemCoreClock, test_primask, test_flags;
extern uint32_t test_flash_size;
extern FLASH_TypeDef test_flash;
#define CoreDebug (&test_core_debug)
#define DWT (&test_dwt)
#define FLASH (&test_flash)
#define FLASH_SIZE test_flash_size
#define FLASH_PAGE_SIZE 0x800u
#define FLASH_FLAG_ALL_ERRORS 0x02u
#define FLASH_FLAG_EOP 0x01u
#define FLASH_TYPEERASE_PAGES 0u
#define FLASH_BANK_1 1u
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0u
#define FLASH_ACR_DCEN 1u
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define DWT_CTRL_NOCYCCNT_Msk 2u
#define DMA_IT_HT 1u
#define USART_CR1_RE 1u
#define UART_RXDATA_FLUSH_REQUEST 1u
#define UART_FLAG_IDLE 1u
#define UART_FLAG_RXNE 2u
#define UART_FLAG_BUSY 4u
#define UART_FLAG_ORE 8u
#define UART_FLAG_NE 16u
#define UART_FLAG_FE 32u
#define UART_FLAG_PE 64u
#define RESET 0u
#define GPIOA 0u
#define GPIO_PIN_1 2u
#define GPIO_PIN_SET 1u
#define GPIO_PIN_RESET 0u
#define HAL_GPIO_WritePin(port, pin, state) ((void)(port), (void)(pin), (void)(state))
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define __HAL_DMA_DISABLE_IT(dma, it) ((void)(dma), (void)(it))
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->counter)
#define __HAL_UART_GET_FLAG(uart, flag) ((void)(uart), (test_flags & (flag)))
#define __HAL_UART_SEND_REQ(uart, req) ((void)(uart), (void)(req))
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
#define __get_PRIMASK() (test_primask)
#define __disable_irq() (test_primask = 1u)
#define __set_PRIMASK(value) (test_primask = (value))
#define __DMB() ((void)0)
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define SET_BIT(reg, bits) ((reg) |= (bits))
#define READ_BIT(reg, bits) ((reg) & (bits))
#define __HAL_FLASH_CLEAR_FLAG(flags) ((void)(flags))
#define __HAL_FLASH_DATA_CACHE_DISABLE() CLEAR_BIT(FLASH->ACR, FLASH_ACR_DCEN)
#define __HAL_FLASH_DATA_CACHE_ENABLE() SET_BIT(FLASH->ACR, FLASH_ACR_DCEN)
#define __HAL_FLASH_DATA_CACHE_RESET() ((void)0)
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
void Error_Handler(void);
void NVIC_SystemReset(void);
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *, uint32_t *);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t, uint32_t, uint64_t);
#endif
