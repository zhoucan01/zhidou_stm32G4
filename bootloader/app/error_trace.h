#ifndef ERROR_TRACE_H
#define ERROR_TRACE_H

#include <stdint.h>

typedef enum
{
  ERROR_ID_NONE = 0,
  ERROR_ID_HAL_CHECK = 0x1000,
  ERROR_ID_BOOT_RX_START = 0x2001,
  ERROR_ID_RX_EVENT_REARM = 0x2002,
  ERROR_ID_RX_ERROR_REARM = 0x2003,
  ERROR_ID_TX_ABORT_RX = 0x2004,
  ERROR_ID_TX_REARM_RX = 0x2005,
  ERROR_ID_UPGRADE_ACK_QUEUE = 0x3001,
  ERROR_ID_UPGRADE_CONTROL_QUEUE = 0x3002,
  ERROR_ID_UPGRADE_PAUSE_RX = 0x3003,
  ERROR_ID_UPGRADE_RESUME_RX = 0x3004,
  ERROR_ID_APP_JUMP_PAUSE_RX = 0x3005,
  ERROR_ID_UPGRADE_STATE = 0x3006,
  ERROR_ID_UNMARKED = 0xffff
} error_id_t;

/* Keil Watch: g_error_trace. The FIRST error survives nested/repeated failures.
 * Strings stay in Flash; no printf, UART transmission or Flash writes are used.
 * detail: HAL return value, queue result, or invalid upgrade state.
 */
typedef struct
{
  uint32_t code;
  const char *reason;
  const char *file;
  const char *function;
  uint32_t line;
  int32_t detail;
  uint32_t active_exception;
  uint32_t interrupt_mask;
} error_trace_t;

extern volatile error_trace_t g_error_trace;
void Error_TraceRecord(uint32_t code, const char *reason, int32_t detail,
                       const char *file, uint32_t line, const char *function);
void Error_HandlerFrom(uint32_t code, const char *reason, int32_t detail,
                       const char *file, uint32_t line, const char *function);

#define ERROR_HANDLER_FROM(code, reason, detail) \
  Error_HandlerFrom((code), (reason), (int32_t)(detail), __FILE__, __LINE__, __func__)

/* Evaluate each HAL operation once, preserving the actual failing return value. */
#define ERROR_CHECK_HAL(operation) do { \
  int32_t error_trace_status = (int32_t)(operation); \
  if(error_trace_status != (int32_t)HAL_OK) \
    ERROR_HANDLER_FROM(ERROR_ID_HAL_CHECK, #operation, error_trace_status); \
} while(0)

#endif
