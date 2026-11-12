#include "main.h"
#include "error_trace.h"

volatile error_trace_t g_error_trace;

void Error_TraceRecord(uint32_t code, const char *reason, int32_t detail,
                       const char *file, uint32_t line, const char *function)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if(g_error_trace.code == ERROR_ID_NONE)
  {
    g_error_trace.reason = reason;
    g_error_trace.file = file;
    g_error_trace.function = function;
    g_error_trace.line = line;
    g_error_trace.detail = detail;
#if defined(__CORTEX_M)
    g_error_trace.active_exception = __get_IPSR();
#else
    g_error_trace.active_exception = 0u;
#endif
    g_error_trace.interrupt_mask = mask;
    __DMB();
    g_error_trace.code = code;
  }
  __set_PRIMASK(mask);
}

void Error_HandlerFrom(uint32_t code, const char *reason, int32_t detail,
                       const char *file, uint32_t line, const char *function)
{
  Error_TraceRecord(code, reason, detail, file, line, function);
  Error_Handler();
}
