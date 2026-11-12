#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "error_trace.h"

uint32_t test_primask;
static unsigned int handler_calls, operation_calls;
void Error_Handler(void) { ++handler_calls; }

static HAL_StatusTypeDef failing_operation(void)
{
  ++operation_calls;
  return HAL_BUSY;
}

int main(void)
{
  uint32_t expected_line;
  test_primask = 1u;
  expected_line = __LINE__ + 1u;
  ERROR_CHECK_HAL(failing_operation());
  assert(handler_calls == 1u && operation_calls == 1u);
  assert(g_error_trace.code == ERROR_ID_HAL_CHECK && g_error_trace.detail == HAL_BUSY);
  assert(g_error_trace.line == expected_line && g_error_trace.interrupt_mask == 1u);
  assert(test_primask == 1u);
  assert(strstr(g_error_trace.file, "test_error_trace.c") != NULL);
  assert(strcmp(g_error_trace.function, "main") == 0);
  assert(strcmp(g_error_trace.reason, "failing_operation()") == 0);
  /* A nested or repeated failure must retain the original source and detail. */
  ERROR_HANDLER_FROM(ERROR_ID_UPGRADE_STATE, "second error", 123);
  assert(handler_calls == 2u && g_error_trace.code == ERROR_ID_HAL_CHECK);
  assert(g_error_trace.line == expected_line && g_error_trace.detail == HAL_BUSY);
  memset((void *)&g_error_trace, 0, sizeof(g_error_trace));
  test_primask = 0u;
  ERROR_CHECK_HAL(HAL_OK);
  assert(g_error_trace.code == ERROR_ID_NONE && handler_calls == 2u);
  ERROR_HANDLER_FROM(ERROR_ID_UPGRADE_ACK_QUEUE, "queue full", 0);
  assert(g_error_trace.code == ERROR_ID_UPGRADE_ACK_QUEUE && g_error_trace.detail == 0);
  assert(test_primask == 0u && handler_calls == 3u);
  puts("Error source, first-failure preservation and HAL result tests passed");
  return 0;
}
