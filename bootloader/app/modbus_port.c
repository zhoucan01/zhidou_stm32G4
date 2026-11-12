#include "modbus_port.h"
#include "usart.h"
#include "error_trace.h"
#include "boot_log.h"
#include <string.h>

#define FRAME_SIZE (MODBUS_MAX_PAYLOAD + 22u)
#define DMA_SIZE (FRAME_SIZE + 1u)
#define CHUNK_SLOTS 5u
#define FRAME_SLOTS 4u
#define RESPONSE_SLOTS 5u
#define RX_TIMEOUT_US 3500u
#define BUS_IDLE_US 2000u

typedef struct
{
  uint8_t data[DMA_SIZE];
  uint16_t length;
  uint32_t tick;
} rx_chunk_t;

typedef struct
{
  uint8_t function;
  union
  {
    modbus_10_data_t control;
    modbus_aa_data_t packet;
  } decoded;
  uint8_t firmware[MODBUS_MAX_PAYLOAD];
} rx_frame_t;

typedef struct
{
  uint8_t data[MODBUS_RESPONSE_CAPACITY];
  uint8_t length;
  uint8_t reset;
} response_t;

static rx_chunk_t chunks[CHUNK_SLOTS];
static volatile uint8_t chunk_head, chunk_tail;
static volatile uint32_t receive_faults, chunk_drops;
static volatile uint8_t rx_paused, rx_recovery_pending;
static rx_frame_t frames[FRAME_SLOTS];
static uint8_t frame_head, frame_tail;
static response_t responses[RESPONSE_SLOTS];
static uint8_t response_head, response_tail;
static uint8_t query_received;
static uint8_t assembly[FRAME_SIZE];
static uint16_t assembly_size;
static uint32_t oversize_remaining, last_rx_tick;
static modbus_t context = {0, MODBUS_STATUS_FORBIDDEN};
static uint16_t registers[4];
static modbus_stats_t stats;
static int (*version_reader)(uint16_t, uint8_t *, uint8_t);
static uint8_t version_mode;

#if BOOT_LOG_ENABLED
static uint8_t log_ready;
volatile uint32_t boot_log_drops;

static uint16_t log_number(uint8_t *out, uint16_t used, uint32_t value)
{
  uint8_t digits[10];
  unsigned int count = 0u;
  do { digits[count++] = (uint8_t)('0' + value % 10u); value /= 10u; } while(value);
  while(count) out[used++] = digits[--count];
  return used;
}

void boot_notice(const char *text, uint32_t first, uint32_t second)
{
  uint8_t line[80] = "[BOOT] ";
  uint16_t used = 7u;
  if(!log_ready || __get_PRIMASK() != 0u) return;
#if defined(__CORTEX_M)
  if(__get_IPSR() != 0u) return;
#endif
  while(*text && used < 55u) line[used++] = (uint8_t)*text++;
  if(first != BOOT_LOG_NONE)
  {
    line[used++] = ' ';
    used = log_number(line, used, first);
  }
  if(second != BOOT_LOG_NONE)
  {
    line[used++] = '/';
    used = log_number(line, used, second);
  }
  line[used++] = '\r'; line[used++] = '\n';
  if(HAL_UART_Transmit(&huart1, line, used, 20u) != HAL_OK) ++boot_log_drops;
}

void boot_log_start(void)
{
  log_ready = 1u;
  boot_notice("USART2 OTA; USART1 log", BOOT_LOG_NONE, BOOT_LOG_NONE);
}
#endif

void modbus_port_set_version_reader(int (*reader)(uint16_t, uint8_t *, uint8_t), uint8_t mode)
{
  version_reader = reader;
  version_mode = mode;
}

static uint8_t next_index(uint8_t index, uint8_t size)
{
  return (uint8_t)((index + 1u) % size);
}

static uint16_t be16(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static int queue_response(const uint8_t *data, uint8_t size, uint8_t reset)
{
  uint8_t next = next_index(response_tail, RESPONSE_SLOTS);
  if(next == response_head)
  {
    ++stats.response_overflows;
    return 0;
  }
  memcpy(responses[response_tail].data, data, size);
  responses[response_tail].length = size;
  responses[response_tail].reset = reset;
  response_tail = next;
  return 1;
}

int modbus_set_ack(uint8_t code, uint16_t packet)
{
  uint8_t reply[7];
  int size = modbus_build_aa_reply(&context, (modbus_ack_t)code, packet, reply, sizeof(reply));
  return size < 0 ? -1 : queue_response(reply, (uint8_t)size, 0u);
}

static int queue_control_reply(uint8_t query, uint8_t reset)
{
  uint8_t reply[MODBUS_RESPONSE_CAPACITY];
  int size = modbus_build_10_reply(&context, query, reply, sizeof(reply));
  return queue_response(reply, (uint8_t)size, reset);
}

int modbus_set_10_reply(uint8_t is_query)
{
  return queue_control_reply(is_query, 0u);
}

static void store_frame(uint16_t size)
{
  uint8_t next = next_index(frame_tail, FRAME_SLOTS);
  rx_frame_t *slot = &frames[frame_tail];
  if(assembly[1] == MODBUS_FUNCTION_VERSION)
  {
    uint8_t reply[MODBUS_RESPONSE_CAPACITY];
    uint16_t offset = be16(assembly + 2), count = be16(assembly + 4), crc;
    if(size != 8u || count == 0u || count > MODBUS_VERSION_CHUNK ||
       offset >= MODBUS_VERSION_SIZE || count > MODBUS_VERSION_SIZE - offset || version_reader == NULL)
    {
      ++stats.frame_errors;
      return;
    }
    if(version_reader(offset, reply + 6, (uint8_t)count) != 1)
      return;
    reply[0] = MODBUS_ADDRESS; reply[1] = MODBUS_FUNCTION_VERSION;
    reply[2] = version_mode; reply[3] = (uint8_t)(offset >> 8);
    reply[4] = (uint8_t)offset; reply[5] = (uint8_t)count;
    crc = modbus_crc16(reply, count + 6u);
    reply[count + 6u] = (uint8_t)crc; reply[count + 7u] = (uint8_t)(crc >> 8);
    query_received = 1u;
    boot_notice("Version offset/count", offset, count);
    (void)queue_response(reply, (uint8_t)(count + 8u), 0u);
    return;
  }
  if(assembly[1] == MODBUS_FUNCTION_CONTROL)
  {
    modbus_10_data_t control;
    if(modbus_read_10(&context, assembly, size, &control) != 1)
    {
      ++stats.frame_errors;
      return;
    }
    if(control.is_query)
    {
      query_received = 1u;
      queue_control_reply(1u, 0u);
      return;
    }
    registers[1] = (uint16_t)(control.key >> 16);
    registers[2] = (uint16_t)control.key;
    registers[3] = control.operation;
    if(control.operation == 0u)
    {
      queue_control_reply(0u, 0u);
      return;
    }
    else if(control.operation == 2u)
    {
      boot_notice("Reset requested", BOOT_LOG_NONE, BOOT_LOG_NONE);
      queue_control_reply(0u, 1u);
      return;
    }
    if(next == frame_head)
    {
      ++stats.frame_overflows;
      return;
    }
    slot->function = MODBUS_FUNCTION_CONTROL;
    slot->decoded.control = control;
  }
  else
  {
    if(next == frame_head)
    {
      ++stats.frame_overflows;
      return;
    }
    if(modbus_read_aa(&context, assembly, size, &slot->decoded.packet,
                       slot->firmware, sizeof(slot->firmware)) != 1)
    {
      ++stats.frame_errors;
      return;
    }
    slot->function = MODBUS_FUNCTION_DATA;
  }
  frame_tail = next;
}

static void assemble_byte(uint8_t byte)
{
  uint32_t expected;
  uint16_t count, received;
  if(oversize_remaining != 0u)
  {
    --oversize_remaining;
    return;
  }
  assembly[assembly_size++] = byte;
  while(assembly_size != 0u)
  {
    if(assembly[0] != MODBUS_ADDRESS)
      goto skip;
    if(assembly_size < 2u)
      return;
    if(assembly[1] == MODBUS_FUNCTION_VERSION)
      expected = 8u;
    else if(assembly[1] == MODBUS_FUNCTION_CONTROL)
    {
      if(assembly_size < 6u)
        return;
      count = be16(assembly + 4);
      if(be16(assembly + 2) != MODBUS_REGISTER_START || (count != 3u && count != 4u))
        goto skip;
      expected = count == 3u ? 14u : 20u;
    }
    else if(assembly[1] == MODBUS_FUNCTION_DATA)
    {
      if(assembly_size < 4u)
        return;
      expected = be16(assembly + 2);
      if(expected < 23u)
        goto skip;
      if(assembly_size < 12u)
        return;
      if((uint32_t)be16(assembly + 10) + 22u != expected)
        goto skip;
      if(expected > FRAME_SIZE)
      {
        if(assembly_size < 16u)
          return;
        (void)modbus_set_ack(MODBUS_ACK_TOO_LONG, be16(assembly + 14));
        oversize_remaining = expected - assembly_size;
        assembly_size = 0;
        return;
      }
    }
    else
      goto skip;
    if(assembly_size < expected)
      return;
    received = (uint16_t)(assembly[expected - 2u] | ((uint16_t)assembly[expected - 1u] << 8));
    if(modbus_crc16(assembly, expected - 2u) != received)
    {
      ++stats.crc16_errors;
      boot_notice("RX CRC16 error", stats.crc16_errors, BOOT_LOG_NONE);
      if(assembly[1] == MODBUS_FUNCTION_DATA)
        (void)modbus_set_ack(MODBUS_ACK_FRAME_CRC, be16(assembly + 14));
      goto skip;
    }
    store_frame((uint16_t)expected);
    assembly_size = (uint16_t)(assembly_size - expected);
    memmove(assembly, assembly + expected, assembly_size);
    continue;
skip:
    ++stats.frame_errors;
    --assembly_size;
    memmove(assembly, assembly + 1, assembly_size);
  }
}

static HAL_StatusTypeDef start_dma(void)
{
  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, chunks[chunk_tail].data, DMA_SIZE);
  if(status == HAL_OK)
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  return status;
}

HAL_StatusTypeDef Modbus_StartReceive(void)
{
  if(SystemCoreClock < 1000000u || huart2.Init.BaudRate == 0u ||
     (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u)
    return HAL_ERROR;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  if((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u)
    return HAL_ERROR;
  last_rx_tick = DWT->CYCCNT;
  return start_dma();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  uint8_t next;
  HAL_StatusTypeDef status;
  if(huart != &huart2 || rx_paused || rx_recovery_pending)
    return;
  next = next_index(chunk_tail, CHUNK_SLOTS);
  if(size == 0u || size > DMA_SIZE)
    ++receive_faults;
  else if(next == chunk_head)
    ++chunk_drops;
  else
  {
    chunks[chunk_tail].length = size;
    chunks[chunk_tail].tick = DWT->CYCCNT;
    __DMB();
    chunk_tail = next;
  }
  status = start_dma();
  if(status == HAL_BUSY)
  {
    ++receive_faults;
    rx_recovery_pending = 1u;
    return;
  }
  if(status != HAL_OK)
    /* 0x2002: DMA/IDLE 回调中重新启动 USART2 DMA 失败。 */
    ERROR_HANDLER_FROM(ERROR_ID_RX_EVENT_REARM, "USART2 DMA rearm after RX event", status);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart2 && !rx_paused)
  {
    /* HAL may still own RX here. Recover only after its IRQ handler returns. */
    ++receive_faults;
    rx_recovery_pending = 1u;
  }
}

static int elapsed_us(uint32_t since, uint32_t us)
{
  return (uint32_t)(DWT->CYCCNT - since) >= (SystemCoreClock / 1000000u) * us;
}

static int uart_active(void)
{
  return __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET ||
         __HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET ||
         __HAL_UART_GET_FLAG(&huart2, UART_FLAG_BUSY) != RESET ||
         __HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET ||
         __HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE) != RESET ||
         __HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE) != RESET ||
         __HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE) != RESET;
}

static void send_pending(void)
{
  uint32_t mask;
  HAL_StatusTypeDef result, status;
  response_t *reply;
  uint8_t reset;
  if(rx_paused || response_head == response_tail || assembly_size != 0u || oversize_remaining != 0u)
    return;
  mask = __get_PRIMASK();
  __disable_irq();
  if(receive_faults != 0u || chunk_drops != 0u || chunk_head != chunk_tail ||
      __HAL_DMA_GET_COUNTER(huart2.hdmarx) != DMA_SIZE || uart_active() ||
      !elapsed_us(last_rx_tick, BUS_IDLE_US))
  {
    __set_PRIMASK(mask);
    return;
  }
  status = HAL_UART_AbortReceive(&huart2);
  if(status != HAL_OK)
    /* 0x2004: 发送前停止 USART2 接收失败。 */
    ERROR_HANDLER_FROM(ERROR_ID_TX_ABORT_RX, "Abort USART2 RX before response", status);
  CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RE);
  __set_PRIMASK(mask);
  reply = &responses[response_head];
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  result = HAL_UART_Transmit(&huart2, reply->data, reply->length, 20u);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  mask = __get_PRIMASK();
  __disable_irq();
  __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  SET_BIT(huart2.Instance->CR1, USART_CR1_RE);
  last_rx_tick = DWT->CYCCNT;
  status = start_dma();
  if(status != HAL_OK)
    /* 0x2005: 应答发送结束后重新启动 USART2 DMA 失败。 */
    ERROR_HANDLER_FROM(ERROR_ID_TX_REARM_RX, "USART2 DMA rearm after response", status);
  __set_PRIMASK(mask);
  if(result != HAL_OK)
  {
    ++stats.transmit_errors;
    return;
  }
  reset = reply->reset;
  response_head = next_index(response_head, RESPONSE_SLOTS);
  if(reset)
  {
    boot_notice("Reset ACK sent", BOOT_LOG_NONE, BOOT_LOG_NONE);
    NVIC_SystemReset();
  }
}

/* 清理超时半帧；只登记错误应答，仍由主循环统一发送。 */
static void expire_partial_frame(void)
{
  if(assembly_size >= 16u && assembly[0] == MODBUS_ADDRESS && assembly[1] == MODBUS_FUNCTION_DATA)
    (void)modbus_set_ack(MODBUS_ACK_RX_TIMEOUT, be16(assembly + 14));
  assembly_size = 0u;
  oversize_remaining = 0u;
  ++stats.timeouts;
}

void modbusdeal(void)
{
  static uint32_t logged_timeouts;
  uint32_t mask, faults, drops;
  uint16_t i;
  uint8_t head;
  mask = __get_PRIMASK();
  __disable_irq();
  if(rx_recovery_pending && !rx_paused)
  {
    HAL_StatusTypeDef status;
    CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RE);
    status = HAL_UART_AbortReceive(&huart2);
    if(status == HAL_OK)
    {
      SET_BIT(huart2.Instance->CR1, USART_CR1_RE);
      status = start_dma();
    }
    if(status == HAL_OK)
      rx_recovery_pending = 0u;
    else if(status != HAL_BUSY)
      ERROR_HANDLER_FROM(ERROR_ID_RX_ERROR_REARM, "USART2 RX recovery in main", status);
    if(rx_recovery_pending)
    {
      __set_PRIMASK(mask);
      return;
    }
  }
  faults = receive_faults;
  drops = chunk_drops;
  receive_faults = 0u;
  chunk_drops = 0u;
  if(faults != 0u || drops != 0u)
  {
    chunk_head = chunk_tail;
    assembly_size = 0u;
    oversize_remaining = 0u;
    last_rx_tick = DWT->CYCCNT;
  }
  __set_PRIMASK(mask);
  if(faults) boot_notice("RX recovered; errors", faults, BOOT_LOG_NONE);
  stats.receive_errors += faults;
  stats.chunk_overflows += drops;
  while((head = chunk_head) != chunk_tail)
  {
    if(receive_faults != 0u || chunk_drops != 0u)
      return;
    __DMB();
    if(assembly_size != 0u || oversize_remaining != 0u)
    {
      /* 回调时间是块结束时间。扣除本块 8N1 传输时间，避免将长块误判为静默。
       * 使用接收时间戳而非主循环执行时间，兼容积压块和 CYCCNT 回绕。
       */
      uint64_t wire_cycles = ((uint64_t)SystemCoreClock * 10u * chunks[head].length +
                              huart2.Init.BaudRate - 1u) / huart2.Init.BaudRate;
      uint64_t timeout_cycles = (SystemCoreClock / 1000000u) * RX_TIMEOUT_US;
      if((uint32_t)(chunks[head].tick - last_rx_tick) >= wire_cycles + timeout_cycles)
        expire_partial_frame();
    }
    last_rx_tick = chunks[head].tick;
    for(i = 0; i < chunks[head].length; ++i)
      assemble_byte(chunks[head].data[i]);
    __DMB();
    chunk_head = next_index(head, CHUNK_SLOTS);
  }
  mask = __get_PRIMASK();
  __disable_irq();
  if(receive_faults == 0u && chunk_drops == 0u && chunk_head == chunk_tail &&
      __HAL_DMA_GET_COUNTER(huart2.hdmarx) == DMA_SIZE && !uart_active() &&
      (assembly_size != 0u || oversize_remaining != 0u) && elapsed_us(last_rx_tick, RX_TIMEOUT_US))
  {
    expire_partial_frame();
  }
  __set_PRIMASK(mask);
  if(logged_timeouts != stats.timeouts)
  {
    logged_timeouts = stats.timeouts;
    boot_notice("RX timeout", logged_timeouts, BOOT_LOG_NONE);
  }
  send_pending();
}

int modbus_port_get_10(modbus_10_data_t *out)
{
  if(out == NULL)
    return -1;
  if(frame_head == frame_tail || frames[frame_head].function != MODBUS_FUNCTION_CONTROL)
    return 0;
  *out = frames[frame_head].decoded.control;
  frame_head = next_index(frame_head, FRAME_SLOTS);
  return 1;
}

int modbus_port_get_aa(modbus_aa_data_t *out, uint8_t *firmware, size_t capacity)
{
  rx_frame_t *slot;
  if(out == NULL || firmware == NULL)
    return -1;
  if(frame_head == frame_tail || frames[frame_head].function != MODBUS_FUNCTION_DATA)
    return 0;
  slot = &frames[frame_head];
  if(capacity < slot->decoded.packet.firmware_size)
    return -1;
  *out = slot->decoded.packet;
  memcpy(firmware, slot->firmware, out->firmware_size);
  frame_head = next_index(frame_head, FRAME_SLOTS);
  return 1;
}

int modbus_port_take_query(void)
{
  int result = query_received;
  query_received = 0u;
  return result;
}

static int rx_idle_locked(void)
{
  return !rx_paused && !query_received && response_head == response_tail &&
         receive_faults == 0u && chunk_drops == 0u &&
         chunk_head == chunk_tail && frame_head == frame_tail &&
         assembly_size == 0u && oversize_remaining == 0u &&
         __HAL_DMA_GET_COUNTER(huart2.hdmarx) == DMA_SIZE && !uart_active() &&
         elapsed_us(last_rx_tick, BUS_IDLE_US);
}

int modbus_port_rx_idle(void)
{
  uint32_t mask = __get_PRIMASK();
  int idle;
  __disable_irq();
  idle = rx_idle_locked();
  __set_PRIMASK(mask);
  return idle;
}

static HAL_StatusTypeDef pause_rx_locked(void)
{
  HAL_StatusTypeDef result;
  rx_paused = 1u;
  CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RE);
  result = HAL_UART_AbortReceive(&huart2);
  if(result == HAL_OK)
  {
    chunk_head = chunk_tail = 0u;
    frame_head = frame_tail = 0u;
    receive_faults = chunk_drops = 0u;
    rx_recovery_pending = 0u;
    assembly_size = 0u;
    oversize_remaining = 0u;
    query_received = 0u;
  }
  return result;
}

HAL_StatusTypeDef modbus_port_pause_rx(void)
{
  uint32_t mask = __get_PRIMASK();
  HAL_StatusTypeDef result;
  __disable_irq();
  result = pause_rx_locked();
  __set_PRIMASK(mask);
  return result;
}

HAL_StatusTypeDef modbus_port_pause_rx_if_idle(void)
{
  uint32_t mask = __get_PRIMASK();
  HAL_StatusTypeDef result;
  __disable_irq();
  result = rx_idle_locked() ? pause_rx_locked() : HAL_BUSY;
  __set_PRIMASK(mask);
  return result;
}

HAL_StatusTypeDef modbus_port_resume_rx(void)
{
  uint32_t mask = __get_PRIMASK();
  HAL_StatusTypeDef result;
  __disable_irq();
  __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  SET_BIT(huart2.Instance->CR1, USART_CR1_RE);
  last_rx_tick = DWT->CYCCNT;
  result = start_dma();
  if(result == HAL_OK)
    rx_paused = 0u;
  __set_PRIMASK(mask);
  return result;
}

int modbus_port_read_registers(uint16_t start, uint16_t count, uint16_t *out)
{
  uint16_t offset, i;
  if(out == NULL || count == 0u || start < MODBUS_REGISTER_START)
    return -1;
  offset = (uint16_t)(start - MODBUS_REGISTER_START);
  if(offset >= 4u || count > 4u - offset)
    return -1;
  for(i = 0; i < count; ++i)
    out[i] = offset + i == 0u ? context.status : registers[offset + i];
  return 1;
}

void modbus_port_set_ota_status(uint16_t status) { context.status = status; }
uint16_t modbus_port_get_ota_status(void) { return context.status; }
void modbus_port_set_session_key(uint32_t key) { context.key = key; }
int modbus_reply_pending(void) { return response_head != response_tail; }
int modbus_get_stats(modbus_stats_t *out)
{
  if(out == NULL)
    return -1;
  *out = stats;
  return 1;
}
