#ifndef MODBUS_STREAM_H
#define MODBUS_STREAM_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define MODBUS_STREAM_CAPACITY 256u
#define MODBUS_STREAM_INVALID_LENGTH 0xffffu
#define MODBUS_STREAM_TIMEOUT_MS 4u

/* Single consumer, task context. Input timestamps come from the RX callback,
 * not the time the task drains the queue. CRC covers the complete frame.
 */
typedef struct
{
  uint8_t data[MODBUS_STREAM_CAPACITY];
  uint16_t used;
  uint32_t last_tick, frame_errors, crc_errors, timeouts;
} modbus_stream_t;

static void ModbusStream_Feed(modbus_stream_t *stream, const uint8_t *data, uint16_t length,
                              uint32_t tick, uint32_t baud,
                              uint16_t (*frame_length)(const uint8_t *, uint16_t),
                              uint16_t (*crc16)(const uint8_t *, uint16_t),
                              void (*handle)(const uint8_t *, uint16_t))
{
  uint16_t i, expected, consumed;
  uint64_t wire_ms;
  if(data == NULL || length == 0u || baud == 0u) return;
  /* Account for the new block's 8N1 wire time before testing the silent gap.
   * Round up wire time and allow one tick for timestamp quantization, so a
   * continuous long block cannot be mistaken for a silent interval.
   */
  wire_ms = ((uint64_t)length * 10000u + baud - 1u) / baud;
  if(stream->used != 0u && (uint32_t)(tick - stream->last_tick) >= wire_ms + MODBUS_STREAM_TIMEOUT_MS + 1u)
  {
    stream->used = 0u;
    ++stream->timeouts;
  }
  stream->last_tick = tick;
  for(i = 0u; i < length; ++i)
  {
    if(stream->used == MODBUS_STREAM_CAPACITY)
    {
      stream->used = 0u;
      ++stream->frame_errors;
    }
    stream->data[stream->used++] = data[i];
    while(stream->used != 0u)
    {
      expected = frame_length(stream->data, stream->used);
      if(expected == MODBUS_STREAM_INVALID_LENGTH || expected > MODBUS_STREAM_CAPACITY)
      {
        ++stream->frame_errors;
        consumed = 1u;
      }
      else if(expected == 0u || stream->used < expected)
        break; /* Preserve the partial frame for the next RX block. */
      else if(crc16(stream->data, expected) != 0u)
      {
        ++stream->crc_errors;
        consumed = 1u;
      }
      else
      {
        handle(stream->data, expected);
        consumed = expected;
      }
      stream->used = (uint16_t)(stream->used - consumed);
      memmove(stream->data, stream->data + consumed, stream->used);
    }
  }
}

#endif
