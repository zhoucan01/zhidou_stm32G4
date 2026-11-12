#include <assert.h>
#include <stdio.h>
#include "modbus.h"
#include "ota_app_bridge.h"
#include "modbus_stream.h"

static modbus_stream_t stream;
static unsigned replies, resets, business_frames;
static int transmit_ok = 1;

static uint16_t crc16(const uint8_t *data, uint16_t length)
{
  return modbus_crc16(data, length);
}

static void seal(uint8_t *data, uint16_t length)
{
  uint16_t crc = crc16(data, (uint16_t)(length - 2u));
  data[length - 2u] = (uint8_t)crc;
  data[length - 1u] = (uint8_t)(crc >> 8);
}

static int read_version(uint16_t offset, uint8_t *out, uint8_t count)
{
  (void)offset;
  memset(out, 0, count);
  out[0] = 'v';
  return 1;
}

static uint16_t frame_length(const uint8_t *data, uint16_t available)
{
  uint16_t ota;
  if(data[0] != 3u && data[0] != 0u) return MODBUS_STREAM_INVALID_LENGTH;
  if(available < 2u) return 0u;
  ota = OtaApp_RequestLength(data, available);
  if(ota != 0u) return ota;
  if(data[1] == 3u) return 8u;
  if(data[1] == 0x10u) return available < 7u ? 0u : (uint16_t)(9u + data[6]);
  return MODBUS_STREAM_INVALID_LENGTH;
}

static void handle(const uint8_t *data, uint16_t length)
{
  uint8_t out[40], reset;
  assert(crc16(data, length) == 0u);
  if(OtaApp_RequestLength(data, length) == 0u)
  {
    ++business_frames;
    return;
  }
  assert(OtaApp_Process(data, length, out, sizeof(out), read_version, &reset) > 0u);
  ++replies;
  if(reset && transmit_ok) ++resets;
}

static void feed(const uint8_t *data, uint16_t length, uint32_t tick)
{
  ModbusStream_Feed(&stream, data, length, tick, 1000000u, frame_length, crc16, handle);
}

static void clear(void)
{
  memset(&stream, 0, sizeof(stream));
  replies = resets = business_frames = 0u;
  transmit_ok = 1;
}

int main(void)
{
  uint8_t version[8] = {3, 0x11, 0, 0, 0, 32};
  uint8_t query[14] = {3, 0x10, 0, 0xe6, 0, 3};
  uint8_t reset[20] = {3, 0x10, 0, 0xe6, 0, 4, 0, 0, 0, 8, 0, 1, 0, 0, 0, 0, 0, 2};
  uint8_t business[11] = {3, 0x10, 0, 2, 0, 1, 2, 0, 4};
  uint8_t large[255] = {3, 0x10, 0, 0, 0, 123, 246};
  uint8_t joined[33];
  unsigned i;
  seal(version, 8); seal(query, 14); seal(reset, 20); seal(business, 11); seal(large, 255);

  /* Every split point must behave like a whole DMA/IDLE request. */
  for(i = 1; i < sizeof(version); ++i)
  {
    clear(); feed(version, (uint16_t)i, 10);
    assert(replies == 0u);
    feed(version + i, (uint16_t)(sizeof(version) - i), 11);
    assert(replies == 1u && stream.used == 0u);
  }
  for(i = 1; i < sizeof(reset); ++i)
  {
    clear(); feed(reset, (uint16_t)i, 10); assert(resets == 0u);
    feed(reset + i, (uint16_t)(sizeof(reset) - i), 11);
    assert(replies == 1u && resets == 1u && stream.used == 0u);
  }
  clear();
  for(i = 0; i < sizeof(query); ++i) feed(query + i, 1, 10);
  assert(replies == 1u);

  clear(); memcpy(joined, query, 14); memcpy(joined + 14, version, 8); memcpy(joined + 22, business, 11);
  feed(joined, sizeof(joined), 10);
  assert(replies == 2u && business_frames == 1u);
  clear(); feed(business, 5, 10); feed(business + 5, 6, 11);
  assert(business_frames == 1u && replies == 0u);
  clear(); feed(large, 8, 10); feed(large + 8, 247, 13);
  assert(business_frames == 1u && stream.timeouts == 0u);

  clear(); feed(reset, 10, 10); feed(reset + 10, 10, 20);
  assert(resets == 0u && stream.timeouts == 1u);
  feed(version, 8, 30); assert(replies == 1u);
  clear(); feed(query, 5, 0xfffffffeu); feed(query + 5, 9, 1u);
  assert(replies == 1u && stream.timeouts == 0u);

  clear(); reset[19] ^= 1u; feed(reset, 20, 10); feed(version, 8, 11);
  assert(resets == 0u && replies == 1u && stream.crc_errors > 0u);
  reset[19] ^= 1u;
  clear(); transmit_ok = 0; feed(reset, 10, 10); feed(reset + 10, 10, 11);
  assert(replies == 1u && resets == 0u);
  clear(); feed(query, 5, 10); stream.used = 0u; /* transport loss invalidates the tail */
  feed(query + 5, 9, 11); assert(replies == 0u);
  puts("APP streaming, fragmentation, timeout and business Modbus tests passed");
  return 0;
}
