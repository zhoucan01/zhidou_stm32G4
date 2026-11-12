#include "modbus.h"
#include <string.h>

/* 协议层：字段解析、CRC 校验和应答组帧，不访问串口或执行升级操作。 */
#define FRAME_SIZE (MODBUS_MAX_PAYLOAD + 22u)

static uint16_t be16(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t be32(const uint8_t *p)
{
  return ((uint32_t)be16(p) << 16) | be16(p + 2);
}

static void put16(uint8_t *p, uint16_t value)
{
  p[0] = (uint8_t)(value >> 8);
  p[1] = (uint8_t)value;
}

static void put32(uint8_t *p, uint32_t value)
{
  put16(p, (uint16_t)(value >> 16));
  put16(p + 2, (uint16_t)value);
}

/** @brief 计算 CRC16/MODBUS；组帧时以低字节在前的顺序追加结果。 */
uint16_t modbus_crc16(const uint8_t *data, size_t size)
{
  uint16_t crc = 0xffffu;
  unsigned int bit;
  while(size-- != 0u)
  {
    crc ^= *data++;
    for(bit = 0; bit < 8u; ++bit)
      crc = (uint16_t)((crc >> 1) ^ ((crc & 1u) ? 0xa001u : 0u));
  }
  return crc;
}

/** @brief 计算固件载荷的 CRC32/ISO-HDLC，不包含协议头及帧尾。 */
uint32_t modbus_crc32(const uint8_t *data, size_t size)
{
  uint32_t crc = 0xffffffffu;
  unsigned int bit;
  while(size-- != 0u)
  {
    crc ^= *data++;
    for(bit = 0; bit < 8u; ++bit)
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
  }
  return crc ^ 0xffffffffu;
}

int modbus_check_aa_crc32(const modbus_aa_data_t *packet, const uint8_t *firmware, size_t size)
{
  if(packet == NULL || firmware == NULL || size == 0u ||
      size != packet->firmware_size || size != packet->firmware_length)
    return -1;
  return modbus_crc32(firmware, size) == packet->received_crc32;
}

int modbus_find_frame(const modbus_t *ctx, const uint8_t *data, size_t size,
                      size_t *cursor, const uint8_t **frame, size_t *frame_size)
{
  size_t available, length;
  const uint8_t *p;
  uint16_t count;
  if(ctx == NULL || data == NULL || cursor == NULL || frame == NULL ||
      frame_size == NULL || *cursor > size)
    return -1;
  *frame = NULL;
  *frame_size = 0u;
  while(*cursor < size)
  {
    p = data + *cursor;
    available = size - *cursor;
    length = 0u;
    if(available >= 2u && p[0] == MODBUS_ADDRESS && p[1] == MODBUS_FUNCTION_VERSION)
      length = 8u;
    else if(available >= 6u && p[0] == MODBUS_ADDRESS &&
        p[1] == MODBUS_FUNCTION_CONTROL && be16(p + 2) == MODBUS_REGISTER_START)
    {
      count = be16(p + 4);
      length = count == 3u ? 14u : (count == 4u ? 20u : 0u);
    }
    else if(available >= 4u && p[0] == MODBUS_ADDRESS && p[1] == MODBUS_FUNCTION_DATA)
    {
      length = be16(p + 2);
      if(length < 23u)
        length = 0u;
    }
    if(length != 0u && length <= available)
    {
      *frame = p;
      *frame_size = length;
      *cursor += length;
      return 1;
    }
    ++*cursor;
  }
  return 0;
}

int modbus_read_10(const modbus_t *ctx, const uint8_t *frame, size_t size, modbus_10_data_t *out)
{
  modbus_10_data_t value;
  if(ctx == NULL || frame == NULL || out == NULL || (size != 14u && size != 20u) ||
      frame[0] != MODBUS_ADDRESS || frame[1] != MODBUS_FUNCTION_CONTROL ||
      be16(frame + 2) != MODBUS_REGISTER_START ||
      be16(frame + 4) != (size == 14u ? 3u : 4u) ||
      (size == 20u && be32(frame + 6) != 8u))
    return -1;
  memset(&value, 0, sizeof(value));
  value.register_start = be16(frame + 2);
  value.register_count = be16(frame + 4);
  value.is_query = (uint8_t)(size == 14u);
  value.ota_status = be16(frame + (value.is_query ? 6u : 10u));
  value.key = be32(frame + (value.is_query ? 8u : 12u));
  value.operation = value.is_query ? 0u : be16(frame + 16);
  value.received_crc16 = (uint16_t)(frame[size - 2u] | ((uint16_t)frame[size - 1u] << 8));
  *out = value;
  return 1;
}

int modbus_read_aa(const modbus_t *ctx, const uint8_t *frame, size_t size,
                   modbus_aa_data_t *out, uint8_t *firmware, size_t capacity)
{
  modbus_aa_data_t value;
  if(ctx == NULL || frame == NULL || out == NULL || firmware == NULL || size < 23u ||
      size > FRAME_SIZE || frame[0] != MODBUS_ADDRESS || frame[1] != MODBUS_FUNCTION_DATA ||
      be16(frame + 2) != size || be16(frame + 10) != size - 22u || capacity < size - 22u)
    return -1;
  memset(&value, 0, sizeof(value));
  value.data_length = be16(frame + 2);
  value.protocol = be16(frame + 4);
  value.key = be32(frame + 6);
  value.firmware_length = be16(frame + 10);
  value.total_packets = be16(frame + 12);
  value.packet = be16(frame + 14);
  value.received_frame_size = size;
  value.firmware_size = size - 22u;
  value.received_crc32 = be32(frame + size - 6u);
  value.received_crc16 = (uint16_t)(frame[size - 2u] | ((uint16_t)frame[size - 1u] << 8));
  if(firmware != frame + 16)
    memcpy(firmware, frame + 16, value.firmware_size);
  *out = value;
  return 1;
}

static int finish_reply(uint8_t *out, size_t size)
{
  uint16_t crc = modbus_crc16(out, size);
  out[size] = (uint8_t)crc;
  out[size + 1u] = (uint8_t)(crc >> 8);
  return (int)(size + 2u);
}

int modbus_build_10_reply(const modbus_t *ctx, uint8_t is_query, uint8_t *out, size_t capacity)
{
  size_t size = is_query ? 14u : 8u;
  if(ctx == NULL || out == NULL || capacity < size)
    return -1;
  out[0] = MODBUS_ADDRESS;
  out[1] = MODBUS_FUNCTION_CONTROL;
  put16(out + 2, MODBUS_REGISTER_START);
  put16(out + 4, is_query ? 3u : 4u);
  if(is_query)
  {
    put16(out + 6, ctx->status);
    put32(out + 8, ctx->key);
  }
  return finish_reply(out, size - 2u);
}

int modbus_build_aa_reply(const modbus_t *ctx, modbus_ack_t ack, uint16_t packet,
                          uint8_t *out, size_t capacity)
{
  if(ctx == NULL || out == NULL || capacity < 7u ||
      ack < MODBUS_ACK_OK || ack > MODBUS_ACK_FLASH_WRITE)
    return -1;
  out[0] = MODBUS_ADDRESS;
  out[1] = MODBUS_FUNCTION_DATA;
  out[2] = (uint8_t)ack;
  put16(out + 3, packet);
  return finish_reply(out, 5u);
}
