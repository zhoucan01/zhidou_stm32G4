#ifndef MODBUS_H
#define MODBUS_H

#include <stddef.h>
#include <stdint.h>

#define MODBUS_ADDRESS             0x03u
#define MODBUS_FUNCTION_CONTROL    0x10u
#define MODBUS_FUNCTION_VERSION    0x11u
#define MODBUS_VERSION_CHUNK       32u
#define MODBUS_VERSION_SIZE        1024u
#define MODBUS_FUNCTION_DATA       0xAAu
#define MODBUS_REGISTER_START      0x00E6u
#define MODBUS_STATUS_FORBIDDEN    0x0000u
#define MODBUS_STATUS_IDLE         0x0001u
#define MODBUS_STATUS_DOWNLOADING  0x0002u
#define MODBUS_STATUS_FLASHING     0x0003u
#define MODBUS_STATUS_DAMAGED      0x0055u
#define MODBUS_MAX_PAYLOAD         1024u
#define MODBUS_RESPONSE_CAPACITY   40u

typedef enum
{
  MODBUS_ACK_OK = 0x01,
  MODBUS_ACK_FIRMWARE_CRC = 0x02,
  MODBUS_ACK_RX_TIMEOUT = 0x03,
  MODBUS_ACK_TOO_SHORT = MODBUS_ACK_RX_TIMEOUT,
  MODBUS_ACK_VERSION = 0x04,
  MODBUS_ACK_KEY = 0x05,
  MODBUS_ACK_SEQUENCE = 0x06,
  MODBUS_ACK_FRAME_CRC = 0x07,
  MODBUS_ACK_TOO_LONG = 0x08,
  MODBUS_ACK_FLASH_WRITE = 0x09
} modbus_ack_t;

typedef struct
{
  uint32_t key;
  uint16_t status;
} modbus_t;

typedef struct
{
  uint16_t register_start, register_count, ota_status;
  uint32_t key;
  uint16_t operation;
  uint8_t is_query;
  uint16_t received_crc16;
} modbus_10_data_t;

typedef struct
{
  uint16_t data_length, protocol;
  uint32_t key;
  uint16_t firmware_length, total_packets, packet;
  size_t received_frame_size, firmware_size;
  uint32_t received_crc32;
  uint16_t received_crc16;
} modbus_aa_data_t;

uint16_t modbus_crc16(const uint8_t *data, size_t size);
uint32_t modbus_crc32(const uint8_t *data, size_t size);
int modbus_check_aa_crc32(const modbus_aa_data_t *packet, const uint8_t *firmware, size_t size);
/* Frame scanning and read functions check format only; the receive path checks CRC16. */
int modbus_find_frame(const modbus_t *ctx, const uint8_t *data, size_t size,
                      size_t *cursor, const uint8_t **frame, size_t *frame_size);
int modbus_read_10(const modbus_t *ctx, const uint8_t *frame, size_t size, modbus_10_data_t *out);
int modbus_read_aa(const modbus_t *ctx, const uint8_t *frame, size_t size,
                   modbus_aa_data_t *out, uint8_t *firmware, size_t capacity);
int modbus_build_10_reply(const modbus_t *ctx, uint8_t is_query, uint8_t *out, size_t capacity);
int modbus_build_aa_reply(const modbus_t *ctx, modbus_ack_t ack, uint16_t packet,
                          uint8_t *out, size_t capacity);

#endif
