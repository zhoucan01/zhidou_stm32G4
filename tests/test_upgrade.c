#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "upgrade.h"
#include "upgrade_flash.h"
#include "upgrade_platform.h"
#include "modbus_port.h"
#include "sha256.h"
#include "boot_log.h"

void boot_notice(const char *text, uint32_t first, uint32_t second)
{
  (void)text; (void)first; (void)second;
}

uint32_t test_primask;

static uint8_t flash[128u * 1024u], programmed[(128u * 1024u) / 8u];
static uint8_t incoming[MODBUS_MAX_PAYLOAD];
static modbus_10_data_t control;
static modbus_aa_data_t packet;
static uint32_t tick, key;
static uint16_t status, ack_packet;
static int control_ready, packet_ready, query, pending_reply, ack, response_query;
static int paused, pause_count, resume_count, erase_count, fail_erase, fail_program, fatal_count, jumps;
static int receive_idle = 1;
static int interrupt_hash_with_receive;

uint32_t HAL_GetTick(void) { return tick; }
void modbus_port_set_version_reader(int (*reader)(uint16_t, uint8_t *, uint8_t), uint8_t mode)
{
  uint8_t version_byte;
  assert(mode == 0u && reader(0, &version_byte, 1) == 1);
}
void Error_Handler(void) { ++fatal_count; assert(!"Error_Handler"); }
void Upgrade_JumpToApp(uint32_t stack, uint32_t entry)
{
  assert(paused && stack == 0x20008000u && entry == MSP1_ADDR + 17u);
  ++jumps;
}

int Upgrade_FlashRead(uint32_t address, void *data, size_t length)
{
  if (address < UPGRADE_RECORD_ADDRESS || address > UPGRADE_FLASH_END ||
      length > UPGRADE_FLASH_END - address) return 0;
  if (interrupt_hash_with_receive)
  {
    receive_idle = 0;
    interrupt_hash_with_receive = 0;
  }
  memcpy(data, flash + address - UPGRADE_FLASH_BASE, length);
  return 1;
}

int Upgrade_FlashErase(uint32_t address, uint32_t length)
{
  assert(paused && address >= UPGRADE_RECORD_ADDRESS && address + length <= UPGRADE_FLASH_END);
  assert(!(address & 2047u) && !(length & 2047u));
  ++erase_count;
  if (erase_count == fail_erase) return 0;
  if (address == UPGRADE_APP_DATA_ADDRESS)
  {
    size_t i;
    for (i = 0; i < 8u; ++i)
      assert(flash[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE + i] == 0xff);
  }
  memset(flash + address - UPGRADE_FLASH_BASE, 0xff, length);
  memset(programmed + (address - UPGRADE_FLASH_BASE) / 8u, 0, length / 8u);
  return 1;
}

int Upgrade_FlashProgram(uint32_t address, const uint8_t *data, size_t length)
{
  size_t i, offset = address - UPGRADE_FLASH_BASE;
  assert(address >= UPGRADE_RECORD_ADDRESS && address + length <= UPGRADE_FLASH_END);
  assert(!(address & 7u) && length && !(length & 7u));
  if (fail_program == 1 || (fail_program == 2 && address == UPGRADE_RECORD_ADDRESS))
    return 0;
  for (i = 0; i < length; i += 8u)
  {
    size_t j;
    int all_ff = 1;
    for (j = 0; j < 8u; ++j) if (data[i + j] != 0xffu) all_ff = 0;
    if (!all_ff)
    {
      assert(!programmed[(offset + i) / 8u]);
      programmed[(offset + i) / 8u] = 1u;
    }
    for (j = 0; j < 8u; ++j)
    {
      assert((flash[offset + i + j] & data[i + j]) == data[i + j]);
      flash[offset + i + j] &= data[i + j];
    }
  }
  return 1;
}

int modbus_port_take_query(void) { int value = query; query = 0; return value; }
int modbus_port_rx_idle(void) { return receive_idle; }
int modbus_reply_pending(void) { return pending_reply; }
void modbus_port_set_ota_status(uint16_t value) { status = value; }
uint16_t modbus_port_get_ota_status(void) { return status; }
void modbus_port_set_session_key(uint32_t value) { key = value; }
HAL_StatusTypeDef modbus_port_pause_rx(void)
{
  assert(!paused); paused = 1; ++pause_count; control_ready = packet_ready = 0; return HAL_OK;
}
HAL_StatusTypeDef modbus_port_resume_rx(void)
{
  assert(paused); paused = 0; ++resume_count; return HAL_OK;
}
HAL_StatusTypeDef modbus_port_pause_rx_if_idle(void)
{
  if (!receive_idle || pending_reply || control_ready || packet_ready || query) return HAL_BUSY;
  return modbus_port_pause_rx();
}
int modbus_set_ack(uint8_t code, uint16_t number)
{
  assert(!pending_reply); pending_reply = 1; ack = code; ack_packet = number; return 1;
}
int modbus_set_10_reply(uint8_t is_query)
{
  assert(!pending_reply); pending_reply = 1; response_query = is_query; return 1;
}
int modbus_port_get_10(modbus_10_data_t *out)
{
  if (!control_ready) return 0;
  control_ready = 0; *out = control; return 1;
}
int modbus_port_get_aa(modbus_aa_data_t *out, uint8_t *data, size_t capacity)
{
  if (!packet_ready) return 0;
  assert(capacity >= packet.firmware_size);
  packet_ready = 0; *out = packet; memcpy(data, incoming, packet.firmware_size); return 1;
}
static uint32_t crc32(const uint8_t *data, size_t size)
{
  uint32_t crc = 0xffffffffu;
  size_t i; unsigned bit;
  for (i = 0; i < size; ++i)
  {
    crc ^= data[i];
    for (bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
  }
  return crc ^ 0xffffffffu;
}
int modbus_check_aa_crc32(const modbus_aa_data_t *value, const uint8_t *data, size_t length)
{
  if (!value || !data || !length || length != value->firmware_size || length != value->firmware_length)
    return -1;
  return crc32(data, length) == value->received_crc32;
}

static void reset_test(void)
{
  memset(flash, 0xff, sizeof(flash)); memset(programmed, 0, sizeof(programmed));
  memset(&control, 0, sizeof(control)); memset(&packet, 0, sizeof(packet));
  tick = key = 0; status = 0; control_ready = packet_ready = query = pending_reply = ack = 0;
  paused = pause_count = resume_count = erase_count = fail_erase = fail_program = fatal_count = jumps = 0;
  receive_idle = 1;
  interrupt_hash_with_receive = 0;
  Upgrade_Init();
  assert(status == MODBUS_STATUS_DAMAGED);
}
static void clear_reply(void) { assert(pending_reply); pending_reply = 0; ack = 0; }
static void start(void)
{
  control.operation = 1; control.key = 0x12345678u; control_ready = 1;
  Upgrade_Poll();
  assert(pending_reply && response_query == 0 && status == MODBUS_STATUS_DOWNLOADING);
  assert(key == 0x12345678u && !paused);
  clear_reply();
}
static void send_packet(uint16_t number, uint16_t last, const uint8_t *data, size_t size)
{
  assert(size && size <= sizeof(incoming));
  memcpy(incoming, data, size);
  memset(&packet, 0, sizeof(packet));
  packet.protocol = 1; packet.key = key; packet.packet = number; packet.total_packets = last;
  packet.firmware_length = (uint16_t)size; packet.firmware_size = size;
  packet.received_crc32 = crc32(data, size);
  packet_ready = 1; Upgrade_Poll();
  assert(pending_reply && ack_packet == number);
}
static void check_packet_error(uint8_t expected)
{
  packet_ready = 1; Upgrade_Poll();
  assert(pending_reply && ack == expected && ack_packet == packet.packet);
  clear_reply();
}
static void put32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value; data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16); data[3] = (uint8_t)(value >> 24);
}
static void make_image(uint8_t *image, size_t length)
{
  sha256_ctx ctx;
  size_t i;
  memset(image, 0, length);
  memcpy(image, "1.0", 3);
  for (i = UPGRADE_HEADER_SIZE; i < length; ++i) image[i] = (uint8_t)(i * 13u);
  put32(image + UPGRADE_HEADER_SIZE, 0x20008000u);
  put32(image + UPGRADE_HEADER_SIZE + 4, MSP1_ADDR + 17u);
  sha256_init(&ctx); sha256_update(&ctx, image + UPGRADE_HEADER_SIZE, length - UPGRADE_HEADER_SIZE);
  sha256_final(&ctx, image + 1024);
}

int main(void)
{
  uint8_t image[UPGRADE_HEADER_SIZE + 71u], data[1024] = {0};
  size_t offset, size;
  uint16_t number, last;

  make_image(image, sizeof(image));
  reset_test(); start();
  assert(erase_count == 2 && pause_count == 1 && resume_count == 1);
  send_packet(0, 400, image, 7); assert(ack == MODBUS_ACK_OK); clear_reply();
  assert(flash[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE] == 0xff);
  Upgrade_Init(); assert(status == MODBUS_STATUS_DAMAGED); /* power loss before commit */

  reset_test(); start();
  last = (uint16_t)((sizeof(image) + 6u) / 7u - 1u);
  for (offset = 0, number = 0; offset < sizeof(image); offset += size, ++number)
  {
    size = sizeof(image) - offset; if (size > 7u) size = 7u;
    send_packet(number, last, image + offset, size);
    assert(ack == MODBUS_ACK_OK);
    clear_reply();
  }
  assert(status == MODBUS_STATUS_IDLE);
  assert(!memcmp(flash + UPGRADE_APP_DATA_ADDRESS - UPGRADE_FLASH_BASE, image, sizeof(image)));
  assert(flash[UPGRADE_APP_DATA_ADDRESS - UPGRADE_FLASH_BASE + sizeof(image)] == 0xff);
  assert(flash[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE] == 0x32);
  assert(flash[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE + 4] == 71);
  /* 待发应答未完成时，不得处理下一条命令或跳转。 */
  pending_reply = 1; control.operation = 3; control_ready = 1;
  Upgrade_Poll(); assert(control_ready && jumps == 0);
  pending_reply = 0; Upgrade_Poll();
  assert(!control_ready && pending_reply && response_query == 1 && jumps == 0);
  clear_reply();
  receive_idle = 0; Upgrade_Poll(); assert(jumps == 0);
  receive_idle = 1; interrupt_hash_with_receive = 1;
  Upgrade_Poll(); assert(jumps == 0);
  receive_idle = 1; Upgrade_Poll(); assert(jumps == 1);
  paused = 0; tick = 0; Upgrade_Init();
  tick = UPGRADE_BOOT_WAIT_MS - 1u; Upgrade_Poll(); assert(jumps == 1);
  tick = UPGRADE_BOOT_WAIT_MS; Upgrade_Poll(); assert(jumps == 2);
  paused = 0; tick = 0; Upgrade_Init();
  assert(status == MODBUS_STATUS_IDLE);
  tick = UPGRADE_BOOT_WAIT_MS - 1u; Upgrade_Poll(); assert(jumps == 2);
  query = 1; Upgrade_Poll(); tick = 5000; Upgrade_Poll(); assert(jumps == 2);
  flash[MSP1_ADDR - UPGRADE_FLASH_BASE + 30] ^= 1u;
  Upgrade_Init(); assert(status == MODBUS_STATUS_DAMAGED);
  tick += 2001; Upgrade_Poll(); assert(jumps == 2);

  /* 未开始升级时收到数据包仍需回复错误，握手状态也能重新开始。 */
  reset_test();
  send_packet(0, 0, image, 8); assert(ack == MODBUS_ACK_KEY); clear_reply();
  query = 1; Upgrade_Poll();
  send_packet(0, 0, image, 8); assert(ack == MODBUS_ACK_KEY); clear_reply();
  start();
  /* 接收中再次开始升级时，仍按原行为重新擦除并重置会话。 */
  start(); assert(erase_count == 4);
  send_packet(0, 400, image, 8); assert(ack == MODBUS_ACK_OK); clear_reply();

  reset_test(); fail_erase = 2;
  control.operation = 1; control.key = 9; control_ready = 1; Upgrade_Poll();
  assert(pending_reply && response_query == 1 && status == MODBUS_STATUS_DAMAGED && !paused);
  assert(erase_count == 2 && pause_count == 1 && resume_count == 1);

  reset_test(); start();
  memcpy(incoming, image, 8);
  memset(&packet, 0, sizeof(packet));
  packet.packet = 0; packet.total_packets = 2; packet.protocol = 2; packet.key = key;
  packet.firmware_size = packet.firmware_length = 8; packet.received_crc32 = crc32(incoming, 8);
  check_packet_error(MODBUS_ACK_VERSION);
  packet.protocol = 1; packet.key ^= 1u; check_packet_error(MODBUS_ACK_KEY);
  packet.key = key; packet.packet = 1; check_packet_error(MODBUS_ACK_SEQUENCE);
  packet.packet = 0; packet.received_crc32 ^= 1u; check_packet_error(MODBUS_ACK_FIRMWARE_CRC);
  assert(flash[UPGRADE_APP_DATA_ADDRESS - UPGRADE_FLASH_BASE] == 0xff);
  data[0] = 'a'; data[1] = 0; data[2] = 'b';
  send_packet(0, 3, data, 8); assert(ack == MODBUS_ACK_TOO_LONG); clear_reply();
  memset(data, 0, sizeof(data));

  reset_test(); start(); fail_program = 1;
  send_packet(0, 3, data, 8); assert(ack == MODBUS_ACK_FLASH_WRITE && status == MODBUS_STATUS_DAMAGED);
  assert(flash[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE] == 0xff);

  reset_test(); start(); image[1024] ^= 1u;
  for (offset = 0, number = 0; offset < sizeof(image); offset += size, ++number)
  {
    size = sizeof(image) - offset; if (size > 1024u) size = 1024u;
    send_packet(number, 2, image + offset, size);
    if (number < 2) { assert(ack == MODBUS_ACK_OK); clear_reply(); }
  }
  assert(ack == MODBUS_ACK_FIRMWARE_CRC && status == MODBUS_STATUS_DAMAGED);
  assert(flash[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE] == 0xff);
  image[1024] ^= 1u;

  reset_test(); start(); fail_program = 2;
  for (offset = 0, number = 0; offset < sizeof(image); offset += size, ++number)
  {
    size = sizeof(image) - offset; if (size > 1024u) size = 1024u;
    send_packet(number, 2, image + offset, size);
    if (number < 2) { assert(ack == MODBUS_ACK_OK); clear_reply(); }
  }
  assert(ack == MODBUS_ACK_FLASH_WRITE && status == MODBUS_STATUS_DAMAGED);
  assert(flash[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE] == 0xff);

  reset_test(); start();
  for (number = 0; number < UPGRADE_IMAGE_CAPACITY / sizeof(data); ++number)
  {
    send_packet(number, 500, data, sizeof(data)); assert(ack == MODBUS_ACK_OK); clear_reply();
  }
  send_packet(number, 500, data, 1);
  assert(ack == MODBUS_ACK_TOO_LONG && flash[UPGRADE_RECORD_ADDRESS - UPGRADE_FLASH_BASE] == 0xff);
  puts("single-partition upgrade safety tests passed");
  return 0;
}
