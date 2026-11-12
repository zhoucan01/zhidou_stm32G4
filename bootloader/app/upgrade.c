#include "upgrade.h"
#include "upgrade_flash.h"
#include "upgrade_platform.h"
#include "modbus_port.h"
#include "sha256.h"
#include "main.h"
#include "error_trace.h"
#include "boot_log.h"
#include <string.h>

/* OTA 镜像依次包含 1 KiB 版本区、1 KiB 摘要区和 APP；摘要本身为 32 字节。 */
#define VERSION_SIZE 1024u
#define HASH_SIZE 32u
#define DAMAGED_REPORT_MS 100u
#define PROTOCOL_VERSION 1u

typedef enum
{
  UPGRADE_WAITING,    /* 上电等待查询，超时后可启动有效 APP。 */
  UPGRADE_HANDSHAKE,  /* 已进入握手，等待升级控制命令。 */
  UPGRADE_RECEIVING,  /* 擦除完成，按顺序接收升级数据。 */
  UPGRADE_START_APP   /* 升级完成，等待通信空闲后启动 APP。 */
} upgrade_state_t;

 upgrade_state_t state;
/* received_size 是接收的镜像字节数，written_size 是已写入 Flash 的字节数。 */
static uint32_t boot_tick, report_tick, session_key, received_size, written_size;
static uint16_t next_packet;
/* version_padding 跨包记录版本区是否已遇到零填充；pending 保存未凑满双字的数据。 */
static uint8_t version_padding, pending_size;
static uint8_t pending[8], firmware[MODBUS_MAX_PAYLOAD], write_buffer[MODBUS_MAX_PAYLOAD + 8u];

static uint32_t read_le32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_le32(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

/* 校验 APP 向量和原始 APP 的 SHA-256，不包含版本区和摘要区。 */
static int image_valid(uint32_t length)
{
  uint8_t vectors[8], expected[HASH_SIZE], actual[HASH_SIZE], block[128];
  uint32_t stack, entry, offset = 0u;
  sha256_ctx hash;
  if(length < 8u || length > UPGRADE_APP_CAPACITY ||
      !Upgrade_FlashRead(MSP1_ADDR, vectors, sizeof(vectors)) ||
      !Upgrade_FlashRead(UPGRADE_HASH_ADDRESS, expected, sizeof(expected)))
    return 0;
  stack = read_le32(vectors);
  entry = read_le32(vectors + 4);
  /* 栈指针须位于 SRAM 且 8 字节对齐；复位入口须为 APP 内的 Thumb 地址。 */
  if(stack <= 0x20000000u || stack > 0x20008000u || (stack & 7u) != 0u ||
      (entry & 1u) == 0u || (entry & ~1u) < MSP1_ADDR + 8u ||
      (entry & ~1u) >= MSP1_ADDR + length)
    return 0;
  sha256_init(&hash);
  while(offset < length)
  {
    uint32_t count = length - offset;
    if(count > sizeof(block))
      count = sizeof(block);
    if(!Upgrade_FlashRead(MSP1_ADDR + offset, block, count))
      return 0;
    sha256_update(&hash, block, count);
    offset += count;
  }
  sha256_final(&hash, actual);
  {
    int valid = memcmp(actual, expected, sizeof(actual)) == 0;
    boot_notice(valid ? "SHA256 PASS" : "SHA256 FAIL", length, BOOT_LOG_NONE);
    return valid;
  }
}

/* 有效记录以小端保存魔数和原始 APP 长度；记录有效后仍需重新校验镜像。 */
static int installed_image_valid(void)
{
  uint8_t record[8];
  return Upgrade_FlashRead(UPGRADE_RECORD_ADDRESS, record, sizeof(record)) &&
         read_le32(record) == UPGRADE_IMAGE_VALID && image_valid(read_le32(record + 4));
}

static void queue_ack(modbus_ack_t code, uint16_t packet)
{
  int result = modbus_set_ack((uint8_t)code, packet);
  boot_notice("ACK code/packet", (uint32_t)code, packet);
  if(result != 1)
    /* 0x3001: OTA 数据应答入队失败；detail=0 队列满，-1 参数非法。 */
    ERROR_HANDLER_FROM(ERROR_ID_UPGRADE_ACK_QUEUE, "OTA data ACK queue failed", result);
}

static void queue_control_reply(uint8_t is_query)
{
  int result = modbus_set_10_reply(is_query);
  if(result != 1)
    /* 0x3002: OTA 控制/状态应答入队失败。 */
    ERROR_HANDLER_FROM(ERROR_ID_UPGRADE_CONTROL_QUEUE, "OTA control reply queue failed", result);
}

/**
 * @brief 暂停接收并擦除单 APP 分区，成功后建立新的升级会话。
 * @return 1 擦除成功且接收恢复；0 擦除失败，旧 APP 不再视为有效。
 */
static int begin_upgrade(uint32_t key)
{
  int erased;
  HAL_StatusTypeDef status;
  boot_notice("Erase start", BOOT_LOG_NONE, BOOT_LOG_NONE);
  modbus_port_set_ota_status(MODBUS_STATUS_FLASHING);
  /* 擦除期间暂停接收，先使旧记录失效，再擦除唯一的 APP 分区。 */
  status = modbus_port_pause_rx();
  if(status != HAL_OK)
    /* 0x3003: 擦除 APP 前暂停串口接收失败。 */
    ERROR_HANDLER_FROM(ERROR_ID_UPGRADE_PAUSE_RX, "Pause RX before OTA erase", status);
  erased = Upgrade_FlashErase(UPGRADE_RECORD_ADDRESS, UPGRADE_APP_DATA_ADDRESS - UPGRADE_RECORD_ADDRESS);
  if(erased)
    erased = Upgrade_FlashErase(UPGRADE_APP_DATA_ADDRESS, UPGRADE_IMAGE_CAPACITY);
  status = modbus_port_resume_rx();
  if(status != HAL_OK)
    /* 0x3004: 擦除后恢复串口接收失败。 */
    ERROR_HANDLER_FROM(ERROR_ID_UPGRADE_RESUME_RX, "Resume RX after OTA erase", status);
  if(!erased)
  {
    boot_notice("Erase FAILED", BOOT_LOG_NONE, BOOT_LOG_NONE);
    state = UPGRADE_HANDSHAKE;
    modbus_port_set_ota_status(installed_image_valid() ? MODBUS_STATUS_IDLE : MODBUS_STATUS_DAMAGED);
    return 0;
  }
  session_key = key;
  received_size = written_size = 0u;
  next_packet = 0u;
  pending_size = version_padding = 0u;
  modbus_port_set_session_key(key);
  modbus_port_set_ota_status(MODBUS_STATUS_DOWNLOADING);
  state = UPGRADE_RECEIVING;
  boot_notice("Erase OK; receiving", BOOT_LOG_NONE, BOOT_LOG_NONE);
  return 1;
}

/* 版本区遇到第一个零后，其余字节必须为零，填充状态需要跨包延续。 */
static int version_padding_valid(const uint8_t *data, size_t size, uint8_t *new_padding)
{
  size_t i;
  uint8_t padding = version_padding;
  for(i = 0u; i < size && received_size + i < VERSION_SIZE; ++i)
  {
    if(data[i] == 0u)
      padding = 1u;
    else if(padding)
      return 0;
  }
  *new_padding = padding;
  return 1;
}

/* Flash 按 8 字节写入；非末包尾部不足 8 字节时暂存在 RAM，末包用 0xFF 补齐。 */
static int write_packet(const uint8_t *data, size_t length, int last)
{
  size_t available = pending_size + length;
  size_t write_size = available & ~(size_t)7u;
  size_t remainder = available - write_size;
  memcpy(write_buffer, pending, pending_size);
  memcpy(write_buffer + pending_size, data, length);
  if(last && remainder != 0u)
  {
    memset(write_buffer + available, 0xff, 8u - remainder);
    write_size += 8u;
    remainder = 0u;
  }
  if(write_size != 0u &&
      !Upgrade_FlashProgram(UPGRADE_APP_DATA_ADDRESS + written_size, write_buffer, write_size))
    return 0;
  written_size += (uint32_t)write_size;
  pending_size = (uint8_t)remainder;
  if(remainder != 0u)
    memcpy(pending, write_buffer + write_size, remainder);
  return 1;
}

static void process_packet(const modbus_aa_data_t *packet)
{
  uint8_t padding, record[8];
  uint32_t total;
  int last;
  boot_notice("RX packet/last", packet->packet, packet->total_packets);
  /* 写入前依次检查升级会话、协议版本、包序和当前包 CRC32。 */
  if(state != UPGRADE_RECEIVING || packet->key != session_key)
  {
    queue_ack(MODBUS_ACK_KEY, packet->packet);
    return;
  }
  if(packet->protocol != PROTOCOL_VERSION)
  {
    queue_ack(MODBUS_ACK_VERSION, packet->packet);
    return;
  }
  if(packet->packet != next_packet || packet->packet > packet->total_packets)
  {
    queue_ack(MODBUS_ACK_SEQUENCE, packet->packet);
    return;
  }
  if(modbus_check_aa_crc32(packet, firmware, packet->firmware_size) != 1)
  {
    queue_ack(MODBUS_ACK_FIRMWARE_CRC, packet->packet);
    return;
  }
  /* total_packets 表示末包包号，包号从 0 开始。 */
  last = packet->packet == packet->total_packets;
  if(packet->firmware_size > UPGRADE_IMAGE_CAPACITY - received_size)
  {
    queue_ack(MODBUS_ACK_TOO_LONG, packet->packet);
    return;
  }
  total = received_size + (uint32_t)packet->firmware_size;
  if((last && total < UPGRADE_HEADER_SIZE + 8u) ||
      !version_padding_valid(firmware, packet->firmware_size, &padding))
  {
    queue_ack(MODBUS_ACK_TOO_LONG, packet->packet);
    return;
  }
  if(!write_packet(firmware, packet->firmware_size, last))
  {
    boot_notice("Flash write FAILED", packet->packet, BOOT_LOG_NONE);
    state = UPGRADE_HANDSHAKE;
    modbus_port_set_ota_status(MODBUS_STATUS_DAMAGED);
    queue_ack(MODBUS_ACK_FLASH_WRITE, packet->packet);
    return;
  }
  version_padding = padding;
  received_size = total;
  boot_notice("Written image bytes", received_size, BOOT_LOG_NONE);
  if(last)
  {
    uint32_t app_length = total - UPGRADE_HEADER_SIZE;
    /* 全部写入后校验 APP；只有校验通过才提交有效记录。 */
    if(!image_valid(app_length))
    {
      state = UPGRADE_HANDSHAKE;
      modbus_port_set_ota_status(MODBUS_STATUS_DAMAGED);
      queue_ack(MODBUS_ACK_FIRMWARE_CRC, packet->packet);
      return;
    }
    put_le32(record, UPGRADE_IMAGE_VALID);
    put_le32(record + 4, app_length);
    if(!Upgrade_FlashProgram(UPGRADE_RECORD_ADDRESS, record, sizeof(record)))
    {
      boot_notice("Record write FAILED", BOOT_LOG_NONE, BOOT_LOG_NONE);
      state = UPGRADE_HANDSHAKE;
      modbus_port_set_ota_status(MODBUS_STATUS_DAMAGED);
      queue_ack(MODBUS_ACK_FLASH_WRITE, packet->packet);
      return;
    }
    modbus_port_set_ota_status(MODBUS_STATUS_IDLE);
    state = UPGRADE_START_APP;
    boot_notice("Record saved; OTA complete", app_length, BOOT_LOG_NONE);
  }
  else
    ++next_packet;
  /* 非末包 ACK 只表示已接收，未对齐的尾部可能仍在 RAM 中。 */
  queue_ack(MODBUS_ACK_OK, packet->packet);
}



static void start_app(void)
{
  uint8_t vectors[8];
  HAL_StatusTypeDef result;
  if(!installed_image_valid() || !Upgrade_FlashRead(MSP1_ADDR, vectors, sizeof(vectors)))
  {
    boot_notice("APP check FAILED", BOOT_LOG_NONE, BOOT_LOG_NONE);
    state = UPGRADE_HANDSHAKE;
    modbus_port_set_ota_status(MODBUS_STATUS_DAMAGED);
    return;
  }
  /* 跳转前再次校验，并等待串口空闲后暂停接收。 */
  result = modbus_port_pause_rx_if_idle();
  if(result == HAL_BUSY)
    return;
  if(result != HAL_OK)
    /* 0x3005: 跳转 APP 前停止接收失败；HAL_BUSY 已在上面单独处理。 */
    ERROR_HANDLER_FROM(ERROR_ID_APP_JUMP_PAUSE_RX, "Pause RX before APP jump", result);
  boot_notice("Jump APP 08004800", BOOT_LOG_NONE, BOOT_LOG_NONE);
  Upgrade_JumpToApp(read_le32(vectors), read_le32(vectors + 4));
}

/** @brief 检查已安装镜像并进入上电等待状态；在串口接收启动后调用。 */
static int read_version(uint16_t offset, uint8_t *data, uint8_t count)
{
  return Upgrade_FlashRead(UPGRADE_APP_DATA_ADDRESS + offset, data, count);
}

void Upgrade_Init(void)
{
  int valid;
  /* 上电先检查现有 APP；无有效镜像时留在 Bootloader。 */
  boot_tick = report_tick = HAL_GetTick();
  modbus_port_set_version_reader(read_version, 0u);
  state = UPGRADE_WAITING;
  valid = installed_image_valid();
  modbus_port_set_ota_status(valid ? MODBUS_STATUS_IDLE : MODBUS_STATUS_DAMAGED);
  boot_notice(valid ? "APP valid; wait ms" : "APP invalid; stay", UPGRADE_BOOT_WAIT_MS, BOOT_LOG_NONE);
}

/* 所有状态共用同一个接收队列，按队首顺序处理控制帧或数据帧。 */
static int process_input(void)
{
  modbus_10_data_t control;
  modbus_aa_data_t packet;

  if(modbus_port_get_10(&control) == 1)
  {
    boot_notice("Control op", control.operation, BOOT_LOG_NONE);
    if(state == UPGRADE_WAITING)
      state = UPGRADE_HANDSHAKE;
    if(control.operation == 1u)
      queue_control_reply(begin_upgrade(control.key) ? 0u : 1u);
    else if(control.operation == 3u)
      /* 单分区不支持回滚，返回当前状态。 */
      queue_control_reply(1u);
    return 1;
  }
  if(modbus_port_get_aa(&packet, firmware, sizeof(firmware)) == 1)
  {
    process_packet(&packet);
    return 1;
  }
  return 0;
}

/**
 * @brief 主循环调度升级状态机；依次处理应答等待、输入和 APP 启动。
 * @note 不在中断中调用；Flash 操作和镜像校验均在此上下文完成。
 */
void Upgrade_Poll(void)
{
  /* 查询会使等待状态进入握手，避免等待超时后自动跳转 APP。 */
  if(modbus_port_take_query())
  {
    boot_notice("Query state/status", (uint32_t)state, modbus_port_get_ota_status());
    if(state == UPGRADE_WAITING) state = UPGRADE_HANDSHAKE;
  }
  /* 先发送已排队的应答，再处理后续命令或跳转。 */
  if(modbus_reply_pending())
    return;

  switch(state)
  {
    case UPGRADE_WAITING:
      if(process_input())
        return;
      if((uint32_t)(HAL_GetTick() - boot_tick) >= UPGRADE_BOOT_WAIT_MS &&
          modbus_port_get_ota_status() == MODBUS_STATUS_IDLE && modbus_port_rx_idle())
      {
        start_app();
        return;
      }
      break;

    case UPGRADE_HANDSHAKE:
      if(process_input())
        return;
      break;

    case UPGRADE_RECEIVING:
      if(process_input())
        return;
      break;

    case UPGRADE_START_APP:
      if(process_input())
        return;
      if(modbus_port_rx_idle())
      {
        start_app();
        return;
      }
      break;

    default:
      /* 0x3006: 升级状态机取值非法，detail 保存当时的 state。 */
      ERROR_HANDLER_FROM(ERROR_ID_UPGRADE_STATE, "Invalid OTA state", state);
      return;
  }

  /* APP 损坏时周期性上报状态，等待重新升级。 */
  if(modbus_port_get_ota_status() == MODBUS_STATUS_DAMAGED &&
      (uint32_t)(HAL_GetTick() - report_tick) >= DAMAGED_REPORT_MS)
  {
    queue_control_reply(1u);
    report_tick = HAL_GetTick();
  }
}
