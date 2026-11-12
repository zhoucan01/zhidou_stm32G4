#ifndef OTA_APP_BRIDGE_H
#define OTA_APP_BRIDGE_H

#include <stdint.h>
#include <stddef.h>

/* APP-only adapter. OTA management and business Modbus both use station 3.
 * The caller verifies CRC16, appends response CRC16 and transmits in task context.
 * Reset only after reset_requested is set AND the complete response was sent.
 */
#define OTA_APP_ADDRESS 3u
#define OTA_APP_VERSION_ADDRESS 0x08004000u

static uint16_t OtaApp_Read16(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint16_t OtaApp_RequestLength(const uint8_t *r, uint16_t available)
{
  uint16_t count;
  if(r == NULL || available < 2u || r[0] != OTA_APP_ADDRESS) return 0u;
  if(r[1] == 0x11u) return 8u;
  if(available < 6u || r[1] != 0x10u || OtaApp_Read16(r + 2) != 0x00e6u) return 0u;
  count = OtaApp_Read16(r + 4);
  return count == 3u ? 14u : (count == 4u ? 20u : 0u);
}

static uint16_t OtaApp_Process(const uint8_t *r, uint16_t length, uint8_t *out,
                              size_t capacity, int (*read_version)(uint16_t, uint8_t *, uint8_t),
                              uint8_t *reset_requested)
{
  uint16_t expected = OtaApp_RequestLength(r, length), offset, count, operation;
  if(out == NULL || reset_requested == NULL || capacity < 2u) return 0u;
  *reset_requested = 0u;
  if(expected == 0u || expected != length) return 0u;
  out[0] = OTA_APP_ADDRESS; out[1] = r[1];
  if(r[1] == 0x11u)
  {
    offset = OtaApp_Read16(r + 2); count = OtaApp_Read16(r + 4);
    if(count == 0u || count > 32u || offset >= 1024u || count > 1024u - offset ||
       capacity < count + 8u || read_version == NULL) return 0u;
    if(read_version(offset, out + 6, (uint8_t)count) != 1) return 0u;
    out[2] = 1u; out[3] = r[2]; out[4] = r[3]; out[5] = (uint8_t)count;
    return (uint16_t)(6u + count);
  }
  if(capacity < 14u) return 0u;
  if(length == 20u)
  {
    if(r[6] != 0u || r[7] != 0u || r[8] != 0u || r[9] != 8u) return 0u;
    operation = OtaApp_Read16(r + 16);
    if(operation == 0u || operation == 2u)
    {
      out[2] = 0u; out[3] = 0xe6u; out[4] = 0u; out[5] = 4u;
      *reset_requested = (uint8_t)(operation == 2u);
      return 6u;
    }
    /* APP rejects start/rollback by reporting forbidden status and zero key. */
  }
  out[2] = 0u; out[3] = 0xe6u; out[4] = 0u; out[5] = 3u;
  for(count = 6u; count < 12u; ++count) out[count] = 0u;
  return 12u;
}

#endif
