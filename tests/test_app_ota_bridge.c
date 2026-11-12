#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ota_app_bridge.h"

static int read_version(uint16_t offset, uint8_t *out, uint8_t count)
{
  unsigned int i;
  for(i = 0; i < count; ++i) out[i] = (uint8_t)(offset + i);
  return 1;
}

int main(void)
{
  uint8_t version[8] = {OTA_APP_ADDRESS, 0x11, 3, 0xe0, 0, 32};
  uint8_t query[14] = {OTA_APP_ADDRESS, 0x10, 0, 0xe6, 0, 3};
  uint8_t control[20] = {OTA_APP_ADDRESS, 0x10, 0, 0xe6, 0, 4, 0, 0, 0, 8, 0, 1};
  uint8_t response[40], reset;
  unsigned int op;
  uint8_t business[11] = {3, 0x10, 0, 2, 0, 1, 2, 0, 1, 0, 0};
  assert(OtaApp_RequestLength(business, sizeof(business)) == 0u);
  assert(OtaApp_RequestLength(query, 5) == 0u);
  assert(OtaApp_RequestLength(query, 6) == 14u);
  assert(OtaApp_Process(query, 14, response, 40, read_version, &reset) == 12u);
  assert(!reset && response[0] == OTA_APP_ADDRESS && response[5] == 3u);
  assert(!memcmp(response + 6, "\0\0\0\0\0\0", 6));
  for(op = 0; op <= 3u; ++op)
  {
    control[17] = (uint8_t)op;
    assert(OtaApp_Process(control, 20, response, 40, read_version, &reset) ==
           ((op == 0u || op == 2u) ? 6u : 12u));
    assert(reset == (op == 2u));
  }
  assert(OtaApp_Process(version, 8, response, 40, read_version, &reset) == 38u);
  assert(!reset && response[2] == 1u && response[5] == 32u && response[6] == 0xe0u);
  version[3] = 0xe1u;
  assert(OtaApp_Process(version, 8, response, 40, read_version, &reset) == 0u);
  control[17] = 2u; control[9] = 7u;
  assert(OtaApp_Process(control, 20, response, 40, read_version, &reset) == 0u && !reset);
  control[0] = 1u;
  assert(OtaApp_RequestLength(control, 20) == 0u);
  puts("APP OTA management protocol tests passed");
  return 0;
}
