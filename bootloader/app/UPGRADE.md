# STM32G431 single-partition upgrade

## Flash layout

| Address range | Purpose |
| --- | --- |
| `0x08000000`–`0x080037FF` | Bootloader, at most 14 KiB |
| `0x08003800`–`0x08003FFF` | Bootloader data and image-validity record |
| `0x08004000`–`0x080043FF` | Version bytes, padded with zero to 1 KiB |
| `0x08004400`–`0x080047FF` | SHA-256 area; the first 32 bytes contain the digest |
| `0x08004800`–`0x0801FFFF` | Application, at most 112,640 bytes |

The application must be linked with its vector table at `0x08004800`. Its
`SystemInit` must not reset `VTOR` to the Bootloader vector table.

The image-validity record at `0x08003800` consists of two little-endian 32-bit
words: magic `0x53484132` and the exact raw application length. The remaining
bytes of the 2 KiB page are erased. Factory programming must provide a valid
record as well as the version, digest and application; a raw application alone
will not boot. The AT32 factory image's offsets and record location are not
compatible with this layout.

## OTA

Send the same `0x10`/`0xAA` protocol used by the AT32 project, with protocol
version 1 and packet numbers starting at 0. The OTA payload is the concatenation
of the 1 KiB version area, the 1 KiB SHA-256 area, and the raw application.
The SHA-256 digest covers exactly the raw application bytes, starting at
`0x08004800`; it is not an authenticity signature. Each packet also carries
the reference protocol's CRC32 and CRC16. The reference PC tool's 56 KiB
image-size limit is more restrictive than this device's partition.

After an accepted start request, the Bootloader erases the validity record
before erasing the only application partition. It checks sequence, key, lengths,
version padding, CRCs, application vectors and SHA-256, and writes the validity
record only after the complete image passes verification. A failure leaves the
device in the Bootloader; there is no backup or rollback partition. The final
packet receives an error rather than a success acknowledgement if verification
or record programming fails.

The sender must wait for the start acknowledgement before sending data and
wait for each packet acknowledgement before sending the next packet. Receive
is paused during the erase operation and queued input may be discarded.

STM32 Flash is programmed in 8-byte units. When a non-final packet ends between
units, up to 7 bytes are kept in RAM until the next packet. Its acknowledgement
therefore confirms acceptance but not persistence of those trailing bytes.
An interrupted transfer always requires starting again; no incomplete image
can be selected using the validity record.

The Bootloader starts a valid application after a four-second idle window
configured by `UPGRADE_BOOT_WAIT_MS` in `upgrade.h`.
A query keeps it in the handshake state. With no valid application it stays in
the Bootloader and periodically reports the damaged status `0x55`. The
single-partition layout does not support the reference project's rollback
operation.

The matching desktop tool is in `STM32_OTA_PC`; its README documents image
generation, serial upgrade, and explicitly confirmed SWD factory programming.

## Module boundaries and host tests

All files retain their existing directories. `modbus.h` exposes the portable
protocol types, parsing, checksums and response builders. Hardware-facing callers
include `modbus_port.h`, which exposes the USART2 DMA receive path, queues,
status accessors and main-loop response scheduling. `upgrade.c` owns the upgrade
state machine; `upgrade_flash.c` and `upgrade_platform.c` retain the STM32G4
Flash and application-jump implementations. Add both `modbus.c` and
`modbus_port.c` when compiling the full firmware; the Keil project includes both.

On Windows with GCC and Python available, run `python tests/run_tests.py` from
the repository root. It builds and runs the protocol/DMA, upgrade safety, Flash
adapter and OTA integration tests in a temporary directory. Run desktop tests
with `python -m unittest discover -s STM32_OTA_PC -p "test*.py"`.

## Version query and application integration

OTA management and business Modbus both use station 3, USART2 at 1 Mbaud/8N1
and PA1 software RS485 direction control (high TX, low RX).
Custom control requests use register `0x00E6`, outside the business register
range; version queries use function `0x11`. The APP bridge is maintained in `ota_app_bridge.h` and copied
to the companion APP's `Inc` directory. Its `product_modbus.c` routes the custom
management frames before normal business requests and resets only after the
complete control acknowledgement was transmitted successfully. APP start and
rollback requests report forbidden status without touching Flash.

Function `0x11` requests are 8 bytes: station, function, big-endian offset and
count (16 bits each), then little-endian Modbus CRC16. Count is 1..32; the range
must fit the 1024-byte version block at `0x08004000`. Replies contain station,
function, mode (0 Bootloader, 1 APP), big-endian offset (16 bits), count (8 bits),
version bytes and CRC16, totalling count+8 bytes. Invalid ranges and bad CRCs
are silent. Reading the version never writes Flash. UTF-8 is decoded only after
all chunks have been assembled on the desktop.

Both Keil and GCC APP builds must link to `0x08004800`; the APP vector table
offset is `0x4800`. The companion Keil post-build packaging generates updated
`firmware_ota.bin` and full factory BIN/HEX with the matching Bootloader. Raw
GCC APP BIN/HEX still require packaging before OTA/factory use.

Pressure testing snapshots the input image, increments only the version block,
automatically restarts APP into the Bootloader and counts success only after
APP mode and exact version readback agree. It supports graceful stop after the
current round, disconnect cancellation and bounded detailed/command/JSONL logs.

The APP's `modbus_stream.h` assembles both business and OTA requests across
DMA/IDLE blocks in task context. RX slots retain completion timestamps and a
transport-loss epoch; a timeout or receive loss invalidates the pending tail.
CRC validation precedes dispatch, and fragmented reset commands are acknowledged
in full before reset. Timing accounts for the new block's 8N1 wire duration and
millisecond timestamp quantization.

If version readback is invalid or times out before a stress round, the desktop
requires an explicit Bootloader status (1, 2 or 0x55) before retrying a complete
transfer from packet zero. Unknown version text alone never authorizes recovery.
APP/forbidden status, Flash-busy status or silence prevents that round's transfer.

## Device-side upgrade logging

USART2 remains the OTA/RS485 transport: PA2 TX, PA3 RX, PA1 DE/RE,
station 3, 1 Mbaud and 8N1. USART1 is the independent debug log port:
PB6 TX, PB7 RX, 115200 baud and 8N1. Connect a USB-TTL RX to PB6 and
share ground. `[BOOT]` messages never pass through the RS485 response path.

`boot_log.h` follows the reference project's ASCII notice style with optional
decimal values and no printf dependency. It reports installed-image checking,
control/query handling, erase results, packet/last-packet indices, cumulative
written bytes, ACK code/packet, SHA256 result, record commit and APP jump.
The implementation lives in the already compiled `modbus_port.c`, so no extra
Keil source entry is needed. `BOOT_LOG_ENABLED=0` disables it. Calls from locked
or interrupt context are skipped; timeout counters are logged after unlocking.
USART1 transmit failures increment `boot_log_drops` and do not invoke Error_Handler.
The blocking HAL transmit completes before returning, including the final log
before resetting or jumping. Existing FIFO-disabled UART mode is retained;
unused FIFO threshold configuration is omitted to fit the 14 KiB Bootloader.

The Keil Bootloader target uses MicroLIB to keep logs and diagnostics within the
fixed 14 KiB partition; the APP and Flash layout are unchanged.
