# NB debug commands

Implemented by `Src/usrdug.c` on the former MCSDK UI USART1 port (PB6 TX,
PB7 RX), 115200 baud, 8N1, using a 3.3 V TTL UART adapter and common ground.
The original MCSDK UI protocol is disabled for the CMake product targets.
USART2 RS485 remains dedicated to Modbus. Send ASCII text with CR, LF,
or CRLF. Command names are case insensitive. Use ASCII commas, not full-width
Chinese commas. Send one request and wait for its reply before the next;
NB commands are not accepted on the RS485 port.

| Command | Meaning |
|---|---|
| `nb+motor=5000` | Request signed mechanical rpm through the existing closed-loop controller |
| `nb+motor=0` | Stop the motor without turning off the heaters |
| `nb+heat1=5` | Set HeatBar to level 5 |
| `nb+heat1=0` | Turn HeatBar off immediately |
| `nb+heat2=5` | Set HeatRoll on F1_TEST only; POWER V004 rejects nonzero levels |
| `nb+table=5,2999,169` | Set `HeatPowerTable[4]` to `{2999,169}` (ARR, PSC) |
| `nb+table=5,2999` | Change only ARR of level 5; preserve PSC |
| `nb+table=5,,169` | Change only PSC of level 5; preserve ARR |

Nonzero motor/heat commands automatically enable the total switch. Motor
requests still require a valid controller state and the existing minimum
closed-loop speed, with maximum magnitude 20000 rpm. A rejected request does
not report OK. Positive/negative speeds select direction; stop before reversing.

Heat levels are 0..30. Table indices are user levels 1..30; ARR is 1..65535,
PSC is 0..65535. Both heaters share this table. A table update takes effect
through the existing zero-cross timing path; this does not change edge selection
or timer prescaler reload behavior. Table edits are RAM-only and lost on reset.
Both timers select the falling edge for levels 1..13 and the rising edge for
levels 14..29. Level 14 is the current ARR=9000 boundary. The boundary is by
level, not recomputed after RAM table edits. Level 30 immediately drives the available heater output high
continuously, bypassing timer delay. Level 0 or the total switch off drives it low.
Table row 30 remains writable but does not determine the full-on output timing.

Successful replies echo normalized arguments followed by `,OK\r\n`.
Failures return `NB+ERR,<reason>\r\n`. OK means the request was accepted, not
that the motor has reached speed or that heater power has been measured.

Build with `build.cmd`; download the appropriate board using `flash.cmd`.
This module is included in both CMake targets and the Keil `MDK-ARM/zhongbaiSt.uvprojx`
target. The Keil target uses POWER V004 definitions (no `F1_TEST_BOARD`) and
ST-Link debugging with debug information enabled. Its output is
`MDK-ARM/zhongbaiSt/zhongbaiSt.axf` and `.hex`.
