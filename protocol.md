# C6 Tracker BLE protocol v3

This document describes the protocol implemented by `esp32_c6_tracker.ino` and consumed by the Android client.

Byte order: little-endian for all multi-byte integer fields.
Coordinates: signed E7 degrees (`lat * 10_000_000`, `lon * 10_000_000`).
Text commands and response payloads are UTF-8/ASCII unless noted otherwise.

## Service

Primary service UUID: `9f6d1000-2b9e-4a5f-8c7d-6f3e4b2a1000`.

| Characteristic | UUID | Access | Payload |
| --- | --- | --- | --- |
| Telemetry | `9f6d1001-2b9e-4a5f-8c7d-6f3e4b2a1000` | read, notify | 20 bytes |
| Navigation | `9f6d1002-2b9e-4a5f-8c7d-6f3e4b2a1000` | read, notify | 20 bytes |
| Settings | `9f6d1003-2b9e-4a5f-8c7d-6f3e4b2a1000` | read | 12-14 bytes by version |
| Command | `9f6d1004-2b9e-4a5f-8c7d-6f3e4b2a1000` | write, write-no-response | text command |
| Response | `9f6d1005-2b9e-4a5f-8c7d-6f3e4b2a1000` | read, notify | text response |
| Short history | `9f6d1006-2b9e-4a5f-8c7d-6f3e4b2a1000` | read, notify | typed packets |
| Track transfer | `9f6d1007-2b9e-4a5f-8c7d-6f3e4b2a1000` | read, notify | typed packets |
| Cycle status | `9f6d1008-2b9e-4a5f-8c7d-6f3e4b2a1000` | read, notify | 20 bytes |

The device also publishes the standard Battery Service `0x180F`, Battery Level `0x2A19`.

## Telemetry packet

Characteristic: Telemetry. Size: 20 bytes. Packet version: byte 0 must be `1`.

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | u8 | version = 1 |
| 1 | u8 flags | bit0 GPS fix, bit1 recording, bit2 SD OK, bit3 altitude valid, bit4 SD write error, bit5 SD low space, bit6 SD tail repaired, bit7 USB host connected |
| 2 | u32 | total distance in meters |
| 6 | i16 | altitude in meters |
| 8 | u16 | battery millivolts |
| 10 | u8 | battery percent |
| 11 | u8 | satellite count |
| 12 | u16 | speed, km/h * 10 |
| 14 | u16 | HDOP * 100 |
| 16 | u16 | current route number |
| 18 | u16 | track point count, clamped |

## Navigation packet

Characteristic: Navigation. Size: 20 bytes.

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | i32 | latitude E7, or 0 when invalid |
| 4 | i32 | longitude E7, or 0 when invalid |
| 8 | i16 | delta altitude from route start, meters |
| 10 | u16 | course degrees * 10 |
| 12 | u32 | route recording seconds |
| 16 | u32 | device uptime seconds |

## Cycle status packet

Characteristic: Cycle status. Size: 20 bytes. Packet version: byte 0 must be `1`.

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | u8 | version = 1 |
| 1 | u8 | state: 1 GPS wait, 2 track transfer, 3 phone connected, 4 BLE connect window, 5 preparing sleep, 6 sleeping, 7 USB hold |
| 2 | u8 | wake reason: 1 timer, 2 button, 3 power/reset |
| 3 | u8 flags | bit0 GPS fix, bit1 BLE connected, bit2 filter enabled, bit3 cycle complete, bit4 cycle acknowledged, bit5 USB host connected |
| 4 | u8 | points collected in current cycle |
| 5 | u8 | min points per wake |
| 6 | u16 | GPS wait seconds |
| 8 | u16 | GPS timeout seconds |
| 10 | u32 | sleep interval seconds |
| 14 | u16 | wake cycle id |
| 16 | u32 | battery sample sequence |

## Commands

Commands are written to the Command characteristic as short text.

| Command | Meaning |
| --- | --- |
| `rec=1` / `rec=0` | start/stop recording |
| `mark` | save a user mark at the current GPS position |
| `mark=NAME|COMMENT` | save a named user mark |
| `new` | start a new route file |
| `settings?` | publish settings |
| `history` | stream short in-memory history on characteristic `1006` |
| `tracks` | stream track list on characteristic `1007` |
| `load=N` | stream full route `N` on characteristic `1007` |
| `sync=ROUTE|POINTS|MARKS` | stream only records after already cached counts |
| `logs` | stream service log on characteristic `1007` |
| `cancel` | cancel active track/log transfer |
| `ack=N` | acknowledge wake cycle id `N` |
| `rename=N|NAME` | set route display name |
| `delete=N` | delete route file and metadata, except current route |
| `gpsto=N` | set GPS timeout seconds, 30..3600 |
| `sats=N` | set minimum satellites, 1..20 |
| `hdop=N` | set maximum HDOP * 100, 50..2000 |
| `sleep=N` | set sleep interval seconds, 10..86400 |
| `screen=0/1` | enable screen during timer wake |
| `ble=0/1` | enable BLE during timer wake |
| `minpts=N` | set minimum points per wake, 1..100 |
| `filter=0/1` | enable GPS filtering |
| `sleepbt=0/1` | allow sleep while BLE is connected |

## Track transfer packets

Characteristic: Track transfer. All packets start with byte 0 = `1`, byte 1 = type.

| Type | Direction | Payload |
| --- | --- | --- |
| 1 | device -> client | list start: u16 current route at 2, u32 total MB at 4, u32 free MB at 8 |
| 2 | device -> client | list item: u16 route at 2, u32 file size at 4, u8 flags at 8 bit0 current |
| 3 | device -> client | list end |
| 4 | device -> client | load start: u16 route at 2, u32 file size at 4, u8 current flag at 8, u8 incremental flag at 9, u32 skipped points at 10, u32 skipped marks at 14 |
| 5 | device -> client | point: u8 point type at 2 (`0` track, `1` mark), i32 latE7 at 4, i32 lonE7 at 8, i16 altitude at 12, u16 speed*10 at 14, u32 GPS epoch at 16 |
| 7 | device -> client | load end: u16 route at 2, u32 total track points at 4, u32 total marks at 8 |
| 8 | device -> client | mark name: u16 mark index at 2, u8 byte length at 4, text bytes at 5 |
| 9 | device -> client | mark comment: u16 mark index at 2, u8 byte length at 4, text bytes at 5 |
| 10 | device -> client | route display name: u16 route at 2, u8 byte length at 4, text bytes at 5 |
| 11 | device -> client | route meta: u16 route at 2, u32 point count at 4, u32 mark count at 8, u32 created epoch at 12, u32 updated epoch at 16 |
| 12 | device -> client | point detail: u8 point type at 2, u16 point index at 3, u16 HDOP*100 at 5, u8 satellites at 7, u16 record id at 8, u16 cycle id at 10 |
| 13 | device -> client | point status: u8 point type at 2, u16 point index at 3, u8 byte length at 5, text bytes at 6 |
| 20 | device -> client | service log start: u32 byte count at 2 |
| 21 | device -> client | service log chunk: up to 18 text bytes at 2 |
| 22 | device -> client | service log end |

Current limitation: track transfer is notification-only and has no per-packet ACK, sequence number, or CRC. A future protocol revision should add session id, sequence, offset, window size, CRC32 and resume commands.
