#!/usr/bin/env python3
"""Console and scripted tester for F1 Product Modbus v2 (slave 3, 1 Mbaud 8N1)."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import struct
import sys
import threading
import time

SLAVE = 3
REG_CONTROL = 0
REG_STATUS = 17
REG_DRIVE = 33
REG_MOTION = 40
REG_SNAPSHOT = 64
SNAPSHOT_COUNT = 39


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def frame(function: int, address: int, value: int) -> bytes:
    body = struct.pack(">BBHH", SLAVE, function, address, value)
    return body + struct.pack("<H", crc16(body))


def multiple_frame(address: int, values: list[int]) -> bytes:
    words = [value & 0xFFFF for value in values]
    payload = struct.pack(f">{len(words)}H", *words)
    body = struct.pack(">BBHHB", SLAVE, 0x10, address, len(words), len(payload)) + payload
    return body + struct.pack("<H", crc16(body))


def signed(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


@dataclass
class Setup:
    total: int
    sequence: int
    bar: int
    roll: int
    motor: int


class Client:
    def __init__(self, port: object):
        self.port = port
        self.lock = threading.Lock()
        self.sequence = int(time.monotonic() * 1000) & 0xFFFF
        self.cached_setup: Setup | None = None

    def exchange(self, request: bytes, expected: int) -> bytes:
        with self.lock:
            self.port.reset_input_buffer()
            self.port.write(request)
            response = self.port.read(3)
            wanted = 5 if len(response) == 3 and response[1] & 0x80 else expected
            if len(response) == 3:
                response += self.port.read(wanted - 3)
            if len(response) != wanted:
                raise RuntimeError(f"timeout: received {len(response)}/{wanted} bytes")
            if crc16(response[:-2]) != int.from_bytes(response[-2:], "little"):
                raise RuntimeError("bad response CRC")
            if response[0] != SLAVE:
                raise RuntimeError(f"unexpected slave {response[0]}")
            if response[1] & 0x80:
                names = {1: "illegal function", 2: "illegal address", 3: "illegal value"}
                raise RuntimeError(names.get(response[2], f"device exception {response[2]}"))
            return response

    def read(self, address: int, quantity: int) -> list[int]:
        response = self.exchange(frame(0x03, address, quantity), 5 + quantity * 2)
        if response[1] != 0x03 or response[2] != quantity * 2:
            raise RuntimeError("unexpected read response")
        return list(struct.unpack(f">{quantity}H", response[3:-2]))

    def read_input(self, address: int, quantity: int) -> list[int]:
        response = self.exchange(frame(0x04, address, quantity), 5 + quantity * 2)
        if response[1] != 0x04 or response[2] != quantity * 2:
            raise RuntimeError("unexpected input-register response")
        return list(struct.unpack(f">{quantity}H", response[3:-2]))

    def write(self, address: int, value: int) -> None:
        request = frame(0x06, address, value & 0xFFFF)
        if self.exchange(request, 8) != request:
            raise RuntimeError("write echo mismatch")

    def write_multiple(self, address: int, values: list[int]) -> None:
        request = multiple_frame(address, values)
        expected = frame(0x10, address, len(values))
        if self.exchange(request, 8) != expected:
            raise RuntimeError("multiple-write acknowledgement mismatch")

    def mission(self, setup: Setup) -> None:
        values = [setup.total, setup.sequence, setup.bar, setup.roll,
                  setup.motor & 0xFFFF, 0, 0, 0]
        self.write_multiple(REG_CONTROL, values)
        self.sequence = setup.sequence
        self.cached_setup = setup

    def next_sequence(self) -> int:
        self.sequence = (self.sequence + 1) & 0xFFFF
        return self.sequence

    def setup(self) -> Setup:
        values = self.read(REG_CONTROL, 5)
        self.cached_setup = Setup(values[0], values[1], values[2], values[3], signed(values[4]))
        self.sequence = self.cached_setup.sequence
        return self.cached_setup

    def setup_cached(self) -> Setup:
        return self.cached_setup if self.cached_setup is not None else self.setup()


def info(client: Client) -> str:
    d = client.read_input(11, 2)
    return (f"INFO DEVICE=STM32G431 PRODUCT_MODBUS "
            f"FW={d[0] >> 8}.{d[0] & 0xFF}.{d[1] >> 8}.{d[1] & 0xFF} "
            f"SLAVE={SLAVE}")


def snapshot(client: Client) -> list[int]:
    return client.read(REG_SNAPSHOT, SNAPSHOT_COUNT)


def elapsed(value: int) -> str:
    return "NA" if value == 0xFFFF else str(value)


def status(client: Client) -> str:
    s = snapshot(client)
    flags, report = s[1], s[19]
    phases = {0: "IDLE", 1: "START", 2: "HOLD", 3: "DECEL", 4: "BREAK_RETRY"}
    reasons = {0: "ACTIVE", 1: "NONE", 2: "COMMAND", 3: "TIMEOUT", 4: "FAULT",
               5: "START_TIMEOUT", 6: "UNEXPECTED_STATE", 7: "RAMPSTOP_DONE",
               8: "RAMPSTOP_LIMIT", 9: "BREAK_RETRY_ABORT", 10: "BREAK_ACK_FAILED",
               11: "BREAK_RESTART_REJECTED", 255: "OTHER"}
    return (f"STATUS STATE={s[0]} PHASE={phases.get(s[2], s[2])} RUN={int(bool(flags&16))} "
            f"OUT={int(bool(flags&2))} BKIN={int(bool(flags&256))} OBS_VALID={int(bool(flags&32))} "
            f"TARGET={signed(s[3])} REF={signed(s[4])} RPM={signed(s[5])} OBS={signed(s[6])} "
            f"VBUS={s[7]}V CURRENT={s[8]}mA FAULT=0x{s[9]:04X} OCCURRED=0x{s[10]:04X} "
            f"LEVEL={signed(s[37])} REACHED={int(bool(report&16))} "
            f"REASON={reasons.get(s[38], s[38])}")


HELP = ("CMD: INFO|CFG|STATUS|DIAG|RUN rpm [MODE CL/OPEN] [PROFILE FAST/STANDARD] "
        "[IQ n] [RAMP ms] [ACC rpm/s] [DEC rpm/s] [HOLD ms]|SET name value|"
        "DRIVE idkp idki iqkp iqki spkp spki|RAMPSTOP|MISSION total bar roll motor|"
        "MOTOR level|BAR level|STOP|ACK|WATCH [interval]|WATCH OFF|QUIT")


def cfg(client: Client) -> str:
    all_values = client.read(REG_DRIVE, 16)
    d, m = all_values[:6], all_values[7:16]
    return (f"CFG MODE={'OPEN' if m[0] else 'CL'} PROFILE={'FAST' if m[1] else 'STANDARD'} "
            f"RPM={signed(m[2])} IQRAW={signed(m[3])} RAMP_MS={m[4]} ACC={m[5]} "
            f"DEC={m[6]} RUN_MS={m[7]} IDKP={signed(d[0])} IDKI={signed(d[1])} "
            f"IQKP={signed(d[2])} IQKI={signed(d[3])} SPKP={signed(d[4])} SPKI={signed(d[5])}")


def execute(client: Client, line: str) -> str:
    parts = line.strip().upper().split()
    if not parts:
        return ""
    cmd = parts[0]
    if cmd in {"QUIT", "EXIT"}:
        raise EOFError
    if cmd == "HELP":
        return HELP
    if cmd == "INFO":
        return info(client)
    if cmd == "STATUS":
        return status(client)
    if cmd == "CFG":
        return cfg(client)
    if cmd == "DIAG":
        s = snapshot(client)
        iq_ms, vlim = s[26] | s[27] << 16, s[28] | s[29] << 16
        crc, frame_err, overflow = s[30] | s[31] << 16, s[32] | s[33] << 16, s[34] | s[35] << 16
        return (f"DIAG ID={signed(s[11])} IQ={signed(s[12])} IDREF={signed(s[13])} "
                f"IQREF={signed(s[14])} VQ_REQ={signed(s[15])} VD_REQ={signed(s[16])} "
                f"VQ={signed(s[17])} VD={signed(s[18])} START_MS={elapsed(s[20])} "
                f"LOOP_MS={elapsed(s[21])} BAND_MS={elapsed(s[22])} SETTLE_MS={elapsed(s[23])} "
                f"REACHED_MS={elapsed(s[24])} REACHED_RPM={signed(s[25])} IQ_LIMIT_MS={iq_ms} "
                f"VLIM_TICKS={vlim} CRC_ERR={crc} FRAME_ERR={frame_err} RX_OVERFLOW={overflow}")
    if cmd == "SETUP":
        s = client.setup()
        return f"SETUP TOTAL={s.total} SEQ={s.sequence} BAR={s.bar} ROLL={s.roll} MOTOR={s.motor}"
    if cmd == "STOP":
        client.write(5, 2)
        client.cached_setup = None
        return "OK STOP"
    if cmd in {"ACK", "CLEAR"}:
        client.write(5, 3)
        if client.read(19, 1)[0] != 1:
            raise RuntimeError("fault acknowledgement rejected")
        return "OK ACK"
    if cmd == "RAMPSTOP":
        client.write(5, 4)
        if client.read(19, 1)[0] != 1:
            raise RuntimeError("ramp stop rejected")
        return "OK RAMPSTOP"
    if cmd == "DRIVE" and len(parts) == 7:
        values = [int(value) for value in parts[1:]]
        client.write_multiple(REG_DRIVE, values)
        return "OK DRIVE " + " ".join(str(value) for value in values)
    if cmd == "SET" and len(parts) == 3:
        names = {"IDKP":33,"IDKI":34,"IQKP":35,"IQKI":36,"SPKP":37,"SPKI":38,
                 "MODE":40,"PROFILE":41,"RPM":42,"IQ":43,"IQRAW":43,"RAMP":44,
                 "RAMP_MS":44,"ACC":45,"DEC":46,"HOLD":47,"RUN_MS":47}
        if parts[1] not in names:
            raise ValueError(f"unknown setting {parts[1]}")
        value_text = parts[2]
        if parts[1] == "MODE": value = {"CL":0,"OPEN":1}.get(value_text, -1)
        elif parts[1] == "PROFILE": value = {"STANDARD":0,"FAST":1}.get(value_text, -1)
        else: value = int(value_text)
        client.write(names[parts[1]], value)
        return f"OK SET {parts[1]}={value_text}"
    if cmd in {"RUN", "RPM"} and len(parts) >= 2:
        m = client.read(REG_MOTION, 8)
        m[2] = int(parts[1]) & 0xFFFF
        aliases = {"MODE":0,"PROFILE":1,"IQ":3,"IQRAW":3,"RAMP":4,"RAMP_MS":4,
                   "ACC":5,"DEC":6,"HOLD":7,"RUN_MS":7}
        index = 2
        while index < len(parts):
            key = parts[index]
            if key not in aliases or index + 1 >= len(parts):
                raise ValueError(f"bad RUN option near {key}")
            text = parts[index + 1]
            if key == "MODE": value = {"CL":0,"OPEN":1}.get(text, -1)
            elif key == "PROFILE": value = {"STANDARD":0,"FAST":1}.get(text, -1)
            else: value = int(text)
            m[aliases[key]] = value & 0xFFFF
            index += 2
        client.write_multiple(REG_MOTION, m + [1])
        if client.read(19, 1)[0] != 1:
            raise RuntimeError("motion start rejected; read STATUS and ACK faults")
        return (f"OK RUN RPM={signed(m[2])} MODE={'OPEN' if m[0] else 'CL'} "
                f"PROFILE={'FAST' if m[1] else 'STANDARD'} IQ={signed(m[3])} "
                f"RAMP={m[4]} ACC={m[5]} DEC={m[6]} HOLD={m[7]}")
    if cmd == "MISSION" and len(parts) == 5:
        total, bar, roll, motor = map(int, parts[1:])
        sequence = client.next_sequence()
        client.mission(Setup(total, sequence, bar, roll, motor))
        return f"OK MISSION SEQ={sequence} TOTAL={total} BAR={bar} ROLL={roll} MOTOR={motor}"
    if cmd in {"MOTOR", "BAR"} and len(parts) == 2:
        value = int(parts[1])
        client.write(2 if cmd == "BAR" else 4, value)
        client.cached_setup = None
        return f"OK {cmd}={value}"
    if cmd in {"POLL", "WATCH"}:
        raise ValueError("WATCH is available in interactive mode; use WATCH [interval] or WATCH OFF")
    raise ValueError(f"unknown command: {' '.join(parts)}; use HELP")


def open_port(serial_module: object, name: str, baud: int, timeout: float) -> object:
    port = serial_module.Serial(baudrate=baud, timeout=timeout, write_timeout=timeout)
    port.port = name
    port.dtr = False
    port.rts = False
    port.open()
    return port


def self_test() -> None:
    assert crc16(b"123456789") == 0x4B37
    assert frame(3, 8, 9).hex() == "03030008000905ec"
    product = multiple_frame(0, [1, 7, 0, 0, 3, 0, 0, 0])
    motion = multiple_frame(REG_MOTION, [0, 1, 10000, 10922, 250, 23000, 10000, 5000, 1])
    assert len(product) == 25 and product[:7] == bytes.fromhex("03100000000810")
    assert len(motion) == 27 and motion[:7] == bytes.fromhex("03100028000912")
    assert crc16(motion[:-2]) == int.from_bytes(motion[-2:], "little")

    class FakeClient:
        def __init__(self) -> None:
            self.calls: list[tuple[int, int]] = []

        def read(self, address: int, quantity: int) -> list[int]:
            self.calls.append((address, quantity))
            if (address, quantity) == (REG_SNAPSHOT, SNAPSHOT_COUNT):
                values = [0] * SNAPSHOT_COUNT
                values[20:25] = [0xFFFF] * 5
                return values
            if (address, quantity) == (REG_DRIVE, 16):
                return [3321, 122, 3321, 122, 200, 25, 0,
                        0, 1, 10000, 10922, 500, 32000, 10000, 0, 0]
            raise AssertionError((address, quantity))

    fake = FakeClient()
    status(fake)  # type: ignore[arg-type]
    assert fake.calls == [(REG_SNAPSHOT, SNAPSHOT_COUNT)]
    fake = FakeClient()
    cfg(fake)  # type: ignore[arg-type]
    assert fake.calls == [(REG_DRIVE, 16)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", help="serial port such as COM8")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--timeout", type=float, default=0.6)
    parser.add_argument("--send", action="append", default=[], help="execute command; repeatable")
    parser.add_argument("--self-test", action="store_true", help="validate frames without a serial port")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        print("SELFTEST OK")
        return 0
    if not args.port:
        parser.error("port is required unless --self-test is used")
    try:
        import serial
    except ImportError:
        print("pyserial is required: py -m pip install pyserial", file=sys.stderr)
        return 2
    try:
        port = open_port(serial, args.port, args.baud, args.timeout)
        with port:
            client = Client(port)
            if args.send:
                for command in args.send:
                    print(execute(client, command))
                return 0
            print(f"Connected {args.port} @ {args.baud}, 8N1, slave={SLAVE}. {HELP}")
            watch_stop = threading.Event()
            watch_thread: threading.Thread | None = None

            def stop_watch() -> None:
                nonlocal watch_thread
                if watch_thread is not None:
                    watch_stop.set()
                    watch_thread.join(timeout=max(args.timeout + 0.2, 1.0))
                    watch_thread = None

            def start_watch(interval: float) -> None:
                nonlocal watch_thread, watch_stop
                stop_watch()
                watch_stop = threading.Event()

                def worker() -> None:
                    while not watch_stop.is_set():
                        try:
                            print("\n" + time.strftime("%H:%M:%S"), status(client), flush=True)
                        except RuntimeError as exc:
                            print(f"\nWATCH ERR {exc}", flush=True)
                        watch_stop.wait(interval)

                watch_thread = threading.Thread(target=worker, name="modbus-watch", daemon=True)
                watch_thread.start()

            try:
                while True:
                    try:
                        line = input("MODBUS> ")
                        parts = line.strip().upper().split()
                        if parts and parts[0] in {"WATCH", "POLL"}:
                            if len(parts) == 2 and parts[1] in {"OFF", "STOP"}:
                                stop_watch()
                                print("OK WATCH OFF")
                            else:
                                interval = float(parts[1]) if len(parts) == 2 else 0.25
                                if interval < 0.05:
                                    raise ValueError("watch interval must be >= 0.05 seconds")
                                start_watch(interval)
                                print(f"OK WATCH interval={interval:g}s; use WATCH OFF to stop")
                            continue
                        answer = execute(client, line)
                        if answer:
                            print(answer)
                    except EOFError:
                        return 0
                    except (RuntimeError, ValueError) as exc:
                        print(f"ERR {exc}")
            finally:
                stop_watch()
    except (OSError, RuntimeError) as exc:
        print(f"ERR {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
