import hashlib
import struct
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

import images
import app
import protocol
import serial_ota
import swd


def binary(address, length=64):
    return struct.pack("<II", 0x20008000, address + 9) + bytes(length - 8)


APP = binary(images.FLASH_BASE + images.APP_OFFSET, 67)
BOOT = binary(images.FLASH_BASE)


def reply_query(status=1, key=0):
    return protocol.frame(struct.pack(">BBHHHI", protocol.ADDRESS, 0x10, 0xe6, 3, status, key))


def reply_control():
    return protocol.frame(struct.pack(">BBHH", protocol.ADDRESS, 0x10, 0xe6, 4))


def reply_data(packet, ack=1):
    return protocol.frame(struct.pack(">BBBH", protocol.ADDRESS, 0xaa, ack, packet))


class ImageTests(unittest.TestCase):
    def test_prepare_upgrade_types(self):
        ota, version = images.prepare_upgrade(APP, "app", "版本1")
        self.assertEqual(version, "版本1")
        self.assertEqual(ota[:1024], "版本1".encode("utf-8").ljust(1024, b"\0"))
        self.assertEqual(ota[1024:1056], hashlib.sha256(APP).digest())
        self.assertEqual(ota[1056:2048], b"\0" * 992)
        self.assertEqual(ota[2048:], APP)
        existing, version = images.prepare_upgrade(ota, "ota", "ignored")
        self.assertEqual(existing, ota)
        self.assertEqual(version, "版本1")
        for data, kind, value in ((APP, "ota", "v"), (ota, "app", "v"),
                                  (APP, "app", "bad\0version"), (APP, "unknown", "v"),
                                  (b"\0" * 8, "app", "v")):
            with self.subTest(kind=kind, value=value), self.assertRaises(ValueError):
                images.prepare_upgrade(data, kind, value)
        bad = bytearray(ota)
        bad[-1] ^= 1
        with self.assertRaises(ValueError):
            images.prepare_upgrade(bad, "ota", "ignored")

    def test_layout_and_digest(self):
        ota = images.make_ota(APP, "v1.2")
        self.assertEqual(len(ota), 2048 + len(APP))
        self.assertEqual(ota[:5], b"v1.2\0")
        self.assertEqual(ota[1024:1056], hashlib.sha256(APP).digest())
        self.assertEqual(images.validate_ota(ota), "v1.2")
        factory = images.make_factory(BOOT, ota)
        self.assertEqual(len(factory), 128 * 1024)
        self.assertEqual(struct.unpack_from("<II", factory, 0x3800), (0x53484132, len(APP)))
        self.assertEqual(factory[0x4000:0x4000 + len(ota)], ota)
        self.assertEqual(images.validate_factory(factory), "v1.2")

    def test_boundaries_and_errors(self):
        largest = binary(images.FLASH_BASE + images.APP_OFFSET, images.APP_CAPACITY)
        images.validate_ota(images.make_ota(largest, "v"))
        with self.assertRaises(ValueError):
            images.validate_app(APP + bytes(images.APP_CAPACITY))
        with self.assertRaises(ValueError):
            images.make_ota(b"\0" * 8, "version")
        with self.assertRaises(ValueError):
            images.make_ota(APP, "a\0b")
        ota = bytearray(images.make_ota(APP, "v"))
        ota[-1] ^= 1
        with self.assertRaises(ValueError):
            images.validate_ota(ota)
        ota = bytearray(images.make_ota(APP, "v"))
        ota[2] = 1
        with self.assertRaises(ValueError):
            images.validate_ota(ota)
        with self.assertRaises(ValueError):
            images.make_factory(BOOT + bytes(images.BOOT_SIZE), images.make_ota(APP, "v"))
        with self.assertRaises(ValueError):
            images.validate_factory(images.make_factory(BOOT, images.make_ota(APP, "v"))[:-1])

    def test_hex_and_overwrite_guard(self):
        factory = images.make_factory(BOOT, images.make_ota(APP, "v"))
        lines = images.intel_hex(factory).splitlines()
        restored = bytearray()
        upper = 0
        for line in lines:
            data = bytes.fromhex(line[1:])
            self.assertEqual(sum(data) & 255, 0)
            addr, kind, payload = int.from_bytes(data[1:3], "big"), data[3], data[4:-1]
            if kind == 4:
                upper = int.from_bytes(payload, "big") << 16
            elif kind == 0:
                self.assertEqual(upper + addr, images.FLASH_BASE + len(restored))
                restored.extend(payload)
        self.assertEqual(restored, factory)
        self.assertEqual(lines[-1], ":00000001FF")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "input.bin"
            path.write_bytes(APP)
            with self.assertRaises(ValueError):
                images.save_output(path, b"x", [path])
            self.assertEqual(path.read_bytes(), APP)


class ProtocolTests(unittest.TestCase):
    def test_crc_and_frame(self):
        self.assertEqual(protocol.crc16(b"123456789"), 0x4b37)
        packet = protocol.data_frame(b"123456789", 2, 1, 0x12345678)
        self.assertEqual(packet[:16], bytes.fromhex("03aa001f000112345678000900020001"))
        self.assertEqual(packet[-6:-2], bytes.fromhex("cbf43926"))
        self.assertEqual(int.from_bytes(packet[-2:], "little"), protocol.crc16(packet[:-2]))
        self.assertEqual(len(protocol.query()), 14)
        self.assertEqual(len(protocol.control(1, 3)), 20)

    def test_stream_split_noise_and_crc(self):
        rx = protocol.RxBuffer()
        bad = bytearray(reply_data(2))
        bad[-1] ^= 1
        stream = b"\xff" + bytes(bad) + reply_query(0x55, 42) + reply_data(4)
        result = []
        for byte in stream:
            result.extend(rx.feed(bytes([byte])))
        self.assertEqual(result, [{"type": "query", "status": 0x55, "key": 42},
                                  {"type": "data", "ack": 1, "packet": 4}])


class FakeSerial:
    def __init__(self, mode="ok"):
        self.mode = mode
        self.writes = []
        self.incoming = bytearray()

    @property
    def in_waiting(self):
        return len(self.incoming)

    def reset_input_buffer(self):
        self.incoming.clear()

    def write(self, data):
        self.writes.append(data)
        if data[1] == 0x10 and data[4:6] == b"\0\3":
            self.incoming.extend(reply_query())
        elif data[1] == 0x10:
            if self.mode == "start_error":
                self.incoming.extend(reply_query(0x55))
            else:
                self.incoming.extend(reply_control())
        elif self.mode != "timeout":
            packet = int.from_bytes(data[14:16], "big")
            self.incoming.extend(reply_data(packet, 2 if self.mode == "bad_ack" else 1))
        return len(data)

    def flush(self):
        pass

    def read(self, size):
        if not self.incoming:
            time.sleep(0.001)
        data = bytes(self.incoming[:size])
        del self.incoming[:size]
        return data


class SerialTests(unittest.TestCase):
    def test_short_ack_reads_only_available_bytes(self):
        port = FakeSerial()
        client = serial_ota.OtaClient(port)
        port.incoming.extend(reply_data(7))
        original_read = port.read

        def read_without_waiting_for_missing_bytes(size):
            self.assertLessEqual(size, port.in_waiting)
            return original_read(size)

        with mock.patch.object(port, "read", side_effect=read_without_waiting_for_missing_bytes):
            self.assertEqual(client.wait("data", 0.1, packet=7)["ack"], 1)

    def test_wait_for_first_byte_then_complete_fragmented_ack(self):
        port = FakeSerial()
        client = serial_ota.OtaClient(port)
        frame = reply_data(8)
        sizes = []
        original_read = port.read

        def fragmented_read(size):
            sizes.append(size)
            if len(sizes) == 1:
                self.assertEqual(size, 1)
                port.incoming.extend(frame[1:])
                return frame[:1]
            self.assertLessEqual(size, port.in_waiting)
            return original_read(size)

        with mock.patch.object(port, "read", side_effect=fragmented_read):
            self.assertEqual(client.wait("data", 0.1, packet=8)["ack"], 1)
        self.assertEqual(sizes, [1, len(frame) - 1])

    def test_idle_receive_and_fragmented_logs(self):
        port = FakeSerial()
        logs = []
        client = serial_ota.OtaClient(port, logs.append)
        frame = reply_query(0x55, 42)
        port.incoming.extend(b"\xff" + frame[:6])
        client.poll()
        self.assertTrue(any("RX [" in line and "FF " in line for line in logs))
        self.assertFalse(any("RX 状态:" in line for line in logs))
        port.incoming.extend(frame[6:])
        client.poll()
        self.assertTrue(any("RX 状态: 固件损坏" in line and "0000002A" in line for line in logs))
        self.assertEqual(client.pending, [])
        self.assertEqual(client.rx_bytes, len(frame) + 1)

    def test_tx_rx_errors_and_unsolicited_drain_are_logged(self):
        port = FakeSerial()
        logs = []
        client = serial_ota.OtaClient(port, logs.append)
        port.incoming.extend(reply_query(0x55))
        reply = client.query(timeout=0.1)
        self.assertEqual(reply["status"], 1)
        self.assertTrue(any("固件损坏" in line for line in logs))
        self.assertTrue(any("TX 请求" in line and protocol.query().hex(" ").upper() in line for line in logs))
        self.assertTrue(any("TX 写入 14/14B" in line for line in logs))
        self.assertTrue(any("RX 状态: 空闲" in line for line in logs))
        bad = bytearray(reply_data(0))
        bad[-1] ^= 1
        client.receive(bytes(bad))
        self.assertTrue(any("RX CRC16 错误" in line for line in logs))
        self.assertTrue(any("RX 重同步" in line for line in logs))

    def test_short_write_and_failed_drain_are_reported(self):
        port = FakeSerial()
        logs = []
        client = serial_ota.OtaClient(port, logs.append)
        with mock.patch.object(port, "write", return_value=2):
            with self.assertRaisesRegex(IOError, "写入不完整"):
                client.send(protocol.query())
        self.assertTrue(any("TX 写入 2/14B" in line for line in logs))
        port.incoming.extend(b"x")
        with mock.patch.object(port, "read", return_value=b""):
            with self.assertRaisesRegex(IOError, "读取失败"):
                client.clear()

    def test_both_input_types_send_complete_ota(self):
        ota = images.make_ota(APP, "版本1")
        for data, kind in ((APP, "app"), (ota, "ota")):
            with self.subTest(kind=kind):
                prepared, _ = images.prepare_upgrade(data, kind, "版本1")
                port = FakeSerial()
                serial_ota.OtaClient(port).upgrade(prepared, 123, timeout=0.1)
                frames = [item for item in port.writes if item[1] == 0xaa]
                self.assertEqual(b"".join(frame[16:-6] for frame in frames), ota)

    def test_complete_upgrade(self):
        port = FakeSerial()
        progress = []
        client = serial_ota.OtaClient(port, progress=progress.append)
        client.upgrade(images.make_ota(APP, "v"), 123, timeout=0.1)
        frames = [item for item in port.writes if item[1] == 0xaa]
        self.assertEqual(len(frames), 3)
        self.assertEqual([int.from_bytes(f[14:16], "big") for f in frames], [0, 1, 2])
        self.assertEqual(progress[-1], 100)

    def test_custom_payload_and_progress_details(self):
        image = images.make_ota(APP, "v")
        events = []
        port = FakeSerial()
        client = serial_ota.OtaClient(port, event=lambda kind, value: events.append((kind, value)))
        client.upgrade(image, 123, timeout=0.1, payload=127)
        frames = [item for item in port.writes if item[1] == 0xaa]
        self.assertEqual(b"".join(item[16:-6] for item in frames), image)
        self.assertTrue(all(len(item[16:-6]) <= 127 for item in frames))
        detail = [value for kind, value in events if kind == "detail"][-1]
        self.assertEqual((detail["percent"], detail["bytes"], detail["completed"]), (100, len(image), len(frames)))
        self.assertEqual(detail["tx_bytes"], sum(map(len, port.writes)))
        self.assertGreater(detail["rx_bytes"], 0)
        self.assertTrue(any(kind == "received" for kind, _ in events))

    def test_invalid_transfer_settings_do_not_touch_device(self):
        for options in ({"payload": 0}, {"payload": 1025}, {"timeout": float("nan")},
                        {"timeout": 0}, {"start_timeout": float("inf")}):
            with self.subTest(options=options):
                port = FakeSerial()
                with self.assertRaises(ValueError):
                    serial_ota.OtaClient(port).upgrade(images.make_ota(APP, "v"), 1, **options)
                self.assertEqual(port.writes, [])

    def test_failure_does_not_send_or_retry(self):
        for mode, error, count in (("start_error", RuntimeError, 0),
                                   ("bad_ack", RuntimeError, 1),
                                   ("timeout", TimeoutError, 1)):
            with self.subTest(mode=mode):
                port = FakeSerial(mode)
                client = serial_ota.OtaClient(port)
                with self.assertRaises(error):
                    client.upgrade(images.make_ota(APP, "v"), 1, timeout=0.01, start_timeout=0.01)
                self.assertEqual(sum(f[1] == 0xaa for f in port.writes), count)


class UpgradeGuiTests(unittest.TestCase):
    def test_selection_confirmation_and_validation(self):
        with tempfile.TemporaryDirectory() as folder:
            app_path = Path(folder) / "app.bin"
            ota_path = Path(folder) / "ota.bin"
            app_path.write_bytes(APP)
            ota = images.make_ota(APP, "stored")
            ota_path.write_bytes(ota)
            window = app.Application.__new__(app.Application)
            window.vars = {key: mock.Mock() for key in ("app", "ota", "input_type", "version")}
            window.vars["app"].get.return_value = str(app_path)
            window.vars["ota"].get.return_value = str(ota_path)
            window.vars["version"].get.return_value = "new"
            window.progress = {}
            window.log = mock.Mock()
            window.pending_action = None
            window.busy = False
            window.confirm_text = mock.Mock()
            window.confirm_button = mock.Mock()
            window.cancel_button = mock.Mock()
            client = mock.Mock()
            client.port.port = "COM3"
            window.client = client
            window.start = lambda callback: callback()

            with mock.patch.object(window, "settings", return_value=(123, 127, 8)):
                window.vars["input_type"].get.return_value = "app"
                window.upgrade()
                client.upgrade.assert_not_called()
                self.assertIn("new", window.confirm_text.set.call_args[0][0])
                window.confirm_action()
                self.assertEqual(client.upgrade.call_args[0], (images.make_ota(APP, "new"), 123))
                self.assertEqual(client.upgrade.call_args.kwargs, {"timeout": 8, "start_timeout": 30, "payload": 127})
                self.assertEqual(app_path.read_bytes(), APP)

                client.reset_mock()
                window.vars["input_type"].get.return_value = "ota"
                window.upgrade()
                client.upgrade.assert_not_called()
                self.assertIn("new", window.confirm_text.set.call_args[0][0])
                window.confirm_action()
                sent_image = client.upgrade.call_args[0][0]
                self.assertEqual(client.upgrade.call_args[0], (images.make_ota(APP, "new"), 123))
                self.assertEqual(sent_image[1024:], ota[1024:])
                self.assertEqual(ota_path.read_bytes(), ota)

                client.reset_mock()
                window.vars["ota"].get.return_value = str(app_path)
                window.upgrade()
                client.upgrade.assert_not_called()
                self.assertTrue(any("升级准备失败" in call[0][0] for call in window.log.call_args_list))

                window.vars["input_type"].get.return_value = "app"
                window.upgrade()
                window.cancel_action()
                window.confirm_action()
                client.upgrade.assert_not_called()

    def test_swd_requires_explicit_confirmation(self):
        window = app.Application.__new__(app.Application)
        from ui_events import UiEvents
        window.events = UiEvents()
        window.pending_action = None
        window.busy = False
        window.confirm_text = mock.Mock()
        window.confirm_button = mock.Mock()
        window.cancel_button = mock.Mock()
        window.log = mock.Mock()
        window.emit = mock.Mock()
        window.probe_ids = {"display": "probe-id"}
        window.vars = {key: mock.Mock() for key in ("factory", "probe", "pyocd")}
        window.vars["probe"].get.return_value = "display"
        window.vars["pyocd"].get.return_value = "pyocd"
        window.start = lambda callback, serial_task=True: callback()
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(app.swd, "program") as program:
            path = Path(folder) / "factory.bin"
            image = images.make_factory(BOOT, images.make_ota(APP, "v"))
            path.write_bytes(image)
            window.vars["factory"].get.return_value = str(path)
            window.flash()
            program.assert_not_called()
            self.assertIn("probe-id", window.confirm_text.set.call_args[0][0])
            window.cancel_action()
            window.confirm_action()
            program.assert_not_called()
            window.flash()
            window.confirm_action()
            program.assert_called_once()
            self.assertEqual(program.call_args.args[:3], ("pyocd", "probe-id", image))
            program.call_args.args[3]("test progress")
            self.assertIn("test progress", window.events.get_nowait()[1])


class SwdTests(unittest.TestCase):
    def test_swd_readback_and_failure(self):
        image = images.make_factory(BOOT, images.make_ota(APP, "v"))
        for mode in ("ok", "wrong_id", "wrong_size", "bad_readback", "flash_error"):
            calls = []

            def fake_run(executable, args, cwd):
                calls.append(args)
                if args[0] == "json":
                    return '{"status":0,"boards":[{"unique_id":"ABC123","info":"probe"}]}'
                if args[0] == "flash":
                    if mode == "flash_error":
                        raise RuntimeError("flash failed")
                    return ""
                command = args[-1]
                path = Path(command.split('"')[1].replace("\\\\", "\\"))
                if "0xe0042000" in command:
                    value = struct.pack("<I", 0 if mode == "wrong_id" else 0x468)
                elif "0x1fff75e0" in command:
                    value = struct.pack("<H", 64 if mode == "wrong_size" else 128)
                else:
                    value = image if mode != "bad_readback" else bytes(len(image))
                path.write_bytes(value)
                return ""

            with self.subTest(mode=mode), mock.patch.object(swd, "run", side_effect=fake_run):
                if mode == "ok":
                    swd.program("pyocd", "ABC123", image)
                    flash = next(item for item in calls if item[0] == "flash")
                    self.assertIn("--no-reset", flash)
                    self.assertIn("sector", flash)
                else:
                    with self.assertRaises(RuntimeError):
                        swd.program("pyocd", "ABC123", image)
                    if mode in ("wrong_id", "wrong_size"):
                        self.assertFalse(any(item[0] == "flash" for item in calls))

    def test_explicit_probe_required(self):
        image = images.make_factory(BOOT, images.make_ota(APP, "v"))
        with mock.patch.object(swd, "run") as run:
            with self.assertRaises(ValueError):
                swd.program("pyocd", "", image)
            run.assert_not_called()

    def test_command_error_is_reported(self):
        result = mock.Mock(returncode=1, stdout=b"device error")
        with mock.patch.object(swd.subprocess, "run", return_value=result):
            with self.assertRaisesRegex(RuntimeError, "device error"):
                swd.run("pyocd", ["flash"], ".")


if __name__ == "__main__":
    unittest.main()
