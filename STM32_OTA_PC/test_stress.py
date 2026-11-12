import json
import struct
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import images
import protocol
import serial_ota
import stress_test
import app
from test_pc import FakeSerial, APP


def version_reply(offset, data, mode=1):
    return protocol.frame(struct.pack(">BBBHB", protocol.ADDRESS, 0x11, mode, offset, len(data)) + data)


class VersionSerial(FakeSerial):
    def __init__(self, version, mode=1):
        super().__init__()
        self.version = version.encode().ljust(1024, b"\0")
        self.mode = mode

    def write(self, data):
        if data[1] != 0x11:
            return super().write(data)
        self.writes.append(data)
        offset, count = struct.unpack_from(">HH", data, 2)
        self.incoming.extend(version_reply(offset, self.version[offset:offset + count], self.mode))
        return len(data)


class VersionTests(unittest.TestCase):
    def test_all_requests_use_station_three_and_station_one_is_ignored(self):
        requests = [protocol.query(), protocol.control(2, 0), protocol.version_query(),
                    protocol.data_frame(b"abc", 0, 0, 42)]
        self.assertTrue(all(frame[0] == 3 for frame in requests))
        old = protocol.frame(struct.pack(">BBBHB", 1, 0x11, 1, 0, 32) + bytes(32))
        current = version_reply(0, bytes(32))
        replies = protocol.RxBuffer().feed(old + current)
        self.assertEqual(len(replies), 1)
        self.assertEqual(replies[0]["type"], "version")

    def test_parser_fragmentation_crc_mode_and_range(self):
        frame = version_reply(32, b"v".ljust(32, b"\0"))
        rx = protocol.RxBuffer()
        self.assertEqual(rx.feed(b"noise" + frame[:-1] + bytes([frame[-1] ^ 1]) + frame[:4]), [])
        reply = rx.feed(frame[4:])[0]
        self.assertEqual((reply["mode"], reply["offset"], reply["data"][:1]), ("app", 32, b"v"))
        for invalid in (version_reply(1000, bytes(32)), version_reply(0, bytes(32), 2)):
            self.assertEqual(protocol.RxBuffer().feed(invalid), [])
        with self.assertRaises(ValueError):
            protocol.version_query(1000, 32)

    def test_query_utf8_across_blocks_and_bootloader_mode(self):
        text = "1." + "a" * 29 + "测试"
        port = VersionSerial(text)
        self.assertEqual(serial_ota.OtaClient(port).query_version(.1), {"mode": "app", "version": text})
        self.assertEqual(len(port.writes), 2)
        self.assertEqual(serial_ota.OtaClient(VersionSerial("1.0", 0)).query_version(.1)["mode"], "bootloader")

    def test_query_rejects_mode_change_and_wrong_offset(self):
        client = serial_ota.OtaClient(FakeSerial())
        replies = [{"mode": "app", "data": b"a" * 32},
                   {"mode": "bootloader", "data": bytes(32)}]
        with mock.patch.object(client, "wait", side_effect=replies):
            with self.assertRaisesRegex(ValueError, "模式变化"):
                client.query_version(.1)
        port = FakeSerial()
        client = serial_ota.OtaClient(port)
        port.incoming.extend(version_reply(32, bytes(32)) + version_reply(0, bytes(32)))
        self.assertEqual(client.wait("version", .1, offset=0)["offset"], 0)

    def test_reset_queries_until_bootloader_ready(self):
        client = serial_ota.OtaClient(FakeSerial())
        with mock.patch.object(client, "reset") as reset, \
             mock.patch.object(client, "query", side_effect=[TimeoutError(), {"status": 0}, {"status": 1}]), \
             mock.patch.object(client.stop_event, "wait", return_value=False):
            self.assertEqual(client.reset_and_query(timeout=.1)["status"], 1)
            reset.assert_called_once()

    def test_version_replacement_preserves_digest_and_app(self):
        original = images.make_ota(APP, "1.0.1")
        updated, version = images.set_version(original, "1.0.2")
        self.assertEqual(updated[1024:], original[1024:])
        self.assertEqual(images.validate_ota(updated), version)


class FakeOta:
    def __init__(self):
        self.stop_event = threading.Event()
        self.log_sink = self.command_sink = None
        self.current = "1.0.1"
        self.mode = "app"
        self.versions = []
        self.fail_first = False
        self.event = lambda *args: None

    def log(self, text):
        if self.log_sink:
            self.log_sink(text)

    def command(self, text):
        if self.command_sink:
            self.command_sink(text)

    def query_version(self, timeout):
        return {"mode": self.mode, "version": self.current}

    def query(self, timeout):
        return {"status": 0 if self.mode == "app" else 0x55, "key": 0}

    def reset_and_query(self, key, timeout):
        self.mode = "bootloader"

    def upgrade(self, image, key, timeout, payload):
        self.versions.append(images.validate_ota(image))
        self.log("TX sample HEX 03 AA")
        if self.fail_first and len(self.versions) == 1:
            raise TimeoutError("ACK missing")
        self.current = self.versions[-1]
        self.mode = "app"


class StressTests(unittest.TestCase):
    def test_erased_version_recovers_with_complete_new_transfer(self):
        class EraseFailure(FakeOta):
            erased = False

            def query_version(self, timeout):
                if self.erased:
                    raise ValueError("erased version block is not UTF-8")
                return super().query_version(timeout)

            def upgrade(self, image, key, timeout, payload):
                if not self.versions:
                    self.versions.append(images.validate_ota(image))
                    self.erased = True
                    raise TimeoutError("erase completed but start ACK was lost")
                super().upgrade(image, key, timeout, payload)
                self.erased = False

        ota = EraseFailure()
        stress = stress_test.StressTest(ota)
        ota.event = lambda kind, value: stress.stop() if kind == "stress" and value["rounds"] == 3 else None
        with tempfile.TemporaryDirectory() as folder:
            with mock.patch.object(stress.stop_event, "wait", side_effect=lambda delay: stress.stop_event.is_set()):
                stress.run(images.make_ota(APP, "1.0.1"), "1.0.1", 1, 1, folder)
            rows = [json.loads(s) for s in next(Path(folder).glob("*.jsonl")).read_text(encoding="utf-8").splitlines()]
        self.assertEqual(ota.versions, ["1.0.2", "1.0.3", "1.0.4"])
        self.assertEqual((rows[-1]["success"], rows[-1]["failed"]), (2, 1))
        self.assertTrue(rows[1]["recovery"])
        self.assertEqual(rows[1]["actual"], {"mode": "app", "version": "1.0.3"})

    def test_blank_bootloader_can_start_but_app_or_silence_cannot(self):
        for error in (ValueError("invalid version"), TimeoutError("version absent")):
            for status in (1, 2, 0x55, 0, 3, None):
                with self.subTest(error=type(error), status=status):
                    ota = FakeOta()
                    ota.query_version = mock.Mock(side_effect=error)
                    ota.query = mock.Mock(return_value={"status": status})
                    stress = stress_test.StressTest(ota)
                    if status in (1, 2, 0x55):
                        current = stress.read_current(.1)
                        self.assertEqual(current["mode"], "bootloader")
                        self.assertIsNone(current["version"])
                        self.assertEqual(stress_test.next_version("1.0.1", current["version"]), "1.0.2")
                    else:
                        if status is None:
                            ota.query.side_effect = TimeoutError("device offline")
                        with self.assertRaises((RuntimeError, TimeoutError)):
                            stress.read_current(.1)
                    self.assertEqual(ota.versions, [])

    def test_gui_stress_uses_snapshot_and_requires_confirmation(self):
        window = app.Application.__new__(app.Application)
        window.require_client = mock.Mock(return_value=mock.Mock())
        window.settings = mock.Mock(return_value=(123, 127, 1))
        window.request_confirmation = mock.Mock()
        window.start = mock.Mock(side_effect=lambda action: action())
        window.stop_stress_button = mock.Mock()
        window.disconnect_button = mock.Mock()
        window.log = mock.Mock()
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "app.bin"
            source.write_bytes(APP)
            fields = {"version": "1.0.1", "input_type": "app", "app": str(source),
                      "startup_timeout": "30", "log_dir": folder}
            window.vars = {key: mock.Mock() for key in fields}
            for key, value in fields.items():
                window.vars[key].get.return_value = value
            with mock.patch.object(app, "StressTest") as runner:
                window.start_stress()
                runner.assert_not_called()
                source.write_bytes(b"changed after preparation")
                window.request_confirmation.call_args[0][1]()
                args, kwargs = runner.return_value.run.call_args
                self.assertEqual(images.validate_ota(args[0]), "1.0.1")
                self.assertEqual(args[0][images.HEADER_SIZE:], APP)
                self.assertEqual(kwargs["payload"], 127)

    def test_gui_close_requests_graceful_stop(self):
        window = app.Application.__new__(app.Application)
        window.busy = True
        window.close_when_done = False
        window.stress = mock.Mock()
        window.stop_stress_button = mock.Mock()
        window.root = mock.Mock()
        window.log = mock.Mock()
        window.close()
        self.assertTrue(window.close_when_done)
        window.stress.stop.assert_called_once()
        window.root.destroy.assert_not_called()

    def test_increments_and_device_version(self):
        self.assertEqual(stress_test.next_version("1.0.1", "1.0.50"), "1.0.51")
        self.assertEqual(stress_test.next_version("1.0.9", "custom"), "1.0.10")
        with self.assertRaises(ValueError):
            stress_test.next_version("v1.0")

    def test_rounds_success_failure_stop_and_logs(self):
        for fail_first in (False, True):
            with self.subTest(fail_first=fail_first), tempfile.TemporaryDirectory() as folder:
                ota = FakeOta()
                ota.fail_first = fail_first
                stress = stress_test.StressTest(ota)

                def event(kind, value):
                    if kind == "stress":
                        rows = next(Path(folder).glob("*.jsonl")).read_text(encoding="utf-8").splitlines()
                        self.assertEqual(len(rows), value["rounds"])
                        if value["rounds"] == 3:
                            stress.stop()

                ota.event = event
                with mock.patch.object(stress.stop_event, "wait", side_effect=lambda delay: stress.stop_event.is_set()):
                    stress.run(images.make_ota(APP, "1.0.1"), "1.0.1", 1, 1, folder)
                self.assertEqual(ota.versions, ["1.0.2", "1.0.3", "1.0.4"])
                rows = [json.loads(s) for s in next(Path(folder).glob("*.jsonl")).read_text(encoding="utf-8").splitlines()]
                self.assertEqual(rows[-1]["success"], 3 - int(fail_first))
                self.assertEqual(rows[-1]["failed"], int(fail_first))
                commands = next(Path(folder).glob("*.commands.log")).read_text(encoding="utf-8")
                self.assertIn("第3轮成功", commands)
                self.assertNotIn("HEX", commands)
                self.assertIsNone(ota.log_sink)
                self.assertFalse(stress.running)

    def test_app_mismatch_bootloader_and_disconnect_never_succeed(self):
        ota = FakeOta()
        stress = stress_test.StressTest(ota)
        with self.assertRaisesRegex(ValueError, "APP版本不匹配"):
            stress.wait_app("1.0.9", .01)
        ota.mode = "bootloader"
        with self.assertRaises(TimeoutError):
            stress.wait_app("1.0.1", .01)
        ota.stop_event.set()
        with self.assertRaisesRegex(RuntimeError, "串口已关闭"):
            stress.wait_app("1.0.1", .01)

    def test_rotation_preserves_history_and_bounds_disk_use(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            old = directory / "older.log"
            old.write_text("preserve")
            log = stress_test.BoundedLog(directory / "test.log", 128)
            try:
                for index in range(1000):
                    log.write("%04d %s\n" % (index, "x" * 40))
                log.flush()
                files = list(directory.glob("test.log*"))
                self.assertEqual(len(files), 4)
                self.assertLessEqual(sum(p.stat().st_size for p in files), 512)
                self.assertIn("0999", (directory / "test.log").read_text())
                self.assertEqual(old.read_text(), "preserve")
            finally:
                log.close()

    def test_log_space_and_directory_limits(self):
        with tempfile.TemporaryDirectory() as folder:
            with mock.patch.object(stress_test.shutil, "disk_usage", return_value=SimpleNamespace(free=0)):
                with self.assertRaises(OSError):
                    stress_test.RunLog(folder)
            old = Path(folder) / "ota_stress_20261102_123456_123456.log"
            old.write_bytes(b"old")
            with mock.patch.object(stress_test, "DIRECTORY_LOG_BUDGET", stress_test.RUN_LOG_BUDGET):
                with self.assertRaises(OSError):
                    stress_test.RunLog(folder)
            self.assertEqual(old.read_bytes(), b"old")

    def test_log_write_error_stops_active_test(self):
        with tempfile.TemporaryDirectory() as folder:
            log = stress_test.RunLog(folder)
            try:
                with mock.patch.object(log.detail, "write", side_effect=OSError("disk full")):
                    with self.assertRaises(OSError):
                        log.write("fail")
                with self.assertRaises(OSError):
                    log.check()
            finally:
                log.close()


if __name__ == "__main__":
    unittest.main()
