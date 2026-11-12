import queue
import threading
import time
import tempfile
from pathlib import Path
import unittest
from unittest import mock

import app
from ui_events import UiEvents, MAX_PENDING_LOG_CHARS


class EventTests(unittest.TestCase):
    def test_preview_truncation_preserves_complete_disk_log(self):
        import serial_ota
        from stress_test import RunLog
        from test_pc import FakeSerial
        events = UiEvents()
        text = "TX HEX " + "AB " * 5000
        with tempfile.TemporaryDirectory() as folder:
            log = RunLog(folder)
            client = serial_ota.OtaClient(FakeSerial(), lambda value: events.put(("log", value)))
            client.log_sink = log.write
            client.log(text)
            log.close()
            detail = next(path for path in Path(folder).glob("*.log") if ".commands." not in path.name)
            self.assertIn(text, detail.read_text(encoding="utf-8"))
        _, preview = events.get_nowait()
        self.assertLess(len(preview), 1100)

    def test_complete_transfer_does_not_need_gui_to_drain_events(self):
        import images
        import serial_ota
        from test_pc import APP, FakeSerial
        events = UiEvents()
        port = FakeSerial()
        client = serial_ota.OtaClient(port, lambda text: events.put(("log", text)),
                                      lambda value: events.put(("progress", value)),
                                      lambda kind, value: events.put((kind, value)))
        result = []

        def transfer():
            try:
                client.upgrade(images.make_ota(APP, "1.0.1"), 42, timeout=.1, payload=7)
                result.append("complete")
            except Exception as exc:
                result.append(exc)

        worker = threading.Thread(target=transfer, daemon=True)
        worker.start()
        worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result, ["complete"])
        self.assertLess(events.qsize(), 525)
        frames = [data for data in port.writes if data[1] == 0xaa]
        self.assertGreater(len(frames), 300)
        latest = {}
        while not events.empty():
            kind, value = events.get_nowait()
            if kind != "log":
                latest[kind] = value
        self.assertEqual(latest["progress"], 100)

    def test_producer_never_waits_for_gui_and_controls_survive_flood(self):
        events = UiEvents()

        def produce():
            for index in range(20000):
                events.put(("progress", index))
                events.put(("detail", {"packet": index}))
                events.put(("log", "AB " * 1024))
            events.put(("error", "failure"))
            events.put(("done", None))

        producer = threading.Thread(target=produce, daemon=True)
        producer.start()
        producer.join(5)
        self.assertFalse(producer.is_alive(), "GUI not pumping must never stall serial worker")
        self.assertLess(events.qsize(), 520)
        self.assertEqual(events.get_nowait(), ("error", "failure"))
        self.assertEqual(events.get_nowait(), ("done", None))
        self.assertEqual(events.get_nowait(), ("progress", 19999))
        self.assertEqual(events.get_nowait(), ("detail", {"packet": 19999}))
        logs = []
        while not events.empty():
            kind, text = events.get_nowait()
            self.assertEqual(kind, "log")
            logs.append(text)
        self.assertLessEqual(sum(map(len, logs)), MAX_PENDING_LOG_CHARS + 200)
        self.assertTrue(any("界面截断" in line for line in logs))

    def test_clear_only_removes_logs_and_stale_transfer_updates(self):
        events = UiEvents()
        events.put(("log", "old"))
        events.put(("progress", 100))
        events.put(("version", {"version": "1.0.1"}))
        events.put(("done", None))
        events.clear_logs()
        events.discard_updates("progress")
        self.assertEqual(events.get_nowait(), ("done", None))
        self.assertEqual(events.get_nowait()[0], "version")
        with self.assertRaises(queue.Empty):
            events.get_nowait()


def window_fixture():
    window = app.Application.__new__(app.Application)
    window.events = UiEvents()
    window.serial_lock = threading.RLock()
    window.serial_active = threading.Event()
    window.idle_stop = threading.Event()
    window.closing = threading.Event()
    window.busy = window.serial_busy = False
    window.pending_action = None
    window.stress = None
    window.close_when_done = False
    window.stop_stress_button = mock.Mock()
    window.buttons = []
    window.disconnect_button = mock.Mock()
    window.root = mock.Mock()
    window.render_logs = mock.Mock()
    window.client = mock.Mock()
    window.port = mock.Mock()
    return window


class WorkerTests(unittest.TestCase):
    def test_connection_callbacks_keep_data_queue_instead_of_tk_window(self):
        import inspect
        from test_pc import FakeSerial
        window = window_fixture()
        fields = {"port": "COM3", "baud": "1000000", "parity": "N", "stopbits": "1"}
        window.vars = {key: mock.Mock() for key in fields}
        for key, value in fields.items():
            window.vars[key].get.return_value = value
        window.port = None
        window.start = lambda action, serial_task=True: action()
        with mock.patch.object(app.serial, "Serial", return_value=FakeSerial()):
            window.connect()
        kind, (_, client) = window.events.get_nowait()
        self.assertEqual(kind, "connected")
        for callback in (client._log, client.progress, client.event):
            captures = inspect.getclosurevars(callback).nonlocals
            self.assertEqual(set(captures), {"events"})
            self.assertIs(captures["events"], window.events)

    def test_close_aborts_regular_io_then_exits_on_worker_done(self):
        window = window_fixture()
        window.busy = window.serial_busy = True
        client = window.client
        window.close()
        self.assertTrue(window.close_when_done)
        client.stop_event.set.assert_called_once()
        window.root.destroy.assert_not_called()
        window.events.put(("done", None))
        window.process_events()
        self.assertTrue(window.closing.is_set())
        window.root.destroy.assert_called_once()

    def test_cancelled_client_does_not_send_next_packet(self):
        import protocol
        import serial_ota
        from test_pc import FakeSerial
        port = FakeSerial()
        client = serial_ota.OtaClient(port)
        client.stop_event.set()
        with self.assertRaises(RuntimeError):
            client.send(protocol.query())
        self.assertEqual(port.writes, [])

    def test_query_and_version_snapshot_settings_on_gui_thread(self):
        gui_thread = threading.get_ident()
        for method in ("query", "query_version", "reset"):
            window = window_fixture()
            ran = threading.Event()

            def settings():
                self.assertEqual(threading.get_ident(), gui_thread)
                return 1, 1024, .1

            def operation(*args):
                self.assertNotEqual(threading.get_ident(), gui_thread)
                ran.set()
                return {"status": 1, "key": 0}

            window.settings = settings
            window.client.query.side_effect = operation
            window.client.query_version.side_effect = operation
            window.client.reset.side_effect = operation
            getattr(window, method)()
            self.assertTrue(ran.wait(1))

    def test_slow_idle_serial_read_never_runs_in_gui_pump(self):
        window = window_fixture()
        entered, release = threading.Event(), threading.Event()
        gui_thread = threading.get_ident()

        def slow_poll():
            self.assertNotEqual(threading.get_ident(), gui_thread)
            entered.set()
            release.wait(2)

        window.client.poll.side_effect = slow_poll
        window.start_idle_reader(window.client)
        try:
            self.assertTrue(entered.wait(1))
            start = time.perf_counter()
            window.process_events()
            self.assertLess(time.perf_counter() - start, .1)
            self.assertEqual(window.client.poll.call_count, 1)
        finally:
            window.idle_stop.set()
            release.set()

    def test_serial_job_waits_for_idle_reader_without_blocking_gui(self):
        window = window_fixture()
        entered, release, job = threading.Event(), threading.Event(), threading.Event()

        def slow_poll():
            entered.set()
            release.wait(2)

        window.client.poll.side_effect = slow_poll
        window.start_idle_reader(window.client)
        try:
            self.assertTrue(entered.wait(1))
            start = time.perf_counter()
            window.start(job.set)
            self.assertLess(time.perf_counter() - start, .1)
            self.assertFalse(job.is_set())
            release.set()
            self.assertTrue(job.wait(1))
        finally:
            window.idle_stop.set()
            release.set()

    def test_disconnect_returns_before_driver_close_finishes(self):
        window = window_fixture()
        entered, release = threading.Event(), threading.Event()

        def slow_close():
            entered.set()
            release.wait(2)

        client = window.client
        client.close.side_effect = slow_close
        try:
            start = time.perf_counter()
            window.disconnect()
            self.assertLess(time.perf_counter() - start, .1)
            self.assertTrue(entered.wait(1))
            self.assertIsNone(window.client)
            client.stop_event.set.assert_called_once()
        finally:
            release.set()

    def test_stress_stop_never_writes_logs_from_gui(self):
        from stress_test import StressTest
        ota = mock.Mock()
        stress = StressTest(ota)
        stress.stop()
        self.assertTrue(stress.stop_event.is_set())
        ota.log.assert_not_called()
        ota.command.assert_not_called()


class RealWidgetTests(unittest.TestCase):
    def setUp(self):
        import tkinter as tk
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        self.root.withdraw()
        with mock.patch.object(app.Application, "refresh_ports"):
            self.window = app.Application(self.root)

    def tearDown(self):
        if hasattr(self, "window"):
            worker = getattr(self.window, "worker_thread", None)
            if worker is not None:
                worker.join(1)
            self.window.busy = False
            self.window.close()
            for name in ("idle_thread", "close_thread"):
                worker = getattr(self.window, name, None)
                if worker is not None:
                    worker.join(1)
            self.window = self.root = None
            # Tk objects must be collected on their creating thread, including
            # cycles retained by mocked callbacks in these integration tests.
            import gc
            gc.collect()

    def pump_until(self, condition):
        deadline = time.monotonic() + 2
        while not condition() and time.monotonic() < deadline:
            self.root.update()
            time.sleep(.001)
        self.assertTrue(condition())

    def test_real_query_buttons_do_not_read_tk_variables_in_worker(self):
        gui_thread = threading.get_ident()
        client = mock.Mock()
        client.query.return_value = {"status": 1, "key": 0}
        self.window.client = client
        for label, method in (("查询 OTA 状态", "query"), ("查询版本号", "query_version")):
            called = threading.Event()

            def operation(*args):
                self.assertNotEqual(threading.get_ident(), gui_thread)
                called.set()
                return {"status": 1, "key": 0}

            getattr(client, method).side_effect = operation
            button = next(widget for widget in self.window.buttons if widget.cget("text") == label)
            button.invoke()
            self.assertTrue(called.wait(1))
            self.pump_until(lambda: not self.window.busy)

    def test_real_text_batches_and_bounds_long_hex_logs(self):
        from ui_events import MAX_DISPLAY_LOG_CHARS
        for _ in range(5000):
            self.window.log("AB " * 1024)
        with mock.patch.object(self.window.log_box, "insert", wraps=self.window.log_box.insert) as insert:
            self.pump_until(self.window.events.empty)
            self.assertLess(insert.call_count, 100)
        count = self.window.log_box.count("1.0", "end-1c", "chars")[0]
        self.assertLessEqual(count, MAX_DISPLAY_LOG_CHARS)
        self.assertEqual(self.window.log_box.cget("wrap"), "none")


if __name__ == "__main__":
    unittest.main()
