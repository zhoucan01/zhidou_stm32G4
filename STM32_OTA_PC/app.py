"""STM32G431 single-partition OTA and factory programming GUI."""
import queue
import threading
import time
import math
import tkinter as tk
from tkinter import filedialog, ttk
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

import images
import protocol
import swd
from serial_ota import OtaClient
from stress_test import StressTest, version_parts
from ui_events import UiEvents, MAX_DISPLAY_LOG_CHARS


class Application:
    def __init__(self, root):
        self.root = root
        root.title("STM32G431 OTA 上位机 · 自定义Modbus")
        root.geometry("1060x980")
        root.minsize(980, 960)
        self.events = UiEvents()
        self.serial_lock = threading.RLock()
        self.serial_active = threading.Event()
        self.idle_stop = threading.Event()
        self.closing = threading.Event()
        self.ports_scanning = False
        self.stress = None
        self.close_when_done = False
        self.busy = False
        self.serial_busy = False
        self.pending_action = None
        self.port = None
        self.client = None
        self.probe_ids = {}
        self.buttons = []
        self.vars = {key: tk.StringVar(value=value) for key, value in {
            "port": "", "baud": "1000000", "parity": "N", "stopbits": "1",
            "boot": "", "app": "", "ota": "", "input_type": "ota",
            "version": "1.0.0", "factory": "", "pyocd": "pyocd", "probe": "",
            "key": str(int(time.time()) & 0xffffffff), "payload": "1024", "timeout": "8",
            "startup_timeout": "30", "log_dir": str(Path.cwd() / "ota_logs")}.items()}
        frame = ttk.Frame(root, padding=10)
        frame.pack(fill="both", expand=True)

        panel = ttk.LabelFrame(frame, text="1. 串口连接", padding=8)
        panel.pack(fill="x")
        ttk.Label(panel, text="串口").pack(side="left")
        self.port_box = ttk.Combobox(panel, textvariable=self.vars["port"], width=16)
        self.port_box.pack(side="left", padx=4)
        self.button(panel, "刷新", self.refresh_ports)
        ttk.Label(panel, text="波特率").pack(side="left", padx=(12, 2))
        ttk.Combobox(panel, textvariable=self.vars["baud"], width=11,
                     values=[9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000]).pack(side="left")
        ttk.Label(panel, text="校验").pack(side="left", padx=(12, 2))
        ttk.Combobox(panel, textvariable=self.vars["parity"], values=["N", "E", "O"], width=3, state="readonly").pack(side="left")
        ttk.Label(panel, text="停止位").pack(side="left", padx=(8, 2))
        ttk.Combobox(panel, textvariable=self.vars["stopbits"], values=["1", "2"], width=3, state="readonly").pack(side="left")
        self.button(panel, "连接", self.connect)
        self.disconnect_button = self.button(panel, "断开", self.disconnect)

        panel = ttk.LabelFrame(frame, text="2. OTA 固件（上位机填写版本区）", padding=8)
        panel.pack(fill="x", pady=8)
        self.path_row(panel, "OTA BIN", "ota")
        self.path_row(panel, "原始 APP BIN", "app")
        row = ttk.Frame(panel)
        row.pack(fill="x", pady=3)
        ttk.Label(row, text="固件类型").pack(side="left")
        for label, value in (("已打包 OTA", "ota"), ("原始 APP（自动打包）", "app")):
            radio = ttk.Radiobutton(row, text=label, variable=self.vars["input_type"], value=value)
            radio.pack(side="left", padx=6)
            self.buttons.append(radio)
        ttk.Label(row, text="版本号").pack(side="left", padx=(12, 2))
        ttk.Entry(row, textvariable=self.vars["version"], width=20).pack(side="left")
        self.button(row, "生成 OTA", self.create_ota)
        ttk.Label(panel, text="APP 链接到 0x08004800，最大 110 KiB；OTA 摘要保留并验证，版本区按填写值更新。",
                  foreground="#9b4510").pack(anchor="w", pady=4)
        row = ttk.Frame(panel)
        row.pack(fill="x")
        self.button(row, "合并 Bootloader + APP 烧录文件…", self.open_factory_pack)

        panel = ttk.LabelFrame(frame, text="3. 协议设置 · 当前 C 代码：普通字段大端 / CRC16 小端 / CRC32 大端", padding=8)
        panel.pack(fill="x")
        row = ttk.Frame(panel)
        row.pack(fill="x", pady=3)
        ttk.Label(row, text="普通字段：大端    CRC16：小端    CRC32：大端    首包号：0").pack(side="left")
        ttk.Label(row, text="每包数据字节").pack(side="left", padx=(16, 4))
        ttk.Entry(row, textvariable=self.vars["payload"], width=7).pack(side="left")
        row = ttk.Frame(panel)
        row.pack(fill="x", pady=3)
        ttk.Label(row, text="会话 Key（十进制 / 0x 十六进制）").pack(side="left")
        ttk.Entry(row, textvariable=self.vars["key"], width=18).pack(side="left", padx=4)
        self.button(row, "当前时间戳", lambda: self.vars["key"].set(str(int(time.time()) & 0xffffffff)))
        ttk.Label(row, text="应答超时（秒）").pack(side="left", padx=(12, 4))
        ttk.Entry(row, textvariable=self.vars["timeout"], width=7).pack(side="left")

        panel = ttk.LabelFrame(frame, text="4. 操作与测试", padding=8)
        panel.pack(fill="x", pady=8)
        row = ttk.Frame(panel)
        row.pack(fill="x")
        for title, command in (("查询 OTA 状态", self.query), ("开始 OTA", self.upgrade),
                               ("查询版本号", self.query_version),
                               ("重启并查询 OTA", self.reset_query_device),
                               ("查询/无操作测试", self.safe_tests), ("离线协议自检", self.offline_tests),
                               ("手动回滚（单分区不支持）", self.rollback), ("重启设备", self.reset_device)):
            self.button(row, title, command)
        row = ttk.Frame(panel)
        row.pack(fill="x", pady=5)
        ttk.Label(row, text="APP 启动超时（秒）").pack(side="left")
        ttk.Entry(row, textvariable=self.vars["startup_timeout"], width=6).pack(side="left")
        ttk.Label(row, text="压测日志目录").pack(side="left", padx=(8, 2))
        ttk.Entry(row, textvariable=self.vars["log_dir"], width=30).pack(side="left")
        self.button(row, "选择目录", self.choose_log_directory)
        self.button(row, "开始压力测试", self.start_stress)
        self.stop_stress_button = ttk.Button(row, text="停止（完成本轮）", command=self.stop_stress, state="disabled")
        self.stop_stress_button.pack(side="left", padx=3)
        self.version_label = ttk.Label(panel, text="设备版本：尚未查询")
        self.version_label.pack(anchor="w")
        self.stress_label = ttk.Label(panel, text="压力测试：未开始")
        self.stress_label.pack(anchor="w")
        self.progress = ttk.Progressbar(panel, maximum=100)
        self.progress.pack(fill="x", pady=7)
        self.progress_label = ttk.Label(panel, text="0% | 已确认 0 / 0 包")
        self.progress_label.pack(anchor="w")
        self.progress_detail = ttk.Label(panel, text="已确认 0 / 0 B | 耗时 0.0s | 平均速度 0.00 KiB/s")
        self.progress_detail.pack(anchor="w")
        self.activity_label = ttk.Label(panel, text="当前阶段：待开始")
        self.activity_label.pack(anchor="w", pady=(3, 0))
        self.rx_preview = ttk.Label(panel, text="最近接收：尚未收到数据", wraplength=940)
        self.rx_preview.pack(anchor="w", pady=(3, 0))
        confirmation = ttk.Frame(panel)
        confirmation.pack(fill="x", pady=(5, 0))
        self.confirm_text = tk.StringVar(value="没有待确认的操作")
        ttk.Label(confirmation, textvariable=self.confirm_text, wraplength=740).pack(side="left", fill="x", expand=True)
        self.confirm_button = ttk.Button(confirmation, text="确认执行", command=self.confirm_action, state="disabled")
        self.confirm_button.pack(side="left", padx=3)
        self.cancel_button = ttk.Button(confirmation, text="取消", command=self.cancel_action, state="disabled")
        self.cancel_button.pack(side="left", padx=3)

        panel = ttk.LabelFrame(frame, text="5. 最近收发日志（HEX 摘要 + 协议字段解释）", padding=6)
        panel.pack(fill="both", expand=True)
        self.log_box = tk.Text(panel, height=12, font=("Consolas", 10), state="disabled", wrap="none")
        scrollbar = ttk.Scrollbar(panel, command=self.log_box.yview)
        self.log_box.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        horizontal = ttk.Scrollbar(panel, orient="horizontal", command=self.log_box.xview)
        horizontal.pack(side="bottom", fill="x")
        self.log_box.configure(xscrollcommand=horizontal.set)
        self.log_box.pack(side="left", fill="both", expand=True)
        row = ttk.Frame(frame)
        row.pack(anchor="e", pady=4)
        self.button(row, "清空日志", self.clear_log)
        self.button(row, "保存当前显示日志", self.save_log)
        self.log("单分区升级会擦除唯一 APP，不支持回滚；设备默认串口为 1000000、8N1。")
        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.pump_id = root.after(20, self.process_events)

    def open_factory_pack(self):
        if getattr(self, "factory_window", None) is not None and self.factory_window.winfo_exists():
            self.factory_window.lift()
            return
        window = self.factory_window = tk.Toplevel(self.root)
        window.title("合并 Bootloader + APP · BIN / HEX / SWD")
        window.transient(self.root)
        panel = ttk.Frame(window, padding=12)
        panel.pack(fill="both", expand=True)
        self.path_row(panel, "Bootloader BIN", "boot")
        self.path_row(panel, "原始 APP BIN", "app")
        row = ttk.Frame(panel)
        row.pack(fill="x", pady=4)
        ttk.Label(row, text="APP 版本号").pack(side="left")
        ttk.Entry(row, textvariable=self.vars["version"], width=24).pack(side="left", padx=8)
        self.button(row, "生成并导出 BIN / HEX", self.create_factory)
        ttk.Label(panel, text="Bootloader：0x08000000，最多 14 KiB；APP：0x08004800，最多 110 KiB。\n"
                  "自动生成 SHA-256、长度和有效记录；导出完整 128 KiB 工厂镜像。\n"
                  "BIN 烧录起点 0x08000000；HEX 已包含地址。", wraplength=700).pack(anchor="w", pady=10)
        flash = ttk.LabelFrame(panel, text="SWD 烧录完整工厂 BIN", padding=8)
        flash.pack(fill="x")
        self.path_row(flash, "工厂 BIN", "factory")
        row = ttk.Frame(flash)
        row.pack(fill="x", pady=4)
        ttk.Label(row, text="pyOCD").pack(side="left")
        ttk.Entry(row, textvariable=self.vars["pyocd"], width=18).pack(side="left", padx=4)
        self.button(row, "枚举下载器", self.refresh_probes)
        self.probe_box = ttk.Combobox(row, textvariable=self.vars["probe"], state="readonly", width=34)
        if hasattr(self, "probe_box") and self.probe_box.winfo_exists():
            self.probe_box["values"] = list(self.probe_ids)
        self.probe_box.pack(side="left", padx=4)
        self.button(row, "确认并烧录", self.flash)
        ttk.Label(panel, text="SWD 操作需在主窗口点击“确认执行”。").pack(anchor="w", pady=5)
        if self.busy:
            for button in self.buttons:
                button.configure(state="disabled")

    def save_log(self):
        path = filedialog.asksaveasfilename(defaultextension=".log", initialfile=datetime.now().strftime("ota_%Y%m%d_%H%M%S.log"),
                                          filetypes=[("日志", "*.log"), ("文本", "*.txt")])
        if path:
            try:
                Path(path).write_text(self.log_box.get("1.0", "end-1c"), encoding="utf-8")
                self.log("当前显示日志已保存：" + path)
            except OSError as exc:
                self.log("保存日志失败：" + str(exc))

    def settings(self):
        key_text = self.vars["key"].get().strip()
        key = int(key_text, 16 if key_text.lower().startswith("0x") else 10)
        payload = int(self.vars["payload"].get())
        timeout = float(self.vars["timeout"].get())
        if not 0 <= key <= 0xffffffff or not 1 <= payload <= 1024:
            raise ValueError("Key 必须为 uint32，每包数据必须为 1～1024 字节")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("应答超时必须为有限正数")
        return key, payload, timeout

    def safe_tests(self):
        try:
            client = self.require_client()
            key, _, timeout = self.settings()
            events = self.events
            def execute():
                client.query(timeout)
                client.clear()
                client.send(protocol.control(0, key))
                client.wait("control", timeout)
                events.put(("log", "查询/无操作测试通过"))
            self.start(execute)
        except (ValueError, RuntimeError) as exc:
            self.log("测试失败：" + str(exc))

    def query_with_timeout(self, timeout, client=None):
        result = (client or self.require_client()).query(timeout)
        self.emit("状态：%s (0x%04X), key=0x%08X" %
                  (protocol.STATUS_NAMES.get(result["status"], "未知"), result["status"], result["key"]))

    def offline_tests(self):
        try:
            key, _, _ = self.settings()
            frames = [protocol.query(), protocol.control(1, key), protocol.data_frame(b"abc", 0, 0, key)]
            if [len(item) for item in frames] != [14, 20, 25]:
                raise ValueError("协议帧长度错误")
            for item in frames:
                if protocol.crc16(item[:-2]) != int.from_bytes(item[-2:], "little"):
                    raise ValueError("CRC16 自检失败")
            rx = protocol.RxBuffer()
            reply = protocol.frame(bytes([protocol.ADDRESS, 0xaa, 1, 0, 0]))
            if rx.feed(reply[:2]) or rx.feed(reply[2:]) != [{"type": "data", "ack": 1, "packet": 0}]:
                raise ValueError("应答分段拼接失败")
            self.log("离线协议自检通过：查询/控制/数据帧长度、CRC16、接收分段拼接")
        except ValueError as exc:
            self.log("离线协议自检失败：" + str(exc))

    def rollback(self):
        self.log("当前为单分区 Bootloader，不支持回滚；请重新升级所需版本。")

    def reset_device(self):
        self.request_confirmation("向当前串口设备发送重启命令？", self.reset)

    def query_version(self):
        try:
            client, timeout = self.require_client(), self.settings()[2]
        except (RuntimeError, ValueError) as exc:
            self.log(str(exc)); return
        self.start(lambda: client.query_version(timeout))

    def reset_query_device(self):
        try:
            client = self.require_client()
            timeout = self.settings()[2]
        except (ValueError, RuntimeError) as exc:
            self.log("操作失败：" + str(exc))
            return
        self.request_confirmation("重启当前设备并立即查询 Bootloader？",
                                  lambda: self.start(lambda: client.reset_and_query(timeout=timeout)))

    def choose_log_directory(self):
        directory = filedialog.askdirectory()
        if directory:
            self.vars["log_dir"].set(directory)

    def start_stress(self):
        try:
            client = self.require_client()
            seed = self.vars["version"].get().strip()
            version_parts(seed)
            kind = self.vars["input_type"].get()
            source = self.vars[kind].get()
            image, _ = images.prepare_upgrade(Path(source).read_bytes(), kind, seed)
            _, payload, timeout = self.settings()
            if (len(image) + payload - 1) // payload > 65536:
                raise ValueError("分包数量超出协议限制，请增大每包数据字节数")
            startup_timeout = float(self.vars["startup_timeout"].get())
            if not math.isfinite(startup_timeout) or startup_timeout <= 0:
                raise ValueError("APP 启动超时必须为有限正数")
            directory = self.vars["log_dir"].get().strip()
            if not directory:
                raise ValueError("请选择压测日志目录")
        except (KeyError, ValueError, OSError, RuntimeError) as exc:
            self.log("压力测试准备失败：" + str(exc))
            return

        def execute():
            self.stress = StressTest(client)
            runner = self.stress
            self.stop_stress_button.configure(state="normal")
            self.start(lambda: runner.run(image, seed, timeout, startup_timeout,
                                          directory, source, payload=payload))
            self.disconnect_button.configure(state="normal")

        self.request_confirmation("持续压力测试：%s；起始版本 %s。每轮擦除唯一 APP，版本逐轮递增，"
                                  "仅 APP 模式且版本匹配才成功。停止或关闭窗口会等待本轮结束。" %
                                  (source, seed), execute)

    def stop_stress(self):
        if self.stress is not None:
            try:
                self.stress.stop()
            except OSError as exc:
                self.log("停止请求已登记，日志写入失败：" + str(exc))
            self.stop_stress_button.configure(state="disabled")

    def button(self, parent, label, command):
        button = ttk.Button(parent, text=label, command=command)
        button.pack(side="left", padx=3)
        self.buttons.append(button)
        return button

    def path_row(self, parent, label, name):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=5, pady=3)
        ttk.Label(row, text=label, width=16).pack(side="left")
        ttk.Entry(row, textvariable=self.vars[name]).pack(side="left", fill="x", expand=True)
        self.button(row, "选择", lambda: self.choose(name))

    def choose(self, name):
        path = filedialog.askopenfilename(filetypes=[("BIN 文件", "*.bin"), ("所有文件", "*.*")])
        if path:
            self.vars[name].set(path)
            if name in ("ota", "app"):
                self.vars["input_type"].set(name)
                try:
                    image, version = images.prepare_upgrade(Path(path).read_bytes(), name, self.vars["version"].get())
                    self.log("已导入 %s：%s，OTA 长度 %d B，原版本 %s" % (name.upper(), path, len(image), version))
                except (ValueError, OSError) as exc:
                    self.log("导入失败：" + str(exc))

    def log(self, text):
        self.events.put(("log", text))

    def render_logs(self, lines):
        if not lines:
            return
        follow = self.log_box.yview()[1] >= 0.98
        self.log_box.configure(state="normal")
        self.log_box.insert("end", "\n".join(lines) + "\n")
        rows = int(self.log_box.index("end-1c").split(".")[0])
        if rows > 5001:
            self.log_box.delete("1.0", "%d.0" % (rows - 5000))
        count = self.log_box.count("1.0", "end-1c", "chars")
        chars = count[0] if count else 0
        if chars > MAX_DISPLAY_LOG_CHARS:
            self.log_box.delete("1.0", "1.0 + %d chars lineend + 1 chars" % (chars - MAX_DISPLAY_LOG_CHARS))
        if follow:
            self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def clear_log(self):
        self.events.clear_logs()
        self.log_box.configure(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.configure(state="disabled")

    def request_confirmation(self, description, action):
        if self.pending_action:
            self.cancel_action()
        self.pending_action = action
        self.confirm_text.set(description)
        self.confirm_button.configure(state="normal")
        self.cancel_button.configure(state="normal")
        self.log("等待界面确认: " + description)

    def cancel_action(self):
        if self.pending_action:
            self.log("已取消待确认操作")
        self.pending_action = None
        self.confirm_text.set("没有待确认的操作")
        self.confirm_button.configure(state="disabled")
        self.cancel_button.configure(state="disabled")

    def confirm_action(self):
        if self.busy or self.pending_action is None:
            return
        action = self.pending_action
        self.pending_action = None
        self.confirm_text.set("没有待确认的操作")
        self.confirm_button.configure(state="disabled")
        self.cancel_button.configure(state="disabled")
        action()

    def process_events(self):
        lines, log_chars = [], 0
        deadline = time.perf_counter() + 0.008
        try:
            for _ in range(200):
                if time.perf_counter() >= deadline:
                    break
                kind, value = self.events.get_nowait()
                if kind == "log":
                    lines.append(value)
                    log_chars += len(value)
                    if log_chars >= 16384:
                        break
                elif kind == "progress":
                    self.progress["value"] = value
                elif kind == "activity":
                    self.activity_label.configure(text="当前阶段：" + value)
                elif kind == "version":
                    self.version_label.configure(text="设备版本：%s | %s" % (value["version"], value["mode"]))
                elif kind == "stress":
                    self.stress_label.configure(text="压测：%d 轮 | 成功 %d | 失败 %d | 目标 %s | 本轮 %s" %
                                                (value["rounds"], value["success"], value["failed"],
                                                 value["version"], value["result"]))
                elif kind == "received":
                    self.rx_preview.configure(text="最近接收 [%d 字节]：%s" % (len(value), value[:48].hex(" ").upper()))
                elif kind == "detail":
                    self.progress_label.configure(text="%d%% | 已确认 %d / %d 包 | 当前协议包号 %d" %
                                                  (value["percent"], value["completed"], value["total"], value["packet"]))
                    self.progress_detail.configure(text="已确认 %d / %d B | 耗时 %.1fs | 平均速度 %.2f KiB/s | 最近包 %.2fms | TX %d B / RX %d B" %
                        (value["bytes"], value["total_bytes"], value["elapsed"], value["rate"] / 1024,
                         value["rtt_ms"], value["tx_bytes"], value["rx_bytes"]))
                elif kind == "ports":
                    self.ports_scanning = False
                    self.port_box["values"] = value
                    if value and not self.vars["port"].get():
                        self.vars["port"].set(value[0])
                elif kind == "ports_error":
                    self.ports_scanning = False
                    self.log("串口枚举失败：" + value)
                elif kind == "connected":
                    self.port, self.client = value
                    self.start_idle_reader(self.client)
                    self.log("串口已连接：" + self.port.port)
                elif kind == "idle_error":
                    client, error = value
                    if self.client is client:
                        self.log("串口接收失败：" + error)
                        self.disconnect()
                elif kind == "probes":
                    self.probe_ids = {("%s | %s" % (uid, info)): uid for uid, info in value}
                    if hasattr(self, "probe_box") and self.probe_box.winfo_exists():
                        self.probe_box["values"] = list(self.probe_ids)
                    self.vars["probe"].set("")
                    self.log("发现 %d 个下载器，请手动选择" % len(value))
                elif kind == "error":
                    self.log("失败: " + value)
                    self.activity_label.configure(text="当前阶段：操作失败，详情见日志")
                elif kind == "done":
                    self.busy = False
                    self.serial_busy = False
                    self.serial_active.clear()
                    self.stress = None
                    self.stop_stress_button.configure(state="disabled")
                    if self.close_when_done:
                        self.close()
                        return
                    for button in self.buttons:
                        if button.winfo_exists():
                            button.configure(state="normal")
        except queue.Empty:
            pass
        self.render_logs(lines)
        if not self.closing.is_set():
            self.pump_id = self.root.after(20 if not self.events.empty() else 50, self.process_events)

    def start(self, action, serial_task=True):
        if self.busy:
            return
        if self.pending_action:
            self.cancel_action()
        self.busy = True
        self.serial_busy = serial_task
        if serial_task:
            self.serial_active.set()
        else:
            self.serial_active.clear()
        self.events.discard_updates("progress", "detail", "received", "activity")
        for button in self.buttons:
            if button.winfo_exists():
                button.configure(state="disabled")
        if serial_task and self.client is not None:
            self.disconnect_button.configure(state="normal")
        events, serial_lock = self.events, self.serial_lock
        def worker():
            try:
                if serial_task:
                    with serial_lock:
                        action()
                else:
                    action()
            except Exception as exc:
                events.put(("error", str(exc)))
            finally:
                events.put(("done", None))
        self.worker_thread = threading.Thread(target=worker, daemon=True)
        self.worker_thread.start()

    def emit(self, text):
        self.events.put(("log", text))

    def refresh_ports(self):
        if self.ports_scanning:
            return
        self.ports_scanning = True
        events = self.events
        def scan():
            try:
                names = [p.device for p in list_ports.comports()]
                events.put(("ports", names))
            except Exception as exc:
                events.put(("ports_error", str(exc)))
        threading.Thread(target=scan, daemon=True).start()

    def connect(self):
        try:
            if self.port:
                raise ValueError("请先断开已有串口")
            port_name = self.vars["port"].get()
            baud = int(self.vars["baud"].get())
            parity = self.vars["parity"].get()
            stopbits = int(self.vars["stopbits"].get())
            if not port_name or baud <= 0:
                raise ValueError("请选择串口并填写正数波特率")
        except (ValueError, serial.SerialException) as exc:
            self.log(str(exc))
            return
        events = self.events
        def open_port():
            port = serial.Serial(port_name, baud, parity=parity, stopbits=stopbits,
                                 timeout=0.02, write_timeout=3)
            client = OtaClient(port, lambda text: events.put(("log", text)),
                               lambda value: events.put(("progress", value)),
                               lambda kind, value: events.put((kind, value)))
            events.put(("connected", (port, client)))
        self.start(open_port, serial_task=False)

    def start_idle_reader(self, client):
        self.idle_stop.set()
        self.idle_stop = stop = threading.Event()
        active, lock, closing, events = self.serial_active, self.serial_lock, self.closing, self.events
        def reader():
            while not stop.wait(0.02) and not closing.is_set():
                if active.is_set() or not lock.acquire(blocking=False):
                    continue
                try:
                    if not active.is_set() and not stop.is_set():
                        client.poll()
                except Exception as exc:
                    events.put(("idle_error", (client, str(exc))))
                    return
                finally:
                    lock.release()
        self.idle_thread = threading.Thread(target=reader, daemon=True)
        self.idle_thread.start()

    def disconnect(self):
        if self.pending_action:
            self.cancel_action()
        self.idle_stop.set()
        port, client = self.port, self.client
        self.port = self.client = None
        if port is None:
            return
        if client is not None:
            client.stop_event.set()
        events = self.events
        def close_port():
            try:
                if client is not None:
                    client.close()
                else:
                    port.close()
                events.put(("log", "串口已断开"))
            except Exception as exc:
                events.put(("log", "关闭串口失败：" + str(exc)))
        self.close_thread = threading.Thread(target=close_port, daemon=True)
        self.close_thread.start()

    def require_client(self):
        if self.client is None:
            raise RuntimeError("请先连接串口")
        return self.client

    def query(self):
        try:
            client, timeout = self.require_client(), self.settings()[2]
        except (RuntimeError, ValueError) as exc:
            self.log(str(exc)); return
        self.start(lambda: client.query(timeout))

    def reset(self):
        try:
            client, timeout = self.require_client(), self.settings()[2]
        except (RuntimeError, ValueError) as exc:
            self.log(str(exc)); return
        events = self.events
        def execute():
            client.reset(timeout)
            events.put(("log", "设备已确认重启请求"))
        self.start(execute)

    def create_ota(self):
        try:
            app_path = self.vars["app"].get()
            data = images.make_ota(Path(app_path).read_bytes(), self.vars["version"].get())
            path = filedialog.asksaveasfilename(defaultextension=".bin", filetypes=[("BIN", "*.bin")])
            if path:
                images.save_output(path, data, [app_path])
                self.vars["ota"].set(path)
                self.vars["input_type"].set("ota")
                self.log("OTA 镜像已生成: " + path)
        except (OSError, ValueError) as exc:
            self.log("生成失败: %s" % exc)

    def create_factory(self):
        try:
            boot = self.vars["boot"].get()
            app = self.vars["app"].get()
            ota = images.make_ota(Path(app).read_bytes(), self.vars["version"].get())
            data = images.make_factory(Path(boot).read_bytes(), ota)
            path = filedialog.asksaveasfilename(defaultextension=".bin", filetypes=[("BIN", "*.bin"), ("HEX", "*.hex")])
            if path:
                if Path(path).suffix.lower() == ".hex":
                    images.save_output(path, images.intel_hex(data).encode("ascii"), [boot, app])
                elif Path(path).suffix.lower() == ".bin":
                    images.save_output(path, data, [boot, app])
                    self.vars["factory"].set(path)
                else:
                    raise ValueError("输出文件必须为 .bin 或 .hex")
                self.log("工厂镜像已生成: " + path)
        except (OSError, ValueError) as exc:
            self.log("生成失败: %s" % exc)

    def upgrade(self):
        if self.pending_action:
            self.cancel_action()
        try:
            client = self.require_client()
            input_type = self.vars["input_type"].get()
            if input_type not in ("app", "ota"):
                raise ValueError("请选择有效的升级固件类型")
            source = self.vars[input_type].get()
            if not source:
                raise ValueError("请先选择%s BIN" % ("原始 APP" if input_type == "app" else "OTA 镜像"))
            image, version = images.prepare_upgrade(Path(source).read_bytes(), input_type, self.vars["version"].get())
            options = self.settings()
            if input_type == "ota":
                version = self.vars["version"].get()
                encoded = version.encode("utf-8")
                if b"\0" in encoded or len(encoded) > 1024:
                    raise ValueError("版本不能包含 NUL，UTF-8 长度不得超过 1024 字节")
                image = encoded.ljust(1024, b"\0") + image[1024:]
                images.validate_ota(image)
        except (OSError, ValueError, RuntimeError) as exc:
            self.log("升级准备失败: %s" % exc)
            return
        source_type = "原始 APP（已在内存打包）" if input_type == "app" else "已打包 OTA 镜像"

        def execute():
            self.progress["value"] = 0
            self.log("升级固件: %s，版本: %s，OTA 长度: %d 字节" % (source_type, version, len(image)))
            key, payload, timeout = options
            self.start(lambda: client.upgrade(image, key, timeout=timeout, start_timeout=max(30, timeout), payload=payload))

        self.request_confirmation("串口: %s；类型: %s；文件: %s；版本: %s。"
                                  "升级将擦除唯一 APP，失败或断电需重新升级，不能回滚。" %
                                  (client.port.port, source_type, source, version), execute)

    def refresh_probes(self):
        executable = self.vars["pyocd"].get()
        events = self.events
        self.start(lambda: events.put(("probes", swd.probes(executable))), serial_task=False)

    def flash(self):
        if self.pending_action:
            self.cancel_action()
        try:
            image = Path(self.vars["factory"].get()).read_bytes()
            version = images.validate_factory(image)
            uid = self.probe_ids.get(self.vars["probe"].get())
            if not uid:
                raise ValueError("请枚举并明确选择下载器")
            executable = self.vars["pyocd"].get()
            if not executable:
                raise ValueError("请指定 pyOCD 可执行文件")
        except (OSError, ValueError) as exc:
            self.log("烧录准备失败: %s" % exc)
            return
        events = self.events
        log = lambda text: events.put(("log", text))
        self.request_confirmation("请核实目标为 STM32G431RBTx；下载器: %s；文件: %s；版本: %s。"
                                  "将擦写完整 128 KiB Flash，无法回滚。" %
                                  (uid, self.vars["factory"].get(), version),
                                  lambda: self.start(lambda: swd.program(executable, uid, image, log),
                                                     serial_task=False))

    def close(self):
        if self.busy:
            if self.stress is not None:
                self.close_when_done = True
                self.stop_stress()
                self.log("请求关闭：完成当前压测轮后自动退出。")
                return
            if self.serial_busy:
                self.close_when_done = True
                self.disconnect()
                self.log("请求关闭：已中止串口操作，正在释放连接。")
                return
            self.log("操作进行中，请等待当前操作完成后再关闭。")
            return
        self.closing.set()
        if getattr(self, "pump_id", None) is not None:
            self.root.after_cancel(self.pump_id)
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    window = Application(tk.Tk())
    window.root.mainloop()
