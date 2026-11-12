"""Blocking serial operations; call from a worker thread."""
import time
import math
import threading
import protocol
from images import validate_ota


def validate_timeout(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("应答超时必须为有限正数")
    return value


class OtaClient:
    def __init__(self, port, log=lambda message: None, progress=lambda value: None,
                 event=lambda kind, value: None):
        self.port = port
        self._log = log
        self.log_sink = None
        self.command_sink = None
        self.stop_event = threading.Event()
        self.progress = progress
        self.event = event
        self.rx = protocol.RxBuffer(self.log)
        self.pending = []
        self.tx_bytes = 0
        self.rx_bytes = 0

    def log(self, text):
        if self.log_sink:
            self.log_sink(text)
        self._log(text)

    def command(self, text):
        if self.command_sink:
            self.command_sink(text)

    def close(self):
        self.stop_event.set()
        self.port.close()

    def receive(self, data):
        if not data:
            return []
        self.rx_bytes += len(data)
        self.event("received", data)
        self.log("RX [%dB, 累计 %dB] HEX %s" % (len(data), self.rx_bytes, data.hex(" ").upper()))
        replies = self.rx.feed(data)
        for reply in replies:
            if reply["type"] == "query":
                self.log("RX 状态: %s (0x%04X), key=0x%08X" %
                         (protocol.STATUS_NAMES.get(reply["status"], "未知"), reply["status"], reply["key"]))
            elif reply["type"] == "data":
                self.log("RX 数据应答: 包号=%d, ACK=0x%02X %s" %
                         (reply["packet"], reply["ack"], protocol.ACK_NAMES.get(reply["ack"], "未知")))
            elif reply["type"] == "control":
                self.log("RX 控制确认")
        return replies

    def poll(self):
        count = self.port.in_waiting
        if count:
            self.receive(self.port.read(min(count, 4096)))

    def clear(self):
        drained = 0
        while self.port.in_waiting:
            count = min(self.port.in_waiting, 4096)
            data = self.port.read(count)
            if not data:
                raise IOError("串口报告有数据但读取失败")
            self.receive(data)
            drained += len(data)
            if drained >= 65536 and self.port.in_waiting:
                raise RuntimeError("串口持续接收数据，无法清理旧应答，请检查总线")
        if self.pending or self.rx.buffer:
            self.log("清理旧应答: %d 帧, 未完成数据 %dB" % (len(self.pending), len(self.rx.buffer)))
        self.rx = protocol.RxBuffer(self.log)
        self.pending = []

    def send(self, data):
        if self.stop_event.is_set():
            raise RuntimeError("串口已关闭，任务停止")
        self.log("TX 请求 [%dB] HEX %s" % (len(data), data.hex(" ").upper()))
        written = self.port.write(data)
        self.tx_bytes += written
        self.log("TX 写入 %d/%dB, 累计 %dB" % (written, len(data), self.tx_bytes))
        if written != len(data):
            raise IOError("串口写入不完整")
        self.port.flush()

    def wait(self, kind, timeout, packet=None, start=False, offset=None):
        deadline = time.monotonic() + validate_timeout(timeout)
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                raise RuntimeError("串口已关闭，任务停止")
            if not self.pending:
                # Block only for the first byte; consume queued bytes without
                # waiting for a short ACK to fill a 256-byte read.
                count = min(self.port.in_waiting, 4096)
                self.pending.extend(self.receive(self.port.read(count or 1)))
                continue
            reply = self.pending.pop(0)
            if start and reply["type"] == "query":
                raise RuntimeError("开始升级未确认，收到状态 0x%02X（可能是周期上报或擦除失败），请重新查询" % reply["status"])
            if reply["type"] != kind:
                self.log("忽略非预期应答: %s" % reply)
                continue
            if packet is not None and reply.get("packet") != packet:
                self.log("忽略其他包的应答: %s" % reply)
                continue
            if offset is not None and reply.get("offset") != offset:
                self.log("忽略其他偏移的版本应答: %s" % reply)
                continue
            return reply
        raise TimeoutError("等待 %s 应答超时；未自动重传" % kind)

    def query(self, timeout=4):
        self.command("发送查询 OTA 状态")
        self.clear()
        self.send(protocol.query())
        result = self.wait("query", timeout)
        self.command("收到 OTA 状态 0x%04X" % result["status"])
        return result

    def reset(self, timeout=8):
        self.command("发送重启")
        self.clear()
        self.send(protocol.control(2, 0))
        result = self.wait("control", timeout)
        self.command("收到重启确认")
        return result

    def query_version(self, timeout=4):
        deadline = time.monotonic() + validate_timeout(timeout)
        data = bytearray()
        mode = None
        self.command("发送版本及运行模式查询")
        for offset in range(0, 1024, protocol.VERSION_CHUNK):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("版本查询超时")
            self.clear()
            self.send(protocol.version_query(offset))
            reply = self.wait("version", remaining, offset=offset)
            if len(reply["data"]) != protocol.VERSION_CHUNK:
                raise ValueError("版本分块长度不匹配")
            if mode is not None and mode != reply["mode"]:
                raise ValueError("版本查询过程中设备运行模式变化")
            mode = reply["mode"]
            data.extend(reply["data"])
            if 0 in reply["data"]:
                break
        raw = bytes(data).split(b"\0", 1)[0]
        try:
            version = raw.decode("utf-8")
        except UnicodeDecodeError:
            raise ValueError("设备版本区不是有效 UTF-8") from None
        if not version or not version.isprintable():
            raise ValueError("设备版本区无有效版本号")
        result = {"mode": mode, "version": version}
        self.log("设备版本：%s，运行模式：%s" % (version, mode))
        self.command("收到回复：模式=%s，版本=%s" % (mode, version))
        self.event("version", result)
        return result

    def reset_and_query(self, key=0, timeout=8):
        timeout = validate_timeout(timeout)
        self.reset(timeout)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.05):
                raise RuntimeError("串口已关闭，任务停止")
            try:
                state = self.query(min(0.2, max(0.001, deadline - time.monotonic())))
            except TimeoutError:
                continue
            if state["status"] in (1, 2, 0x55):
                self.event("activity", "Bootloader 已就绪，可以开始 OTA")
                return state
        raise TimeoutError("复位后未查询到允许 OTA 的 Bootloader 状态")

    def upgrade(self, image, key, timeout=8, start_timeout=30, payload=1024):
        validate_ota(image)
        if not 0 <= key <= 0xffffffff:
            raise ValueError("会话密钥必须为 uint32")
        if not isinstance(payload, int) or not 1 <= payload <= 1024:
            raise ValueError("每包数据必须为 1～1024 字节")
        if (len(image) + payload - 1) // payload > 65536:
            raise ValueError("分包数量超出协议限制，请增大每包数据字节数")
        if not all(math.isfinite(value) and value > 0 for value in (timeout, start_timeout)):
            raise ValueError("应答超时必须为有限正数")
        started = time.monotonic()
        tx_start, rx_start = self.tx_bytes, self.rx_bytes
        self.event("activity", "查询设备状态")
        result = self.query(timeout)
        self.log("设备状态: %s (0x%02X)" %
                 (protocol.STATUS_NAMES.get(result["status"], "未知"), result["status"]))
        self.clear()
        self.log("开始升级: key=0x%08X，等待擦除确认" % key)
        self.event("activity", "等待擦除确认")
        self.send(protocol.control(1, key))
        self.command("开始升级：等待擦除确认")
        self.wait("control", start_timeout, start=True)
        chunks = [image[i:i + payload] for i in range(0, len(image), payload)]
        confirmed = 0
        for index, chunk in enumerate(chunks):
            self.event("activity", "发送数据包 %d/%d，等待确认" % (index + 1, len(chunks)))
            packet_started = time.monotonic()
            self.log("发送数据包: %d/%d，载荷 %dB" % (index + 1, len(chunks), len(chunk)))
            self.send(protocol.data_frame(chunk, len(chunks) - 1, index, key))
            reply = self.wait("data", timeout, packet=index)
            if reply["ack"] != 1:
                raise RuntimeError("包 %d 被拒绝: %s (0x%02X)" %
                                   (index, protocol.ACK_NAMES.get(reply["ack"], "未知错误"), reply["ack"]))
            self.progress((index + 1) * 100 // len(chunks))
            confirmed += len(chunk)
            elapsed = max(time.monotonic() - started, 1e-9)
            self.log("每包进度：%d/%d 包 | 协议包号 %d | %d%% | 已确认 %d/%d B | 本包 %d B | 确认耗时 %.2f ms" %
                     (index + 1, len(chunks), index, (index + 1) * 100 // len(chunks),
                      confirmed, len(image), len(chunk), (time.monotonic() - packet_started) * 1000))
            self.event("detail", {"percent": (index + 1) * 100 // len(chunks),
                                 "completed": index + 1, "total": len(chunks), "packet": index,
                                 "bytes": confirmed, "total_bytes": len(image), "elapsed": elapsed,
                                 "rate": confirmed / elapsed,
                                 "rtt_ms": (time.monotonic() - packet_started) * 1000,
                                 "tx_bytes": self.tx_bytes - tx_start, "rx_bytes": self.rx_bytes - rx_start})
        self.event("activity", "升级完成，末包已确认；请检查 APP 运行")
        self.command("全部 %d 个固件包已确认；需等待 APP 启动并核对版本" % len(chunks))
        self.log("最终包已确认：镜像校验及有效记录完成；不代表 APP 已成功运行")
