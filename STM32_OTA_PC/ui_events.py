"""Nonblocking handoff: reliable controls, latest telemetry and bounded log previews."""
from collections import deque
from datetime import datetime
import queue
import threading

MAX_LOG_LINE = 1024
MAX_PENDING_LOG_CHARS = 256 * 1024
MAX_PENDING_LOG_LINES = 512
MAX_DISPLAY_LOG_CHARS = 256 * 1024
LATEST_EVENTS = frozenset(("progress", "detail", "received", "activity", "version", "stress"))


class UiEvents:
    def __init__(self):
        self._lock = threading.Lock()
        self._control = deque()
        self._latest = {}
        self._logs = deque()
        self._log_chars = 0
        self._dropped = 0

    def put(self, item, block=True, timeout=None):
        kind, value = item
        if kind == "log":
            value = str(value)
            if len(value) > MAX_LOG_LINE:
                value = value[:MAX_LOG_LINE - 20] + " … [界面截断]"
            value = datetime.now().strftime("%H:%M:%S.%f")[:-3] + " " + value
        with self._lock:
            if kind in LATEST_EVENTS:
                self._latest[kind] = value
            elif kind == "log":
                self._logs.append(value)
                self._log_chars += len(value)
                while (len(self._logs) > MAX_PENDING_LOG_LINES or
                       self._log_chars > MAX_PENDING_LOG_CHARS):
                    self._log_chars -= len(self._logs.popleft())
                    self._dropped += 1
            else:
                self._control.append(item)

    put_nowait = put

    def get_nowait(self):
        with self._lock:
            if self._control:
                return self._control.popleft()
            if self._latest:
                key = next(iter(self._latest))
                return key, self._latest.pop(key)
            if self._dropped:
                count, self._dropped = self._dropped, 0
                return "log", "[UI] 省略 %d 条显示日志；压测磁盘日志按分卷预算保留。" % count
            if self._logs:
                text = self._logs.popleft()
                self._log_chars -= len(text)
                return "log", text
        raise queue.Empty

    def clear_logs(self):
        with self._lock:
            self._logs.clear()
            self._log_chars = self._dropped = 0

    def discard_updates(self, *keys):
        with self._lock:
            for key in keys:
                self._latest.pop(key, None)

    def qsize(self):
        with self._lock:
            return len(self._control) + len(self._latest) + len(self._logs) + bool(self._dropped)

    def empty(self):
        return self.qsize() == 0
