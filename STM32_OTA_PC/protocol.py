"""Wire format shared with bootloader/app/modbus.c."""
import binascii
import struct

VERSION_CHUNK = 32
ADDRESS = 3

ACK_NAMES = {1: "成功", 2: "CRC32 或最终镜像校验失败", 3: "接收超时",
             4: "协议版本错误", 5: "密钥错误", 6: "包序错误",
             7: "CRC16 错误", 8: "长度错误", 9: "Flash 写入失败"}
STATUS_NAMES = {0: "禁止升级", 1: "空闲", 2: "下载中", 3: "Flash 操作中", 0x55: "固件损坏"}


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


def frame(data):
    return data + struct.pack("<H", crc16(data))


def query():
    return frame(struct.pack(">BBHHHI", ADDRESS, 0x10, 0xe6, 3, 0, 0))


def control(operation, key):
    return frame(struct.pack(">BBHHIHIH", ADDRESS, 0x10, 0xe6, 4, 8, 1, key, operation))


def version_query(offset=0, count=VERSION_CHUNK):
    if not 0 <= offset < 1024 or not 1 <= count <= VERSION_CHUNK or offset + count > 1024:
        raise ValueError("版本查询范围无效")
    return frame(struct.pack(">BBHH", ADDRESS, 0x11, offset, count))


def data_frame(payload, total, packet, key):
    if not 1 <= len(payload) <= 1024 or not 0 <= packet <= total <= 65535:
        raise ValueError("数据帧长度或包号无效")
    head = struct.pack(">BBHHIHHH", ADDRESS, 0xaa, len(payload) + 22, 1, key,
                       len(payload), total, packet)
    return frame(head + payload + struct.pack(">I", binascii.crc32(payload) & 0xffffffff))


class RxBuffer:
    def __init__(self, trace=None):
        self.buffer = bytearray()
        self.trace = trace

    def feed(self, data):
        self.buffer.extend(data)
        replies = []
        discarded = 0
        while len(self.buffer) >= 2:
            if self.buffer[0] != ADDRESS or self.buffer[1] not in (0x10, 0x11, 0xaa):
                del self.buffer[0]
                discarded += 1
                continue
            if self.buffer[1] == 0x11:
                if len(self.buffer) < 6:
                    break
                offset = int.from_bytes(self.buffer[3:5], "big")
                count = self.buffer[5]
                if self.buffer[2] not in (0, 1) or not 1 <= count <= VERSION_CHUNK or offset + count > 1024:
                    del self.buffer[0]
                    discarded += 1
                    continue
                size = count + 8
            elif self.buffer[1] == 0xaa:
                size = 7
            else:
                if len(self.buffer) < 6:
                    break
                if self.buffer[2:4] != b"\x00\xe6":
                    del self.buffer[0]
                    discarded += 1
                    continue
                count = int.from_bytes(self.buffer[4:6], "big")
                if count not in (3, 4):
                    del self.buffer[0]
                    discarded += 1
                    continue
                size = 14 if count == 3 else 8
            if len(self.buffer) < size:
                break
            raw = bytes(self.buffer[:size])
            if crc16(raw[:-2]) != int.from_bytes(raw[-2:], "little"):
                if self.trace:
                    self.trace("RX CRC16 错误: " + raw.hex(" ").upper())
                del self.buffer[0]
                discarded += 1
                continue
            del self.buffer[:size]
            if raw[1] == 0x11:
                replies.append({"type": "version", "mode": "app" if raw[2] else "bootloader",
                                "offset": int.from_bytes(raw[3:5], "big"), "data": raw[6:-2]})
            elif raw[1] == 0xaa:
                replies.append({"type": "data", "ack": raw[2], "packet": int.from_bytes(raw[3:5], "big")})
            elif size == 14:
                replies.append({"type": "query", "status": int.from_bytes(raw[6:8], "big"),
                                "key": int.from_bytes(raw[8:12], "big")})
            else:
                replies.append({"type": "control"})
        if discarded and self.trace:
            self.trace("RX 重同步: 丢弃 %d 字节噪声或非法数据" % discarded)
        return replies
