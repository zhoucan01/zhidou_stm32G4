"""STM32G431RBTx single-slot image layout and validation."""
import hashlib
import os
import struct
from pathlib import Path

FLASH_BASE = 0x08000000
FLASH_SIZE = 128 * 1024
BOOT_SIZE = 14 * 1024
RECORD_OFFSET = 0x3800
DATA_OFFSET = 0x4000
APP_OFFSET = 0x4800
HEADER_SIZE = 2048
APP_CAPACITY = FLASH_SIZE - APP_OFFSET
MAGIC = 0x53484132


def validate_app(app, address=FLASH_BASE + APP_OFFSET, capacity=APP_CAPACITY):
    if not 8 <= len(app) <= capacity:
        raise ValueError("APP 长度不在允许范围内")
    stack, entry = struct.unpack_from("<II", app)
    if not 0x20000000 < stack <= 0x20008000 or stack % 8:
        raise ValueError("APP 初始栈指针无效")
    if not entry & 1 or not address + 8 <= (entry & ~1) < address + len(app):
        raise ValueError("APP 复位向量无效，请确认链接地址")


def make_ota(app, version):
    validate_app(app)
    encoded = version.encode("utf-8")
    if b"\0" in encoded or len(encoded) > 1024:
        raise ValueError("版本不能包含 NUL，UTF-8 长度不得超过 1024 字节")
    return encoded.ljust(1024, b"\0") + hashlib.sha256(app).digest().ljust(1024, b"\0") + app


def validate_ota(image):
    if len(image) < HEADER_SIZE + 8 or len(image) > HEADER_SIZE + APP_CAPACITY:
        raise ValueError("OTA 镜像长度无效")
    version = image[:1024]
    head, sep, tail = version.partition(b"\0")
    if sep and any(tail):
        raise ValueError("版本区零填充无效")
    try:
        text = head.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("版本不是有效的 UTF-8") from exc
    if any(image[1056:2048]):
        raise ValueError("SHA-256 区填充必须为零")
    app = image[HEADER_SIZE:]
    validate_app(app)
    if image[1024:1056] != hashlib.sha256(app).digest():
        raise ValueError("APP SHA-256 不匹配")
    return text


def prepare_upgrade(data, input_type, version):
    if input_type == "app":
        image = make_ota(data, version)
    elif input_type == "ota":
        image = data
    else:
        raise ValueError("请选择有效的升级固件类型")
    return image, validate_ota(image)


def set_version(image, version):
    """Replace only the version block in an already validated OTA image."""
    validate_ota(image)
    encoded = version.encode("utf-8")
    if not encoded or b"\0" in encoded or len(encoded) > 1024:
        raise ValueError("版本不能为空、不能包含 NUL，UTF-8 长度不得超过 1024 字节")
    result = encoded.ljust(1024, b"\0") + image[1024:]
    return result, version


def make_factory(boot, ota):
    if not 8 <= len(boot) <= BOOT_SIZE:
        raise ValueError("Bootloader 长度必须为 8 字节至 14 KiB")
    validate_app(boot, FLASH_BASE, BOOT_SIZE)
    validate_ota(ota)
    out = bytearray(b"\xff" * FLASH_SIZE)
    out[:len(boot)] = boot
    struct.pack_into("<II", out, RECORD_OFFSET, MAGIC, len(ota) - HEADER_SIZE)
    out[DATA_OFFSET:DATA_OFFSET + len(ota)] = ota
    validate_factory(bytes(out))
    return bytes(out)


def validate_factory(image):
    if len(image) != FLASH_SIZE:
        raise ValueError("工厂 BIN 必须为完整 128 KiB")
    magic, length = struct.unpack_from("<II", image, RECORD_OFFSET)
    if magic != MAGIC or not 8 <= length <= APP_CAPACITY:
        raise ValueError("工厂镜像有效记录无效")
    if image[RECORD_OFFSET + 8:DATA_OFFSET] != b"\xff" * (DATA_OFFSET - RECORD_OFFSET - 8):
        raise ValueError("记录页保留区域必须为空")
    if any(b != 0xff for b in image[APP_OFFSET + length:]):
        raise ValueError("APP 末尾后必须为擦除状态")
    validate_app(image[:BOOT_SIZE], FLASH_BASE, BOOT_SIZE)
    return validate_ota(image[DATA_OFFSET:APP_OFFSET + length])


def intel_hex(data):
    def line(addr, kind, payload):
        body = bytes([len(payload), addr >> 8, addr & 255, kind]) + payload
        return ":" + (body + bytes([-sum(body) & 255])).hex().upper() + "\n"
    result = []
    upper = None
    for offset in range(0, len(data), 16):
        addr = FLASH_BASE + offset
        if addr >> 16 != upper:
            upper = addr >> 16
            result.append(line(0, 4, struct.pack(">H", upper)))
        result.append(line(addr & 0xffff, 0, data[offset:offset + 16]))
    result.append(line(0, 1, b""))
    return "".join(result)


def save_output(path, data, inputs=()):
    target = Path(path).resolve()
    for source in inputs:
        source = Path(source).resolve()
        if target == source or (target.exists() and source.exists() and os.path.samefile(target, source)):
            raise ValueError("输出文件不能覆盖输入文件")
    target.write_bytes(data)
