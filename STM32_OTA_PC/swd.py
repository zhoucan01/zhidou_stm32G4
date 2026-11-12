"""Explicit probe selection, device checks and readback for pyOCD."""
import json
import subprocess
import tempfile
from pathlib import Path

from images import FLASH_BASE, FLASH_SIZE, validate_factory

TARGET = "stm32g431rbtx"


def run(executable, args, cwd):
    try:
        result = subprocess.run([executable] + args, cwd=str(cwd), stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=180, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError("无法执行 pyOCD: %s" % exc) from exc
    output = result.stdout.decode("utf-8", errors="replace")
    if result.returncode:
        raise RuntimeError("pyOCD 失败 (退出码 %d): %s" % (result.returncode, output))
    return output


def probes(executable):
    with tempfile.TemporaryDirectory(prefix="stm32_ota_") as folder:
        output = run(executable, ["json", "--no-config", "--probes"], folder)
    try:
        info = json.loads(output)
        if info["status"] != 0:
            raise RuntimeError("pyOCD 枚举失败: %s" % info.get("error", "未知错误"))
        return [(item["unique_id"], item["info"]) for item in info["boards"]]
    except (ValueError, KeyError, TypeError) as exc:
        raise RuntimeError("pyOCD 下载器列表格式错误") from exc


def program(executable, uid, image, log=lambda message: None):
    validate_factory(image)
    if not uid:
        raise ValueError("必须明确选择下载器")
    available = probes(executable)
    if sum(1 for item, _ in available if uid in item) != 1 or uid not in [item for item, _ in available]:
        raise ValueError("所选下载器不存在或 ID 不唯一")
    with tempfile.TemporaryDirectory(prefix="stm32_ota_") as folder:
        directory = Path(folder)
        base = ["--no-config", "-W", "-t", TARGET, "-u", uid, "-M", "halt"]

        def read_memory(address, length, name):
            output = directory / name
            escaped = str(output).replace("\\", "\\\\").replace('"', '\\"')
            command = 'savemem 0x%08x %d "%s"' % (address, length, escaped)
            run(executable, ["cmd"] + base + ["-c", command], directory)
            if not output.is_file():
                raise RuntimeError("pyOCD 未生成读回文件")
            data = output.read_bytes()
            if len(data) != length:
                raise RuntimeError("pyOCD 读回长度不正确")
            return data

        device_id = int.from_bytes(read_memory(0xe0042000, 4, "id.bin"), "little") & 0xfff
        flash_kb = int.from_bytes(read_memory(0x1fff75e0, 2, "size.bin"), "little")
        if device_id != 0x468 or flash_kb != 128:
            raise RuntimeError("目标芯片不匹配: DEV_ID=0x%03X, Flash=%d KiB" % (device_id, flash_kb))
        log("芯片检查通过: DEV_ID=0x%03X, Flash=%d KiB；开始擦写" % (device_id, flash_kb))
        source = directory / "factory.bin"
        source.write_bytes(image)
        run(executable, ["flash"] + base + ["--erase", "sector", "--no-reset",
                                           "--base-address", hex(FLASH_BASE), str(source)], directory)
        actual = read_memory(FLASH_BASE, FLASH_SIZE, "readback.bin")
        if actual != image:
            raise RuntimeError("SWD 写后读回校验失败，请勿启动设备")
        log("SWD 完整 128 KiB 写后读回一致，可手动复位设备")
