"""Build and run firmware host tests with GCC and the STM32 HAL mocks."""
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "bootloader" / "app"
CASES = {
    "test_error_trace": ["error_trace.c"],
    "test_app_modbus_stream": ["modbus.c"],
    "test_app_ota_bridge": [],
    "test_modbus": ["modbus.c", "modbus_port.c", "error_trace.c"],
    "test_upgrade": ["upgrade.c", "sha256.c", "error_trace.c"],
    "test_upgrade_flash": ["upgrade_flash.c"],
    "test_upgrade_integration": ["upgrade.c", "modbus.c", "modbus_port.c", "sha256.c", "error_trace.c"],
}


def main():
    with tempfile.TemporaryDirectory(prefix="bootloader-tests-") as directory:
        for name, sources in CASES.items():
            executable = Path(directory) / (name + ".exe")
            subprocess.run([
                "gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                "-I" + str(ROOT / "tests" / "modbus_mock"), "-I" + str(APP),
                str(ROOT / "tests" / (name + ".c")),
                *(str(APP / source) for source in sources), "-o", str(executable),
            ], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
