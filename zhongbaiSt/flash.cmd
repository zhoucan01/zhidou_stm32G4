@echo off
setlocal
pushd "%~dp0"

set "selection=%~1"
if not defined selection set "selection=power-product"
set "pyocd_options=-f 100k -M under-reset"

if /I "%selection%"=="product" (
  cmake --preset gcc-release && cmake --build --preset product-modbus
  if errorlevel 1 goto :failed
  set "image=%CD%\build\gcc-release\F1_Product_Modbus_v2_M1.hex"
  set "pyocd_options=-f 100k -M under-reset"
  goto :flash
)

if /I "%selection%"=="power-product" (
  cmake --preset gcc-release && cmake --build --preset power-product-modbus
  if errorlevel 1 goto :failed
  set "image=%CD%\build\gcc-release\F1_POWER_V004_Product_Modbus_v2_M1.hex"
  set "pyocd_options=-f 100k -M under-reset"
  goto :flash
)

if exist "%selection%" (
  for %%I in ("%selection%") do set "image=%%~fI"
  goto :flash
)

echo Usage: flash.cmd [product^|power-product^|path-to-hex]
echo Default: power-product = F1_POWER_V004_Product_Modbus_v2_M1.hex
goto :failed

:flash
echo Flash image: "%image%"
for /f "tokens=*" %%H in ('powershell -NoProfile -Command "(Get-FileHash -LiteralPath '%image%' -Algorithm SHA256).Hash"') do echo SHA256: %%H
py -m pyocd flash -t stm32g431rbtx %pyocd_options% "%image%"
if errorlevel 1 goto :failed
popd
exit /b 0

:failed
set "rc=%errorlevel%"
if "%rc%"=="0" set "rc=1"
popd
exit /b %rc%
