@echo off
python "%~dp0product_modbus.py" %*
exit /b %errorlevel%
