@echo off
pushd "%~dp0"
cmake --preset gcc-release && cmake --build --preset build
set "rc=%errorlevel%"
popd
exit /b %rc%
