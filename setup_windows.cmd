@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0training\scripts\setup_windows.ps1"
exit /b %errorlevel%
