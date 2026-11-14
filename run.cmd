@echo off
call "%~dp0python.cmd" "%~dp0training\scripts\run.py" %*
exit /b %errorlevel%
