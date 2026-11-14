@echo off
setlocal
cd /d "%~dp0"
if not exist "%~dp0.tmp" mkdir "%~dp0.tmp"
set TEMP=%~dp0.tmp
set TMP=%~dp0.tmp
set MPLCONFIGDIR=%~dp0.cache\matplotlib
set NUMBA_CACHE_DIR=%~dp0.cache\numba
set OMP_NUM_THREADS=2
set MKL_NUM_THREADS=2
set PYTHONPATH=%~dp0training\envs;%PYTHONPATH%
set PROJECT_PYTHON=%~dp0.venv\Scripts\python.exe
if not exist "%PROJECT_PYTHON%" set PROJECT_PYTHON=%~dp0..\.venv\Scripts\python.exe
if not exist "%PROJECT_PYTHON%" (
  echo Missing Python environment. Run setup_windows.cmd first.
  exit /b 1
)
"%PROJECT_PYTHON%" %*
exit /b %errorlevel%
