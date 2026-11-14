$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
Set-Location -LiteralPath $root
New-Item -ItemType Directory -Force -Path "$root\.tools", "$root\.cache", "$root\.tmp", "$root\reports" | Out-Null
$env:TEMP="$root\.tmp"
$env:TMP=$env:TEMP
$env:UV_CACHE_DIR="$root\.cache\uv"
$env:UV_PYTHON_INSTALL_DIR="$root\.runtime"
$env:UV_PYTHON_BIN_DIR="$root\.tools\python"
$uv="$root\.tools\uv\uv.exe"
if (-not (Test-Path -LiteralPath $uv)) {
    Invoke-WebRequest -Uri 'https://github.com/astral-sh/uv/releases/latest/download/uv-x86_64-pc-windows-msvc.zip' -OutFile "$root\.tools\uv.zip"
    Expand-Archive -LiteralPath "$root\.tools\uv.zip" -DestinationPath "$root\.tools\uv" -Force
}
if (-not (Test-Path -LiteralPath "$root\.venv\Scripts\python.exe")) {
    & $uv python install 3.10
    if ($LASTEXITCODE -ne 0) { throw 'Python installation failed' }
    & $uv venv --python 3.10 "$root\.venv"
    if ($LASTEXITCODE -ne 0) { throw 'venv creation failed' }
}
$python="$root\.venv\Scripts\python.exe"
& $uv pip install --python $python 'torch==2.8.0' --index-url 'https://download.pytorch.org/whl/cpu'
if ($LASTEXITCODE -ne 0) { throw 'CPU torch installation failed' }
if (Test-Path -LiteralPath "$root\config\requirements-win-lock.txt") {
    & $uv pip install --python $python -r "$root\config\requirements-win-lock.txt"
} else {
    & $uv pip install --python $python -r "$root\config\requirements.txt"
}
if ($LASTEXITCODE -ne 0) { throw 'Dependencies failed' }
& $uv pip check --python $python
if ($LASTEXITCODE -ne 0) { throw 'Dependency check failed' }
& $python "$root\training\scripts\run.py" doctor
if ($LASTEXITCODE -ne 0) { throw 'Doctor failed' }
Write-Host 'READY. Use run.cmd from this directory.'
