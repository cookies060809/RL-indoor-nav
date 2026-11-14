param([string]$Stage='mixed',[int]$Steps=300000,[string]$Prefix='formal',[string]$Config='config\windows_recommended32.yaml')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
foreach ($seed in @(42,43,44)) {
    & "$root\run.cmd" --config "$root\$Config" train --name "${Prefix}_${Stage}_s$seed" --stage $Stage --steps $Steps --seed $seed
    if ($LASTEXITCODE -ne 0) { throw "Training failed for seed $seed; remaining runs were not started" }
}
