param(
    [string]$Config='config\yy_v2_fixed50k.yaml',
    [int]$Steps=50000,
    [string]$Prefix='yy_v2_fixed',
    [string]$RunRoot='experiments\yy_v2\runs',
    [string]$Stage=''
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
foreach ($seed in @(42,43,44)) {
    $arguments=@('--config',"$root\$Config",'train','--name',"${Prefix}_s$seed",'--steps',$Steps,'--seed',$seed,'--run-root',"$root\$RunRoot")
    if ($Stage -ne '') { $arguments += @('--stage',$Stage) }
    & "$root\run.cmd" @arguments
    if ($LASTEXITCODE -ne 0) { throw "Training failed for seed $seed; remaining runs were not started" }
}
