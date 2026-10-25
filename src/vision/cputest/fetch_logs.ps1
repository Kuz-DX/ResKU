# Download cputest logs from jecs on Windows PowerShell.
param(
    [string]$RemoteHost = "jecs@192.168.0.100",
    [string]$LocalDirectory = "./perf_logs"
)

$ErrorActionPreference = "Stop"
New-Item -ItemType Directory -Force -Path $LocalDirectory | Out-Null
scp -r "${RemoteHost}:~/perf_logs/." $LocalDirectory

$files = @(Get-ChildItem -Path $LocalDirectory -File -Recurse)
$latest = $files | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Write-Host "Downloaded files: $($files.Count)"
Write-Host "Latest file: $(if ($latest) { $latest.FullName } else { 'none' })"
