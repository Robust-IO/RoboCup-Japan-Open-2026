"""Read-only Windows/WSL connectivity evidence; never changes network settings."""
import json
from pathlib import Path
import subprocess

print('WSL_BOOT', Path('/proc/sys/kernel/random/boot_id').read_text().strip(), flush=True)
for command in (['ss','-lntp'], ['ss','-nt'], ['hostname','-I']):
    print(subprocess.run(command, capture_output=True, text=True).stdout, flush=True)
ps = r'''
$ErrorActionPreference = 'Continue'
Write-Output 'UNITY_PROCESSES'
Get-Process -Name Unity,Handyman -ErrorAction SilentlyContinue | Select-Object Id,ProcessName,StartTime,Path | ConvertTo-Json -Compress
Write-Output 'ACTIVE_IP_ADDRESSES'
Get-NetIPAddress -AddressFamily IPv4 | Where-Object {$_.AddressState -eq 'Preferred'} | Select-Object InterfaceAlias,IPAddress | ConvertTo-Json -Compress
Write-Output 'UNITY_LOG_CONNECTION_LINES'
$log = Join-Path $env:LOCALAPPDATA 'Unity/Editor/Editor.log'
if (Test-Path $log) {Get-Content $log -Tail 1200 | Select-String -Pattern 'rosbridge|50001|9090|WebSocket|SocketException|connection|connect to|ros bridge' | Select-Object -Last 35 | ForEach-Object {$_.Line}}
Write-Output 'UNITY_PROJECT_DIRECTORIES'
Get-ChildItem -Path 'D:/handyman' -Directory -ErrorAction SilentlyContinue | Select-Object -ExpandProperty FullName
'''
r = subprocess.run(['/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe',
    '-NoProfile','-NonInteractive','-Command',ps], capture_output=True, timeout=30)
print(r.stdout.decode('utf-8', errors='replace'), flush=True)
print(r.stderr.decode('utf-8', errors='replace')[-1200:], flush=True)
