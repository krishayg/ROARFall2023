@echo off
set targetPort=2000

PowerShell -Command "& { Get-Process | Where-Object { $_.Id -in (Get-NetTCPConnection -LocalPort %targetPort% -ErrorAction SilentlyContinue).OwningProcess } | ForEach-Object { Stop-Process -Id $_.Id -Force } }"