# 串口监视器（不需要安装任何软件）
# 用法：双击同目录的 serial_mon.bat，或者在本目录执行：
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\serial_mon.ps1

$ErrorActionPreference = 'Stop'

$ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
if ($ports.Count -eq 0) {
    Write-Host "没有找到串口。"
    Write-Host "检查：USB 线是否插在板子的 USB 口（不是 ST-Link 那个口）、固件是否已经烧进去。"
    Read-Host "按回车退出"
    exit 1
}

Write-Host "可用串口："
$ports | ForEach-Object { Write-Host "  $_" }

$port = Read-Host "输入串口名（例如 COM5）"
if ([string]::IsNullOrWhiteSpace($port)) { exit 1 }

$sp = New-Object System.IO.Ports.SerialPort $port.Trim(), 115200, 'None', 8, 'One'
$sp.NewLine = "`r`n"
$sp.ReadTimeout = 2000
$sp.Open()

Write-Host "已打开 $port，Ctrl+C 停止。" -ForegroundColor Green
Write-Host "正常应该每秒刷 10 行左右，形如："
Write-Host "  sw=97,158 cls=97,158 on=1 rc=0,0,0,0 rpm=0,0,0,0"
Write-Host ""

while ($true) {
    try {
        $line = $sp.ReadLine()
        Write-Host $line
    } catch [System.TimeoutException] {
        # 2 秒没数据：提醒一次，不退出
        Write-Host "（2 秒没收到数据：检查固件是否烧录、USB 线是否是数据线）" -ForegroundColor Yellow
    }
}
