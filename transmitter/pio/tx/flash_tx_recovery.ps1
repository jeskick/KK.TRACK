# TX 救砖：手动进下载模式后全片烧录（OTA 中断后 app 损坏、只亮电源灯时用）
# ESP32-C3 下载模式：GPIO9=低(按住按键)，GPIO8=高，GPIO2=高
# 若进不了下载：临时飞线 GPIO8→3.3V、GPIO2→3.3V，按住 GPIO9，再 RESET
# 用法:
#   1. 关掉所有串口监视器
#   2. 按住 TX 板上的 GPIO9 按键不松
#   3. 运行本脚本，看到 "Connecting..." 时短按一下 RESET（或拔插 USB 一次仍按住 GPIO9）
#   4. 出现 "Chip is ESP32-C3" 后再松开 GPIO9
#
#   .\flash_tx_recovery.ps1 COM46
param(
    [string]$Port = "COM46"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Build = Join-Path $Root ".pio\build\esp32-c3"
$Pio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pio.exe"
$Py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
$Esptool = Join-Path $env:USERPROFILE ".platformio\packages\tool-esptoolpy\esptool.py"
$Ninja = Join-Path $env:USERPROFILE ".platformio\packages\tool-ninja\ninja.exe"

Write-Host ""
Write-Host "=== TX 恢复烧录（手动下载模式）===" -ForegroundColor Cyan
Write-Host "1) 按住 GPIO9 按键"
Write-Host "2) 脚本出现 Connecting... 时按 RESET（或拔插 USB，仍按住 GPIO9）"
Write-Host "3) 看到 Chip is ESP32-C3 后松开 GPIO9"
Write-Host ""

foreach ($f in @($Pio, $Py, $Esptool, $Ninja)) {
    if (-not (Test-Path $f)) { throw "Missing: $f" }
}

function Ensure-IdfFlashAssets {
    param([string]$BuildDir)
    $part = Join-Path $BuildDir "partition_table\partition-table.bin"
    $ota = Join-Path $BuildDir "ota_data_initial.bin"
    if ((Test-Path $part) -and (Test-Path $ota)) { return }
    Push-Location $BuildDir
    try {
        & $Ninja "partition_table/partition-table.bin" "ota_data_initial.bin" | Out-Host
        if ($LASTEXITCODE -ne 0) { throw "ninja partition/ota targets failed" }
    } finally {
        Pop-Location
    }
}

function Parse-FlashProjectArgs {
    param([string]$BuildDir)
    $argsFile = Join-Path $BuildDir "flash_project_args"
    if (-not (Test-Path $argsFile)) { throw "Missing $argsFile — run pio run first" }
    $flashMode = @("--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "4MB")
    $images = @()
    $lines = Get-Content $argsFile | Where-Object { $_ -and -not $_.StartsWith("--") }
    foreach ($line in $lines) {
        $tok = $line -split '\s+', 2
        if ($tok.Count -lt 2) { continue }
        $off = $tok[0]
        $rel = $tok[1] -replace '/', '\'
        $bin = Join-Path $BuildDir $rel
        if (-not (Test-Path $bin)) {
            $leaf = Split-Path $rel -Leaf
            if ($leaf -match "kk_.*\.bin") { $bin = Join-Path $BuildDir "firmware.bin" }
            elseif ($leaf -eq "bootloader.bin") { $bin = Join-Path $BuildDir "bootloader.bin" }
        }
        if (-not (Test-Path $bin)) { throw "Missing flash image: $bin ($line)" }
        $images += @{ Offset = $off; Path = $bin }
    }
    return @{ FlashMode = $flashMode; Images = $images }
}

Push-Location $Root
try {
    $env:PYTHONUTF8 = "1"
    & $Pio run | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "pio run failed" }

    Ensure-IdfFlashAssets -BuildDir $Build
    $plan = Parse-FlashProjectArgs -BuildDir $Build

    Write-Host "先探测芯片（请保持 GPIO9 按住 + RESET）..." -ForegroundColor Yellow
    $chipArgs = @(
        $Esptool, "--chip", "esp32c3", "--port", $Port, "--baud", "115200",
        "--connect-attempts", "30", "--before", "no_reset", "--after", "no_reset",
        "chip_id"
    )
    & $Py @chipArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Host ""
        Write-Host "chip_id 失败 — 常见原因:" -ForegroundColor Red
        Write-Host "  - 未进下载模式（GPIO9 按住 + RESET）"
        Write-Host "  - COM 口选错（TX=COM46 RX=COM47）"
        Write-Host "  - USB 线/CH340 接触不良"
        throw "chip_id failed"
    }

    $writeArgs = @(
        $Esptool, "--chip", "esp32c3", "--port", $Port, "--baud", "115200",
        "--connect-attempts", "10",
        "--before", "no_reset", "--after", "hard_reset",
        "write_flash", "--erase-all"
    ) + $plan.FlashMode
    foreach ($img in $plan.Images) {
        $writeArgs += @($img.Offset, $img.Path)
    }

    Write-Host "chip_id OK，开始全片擦除 + 烧录 ..." -ForegroundColor Green
    & $Py @writeArgs
    if ($LASTEXITCODE -ne 0) { throw "flash failed" }
    Write-Host "Done. 断电重上，应看到蓝/绿灯和 IMU 日志。" -ForegroundColor Green
}
finally {
    Pop-Location
}
