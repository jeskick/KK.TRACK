# Full flash: bootloader + IDF partition table + OTA data + app (TWO_OTA_LARGE).
# Usage (close serial monitor first):
#   .\flash_rx.ps1 COM47
param(
    [string]$Port = "COM47"
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Build = Join-Path $Root ".pio\build\esp32-c3"
$Pio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pio.exe"
$Py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
$Esptool = Join-Path $env:USERPROFILE ".platformio\packages\tool-esptoolpy\esptool.py"
$Ninja = Join-Path $env:USERPROFILE ".platformio\packages\tool-ninja\ninja.exe"

foreach ($f in @($Pio, $Py, $Esptool, $Ninja)) {
    if (-not (Test-Path $f)) { throw "Missing: $f" }
}

function Ensure-IdfFlashAssets {
    param([string]$BuildDir)
    $part = Join-Path $BuildDir "partition_table\partition-table.bin"
    $ota = Join-Path $BuildDir "ota_data_initial.bin"
    if ((Test-Path $part) -and (Test-Path $ota)) { return }

    Write-Host "Building IDF partition table + OTA data ..."
    Push-Location $BuildDir
    try {
        & $Ninja "partition_table/partition-table.bin" "ota_data_initial.bin" | Out-Host
        if ($LASTEXITCODE -ne 0) { throw "ninja partition/ota targets failed" }
    }
    finally {
        Pop-Location
    }
    if (-not (Test-Path $part)) { throw "Missing: $part (OTA partition table not built)" }
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
            if ($leaf -eq "kk_rx.bin") { $bin = Join-Path $BuildDir "firmware.bin" }
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

    # Full flash uses stub (not --no-stub): chip is erased, nothing on UART yet.
    # --no-stub is only for pio upload when old firmware may spam the serial line.
    $writeArgs = @(
        $Esptool, "--chip", "esp32c3", "--port", $Port, "--baud", "115200",
        "--connect-attempts", "25",
        "--before", "default_reset", "--after", "hard_reset",
        "write_flash", "--erase-all"
    ) + $plan.FlashMode
    foreach ($img in $plan.Images) {
        $writeArgs += @($img.Offset, $img.Path)
    }

    Write-Host "Erasing + flashing full image on $Port (close serial monitor first) ..."
    & $Py @writeArgs
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Retrying in 2s ..."
        Start-Sleep -Seconds 2
        & $Py @writeArgs
        if ($LASTEXITCODE -ne 0) { throw "flash failed" }
    }

    Write-Host "Done. Boot log should show ota_0 / ota_1 and OTA target=ota_1."
    Write-Host "ota/RX.*.bin updated by pio pre/post (tools/pio_fw_version.py)."
}
finally {
    Pop-Location
}
