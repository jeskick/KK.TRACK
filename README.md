# KK-track 无线头追

TX 读 BNO085 → BLE 发姿态 → RX 输出 PPM / SBUS / CRSF；手机经 WiFi 网页调参。

纯 C / ESP-IDF（PlatformIO），目标 **ESP32-C3**，4MB Flash，双 OTA 槽。

> 更完整的说明在本地 **`docs/`** 目录（不上传 Git）：`BUILD.md`、`USAGE.md`、`HARDWARE.md`、`IMU*.md` 等。

## 工程目录

| 路径 | 用途 |
|------|------|
| [`components/kk/`](components/kk/) | 共享库（RX 工程含 `.kk_rx_project` 标记时编 RX 专用代码） |
| [`receiver/pio/rx/`](receiver/pio/rx/) | 接收端固件 |
| [`transmitter/pio/tx/`](transmitter/pio/tx/) | 发射端固件 |
| [`ota/`](ota/) | 构建产物 `RX.x.y.z.bin` / `TX.x.y.z.bin` |

## 构建

```powershell
cd receiver\pio\rx          # 或 transmitter\pio\tx
pio run                       # 调试版（WARN 日志）
pio run -e esp32-c3-release   # 正式版（无日志，体积更小）
pio run -t upload --upload-port COMxx
pio device monitor --port COMxx --baud 115200   # 仅调试版
```

每次 `pio run` 会自动 bump 版本并复制到 `ota/`（`tools/pio_fw_version.py`）。

**新板首次全片刷**（擦除 + OTA 分区表，先关串口监视器）：

```powershell
cd receiver\pio\rx
.\flash_rx.ps1 COM47

cd transmitter\pio\tx
.\flash_tx.ps1 COM46
```

日常升级用 `pio run -t upload` 或网页 OTA 即可。

## 配对简述

1. RX 上电 BLE 广播 `KK-TRACK-RX`，TX 扫描连接
2. BLE 稳定后 RX 开 WiFi `TRACK-KK` / `12345678`
3. 浏览器 **http://192.168.4.1** 调通道、偏移、IMU 安装角、RC 协议
4. 短按 GPIO9 归零；长按 5s 重配对

## RC 输出

GPIO10：PPM / SBUS / CRSF（网页切换）。CRSF 单向 TX，地址 `0xC8`。
