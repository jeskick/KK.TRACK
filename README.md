# KK-track 无线头追

TX 读 BNO085 → BLE 发姿态 → RX 输出 PPM 舵机；手机经 WiFi 网页调参。

纯 C / ESP-IDF（PlatformIO），目标 **ESP32-C3**。

## 工程

| 目录 | 用途 |
|------|------|
| [`components/kk/`](components/kk/) | 共享库 |
| [`receiver/pio/rx/`](receiver/pio/rx/) | **接收端** |
| [`transmitter/pio/tx/`](transmitter/pio/tx/) | **发射端** |
| [`transmitter/pio/imu_test/`](transmitter/pio/imu_test/) | IMU 独立测试 |

## 快速开始

```powershell
cd receiver\pio\rx    # 或 transmitter\pio\tx
pio run -t upload --upload-port COMxx
pio device monitor --port COMxx --baud 115200
```

首次 clone 后若缺少 `components/kk` 联接，在本地运行 `docs/tools/link_kk_component.ps1`（文档目录仅保留在本地，不上传 GitHub）。

## 配对简述

1. RX 上电 BLE 广播 `KK-TRACK-RX`，TX 扫描连接
2. BLE 稳定后 RX 开 WiFi `TRACK-KK` / `12345678`
3. 浏览器 **http://192.168.4.1** 调通道、偏移、IMU 安装角
4. 短按 GPIO9 归零；长按 5s 重配对
