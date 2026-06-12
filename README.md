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
```

文档入口：**[docs/README.md](docs/README.md)**

| 常用 | 链接 |
|------|------|
| 烧录 / COM 口 | [docs/BUILD.md](docs/BUILD.md) |
| 配对 / 网页 / LED | [docs/USAGE.md](docs/USAGE.md) |
| 引脚 | [docs/HARDWARE.md](docs/HARDWARE.md) |
| IMU 安装基准 | [docs/IMU_MOUNT_REFERENCE.md](docs/IMU_MOUNT_REFERENCE.md) |
