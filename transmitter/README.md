# 发射端 (TX)

| 工程 | 用途 |
|------|------|
| [`pio/tx/`](pio/tx/) | 主固件（BLE + IMU） |
| [`pio/imu_test/`](pio/imu_test/) | BNO085 独立测试 |

```powershell
cd transmitter\pio\tx
pio run -t upload --upload-port COMxx
```

见 [../docs/BUILD.md](../docs/BUILD.md)、[../docs/IMU.md](../docs/IMU.md)。
