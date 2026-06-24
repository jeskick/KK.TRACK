# 发射端 (TX)

固件：`transmitter/pio/tx/`（BLE 中心 + BNO085 IMU）

```powershell
cd transmitter\pio\tx
pio run -t upload --upload-port COMxx
pio run -e esp32-c3-release          # 正式版
.\flash_tx.ps1 COMxx                 # 新板首次全片刷
```

详细说明见本地 `docs/BUILD.md`、`docs/IMU.md`（不上传 Git）。
