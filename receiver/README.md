# 接收端 (RX)

固件：`receiver/pio/rx/`（BLE + WiFi AP + RC 输出 + 网页）

```powershell
cd receiver\pio\rx
pio run -t upload --upload-port COMxx
pio run -e esp32-c3-release          # 正式版
.\flash_rx.ps1 COMxx                 # 新板首次全片刷
```

详细说明见本地 `docs/BUILD.md`、`docs/USAGE.md`（不上传 Git）。
