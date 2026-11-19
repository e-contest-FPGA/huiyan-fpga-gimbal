# 从仿真到上板

## 独立仿真

准备 Python 3.9 或以上版本和 Icarus Verilog。确保 `iverilog` 和 `vvp` 已加入 PATH。从仓库根目录运行以下命令。

```sh
python tools/simulate.py
python tools/simulate.py --test servo_background --test frame_meter
```

输出位于 `out/sim`。测试数据由台架生成，便于复现目标、背景和遮挡条件。

## 完整板级工程

板级目标是 HX1P35A / PH1P35MDG324。配套 SC500 摄像头、HDMI 显示器、双轴舵机、独立舵机电源和下载线。归档构建工具为 Anlogic TD 6.2.1 Engineer（6.2.168116）。

拿到团队原始工程归档后，用下面的命令核对 SHA-256 并解包。

```sh
python tools/restore_board_project.py "SC500_720P_V20S2F2_FrameLatency_20261027.zip"
```

在解包目录中打开 `td_project/camera_to_dsi_display.al`。脚本也接受 `SC500_720P_V21F1_帧延迟与性能优化_20261025.zip`。两版的 [归档与固件哈希](source-provenance.json) 可用于核对工程和固件来源。

首次验证可从 V20S2F2 的 SRAM 下载开始。SW3 下拨并给相机排线留出转动空间，再从下载工具中手动选择固件。两只舵机使用独立 5V 电源并与 FPGA 共地，信号的封装管脚为水平 V2、俯仰 V1。排针编号以对应板卡原理图和工程约束为准。

| 控件 | 操作 |
| --- | --- |
| K3 / K4 | 下一种 / 上一种画面模式 |
| SW1 | 启用运动发现与自动目标学习 |
| SW2 | 手动学习画面中央目标 |
| SW3 | 开启双轴跟随。下拨后缓慢回中 |
| SW4 | 启用目标优先测光。下拨采用全局测光 |

先固定相机，使用 SW2 检查目标框和坐标。确认框能落在目标上后，再开启 SW3 验证方向和跟随。

## 串口记录

V21F1 的诊断快照使用 115200 8N1，数据包为 80 字节并带 CRC。接收端使用适当的 3.3V TTL 串口连接。以下为 Windows 上位机命令。

```powershell
New-Item -ItemType Directory -Force out | Out-Null
powershell -File tools/capture_telemetry.ps1 -Port COM7 -Output out/run.bin -Seconds 60
python tools/telemetry_v21.py out/run.bin --out out/run
```

将 COM7 改成设备实际端口。V21F1 画面上的帧龄来自内部帧号。遥测协议有早期 LED 计时预留字段，在 V21F1 生产配置中处于无效状态。
