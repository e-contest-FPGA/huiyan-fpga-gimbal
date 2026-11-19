# 从仿真到上板

## 独立仿真

准备 Python 3.9 或以上版本和 Icarus Verilog。确保 `iverilog` 和 `vvp` 已加入 PATH。从仓库根目录运行以下命令。

```sh
python tools/simulate.py
python tools/simulate.py --test servo_background --test frame_meter
python tools/simulate.py --test servo_system
```

输出位于 `out/sim`。全量命令运行 9 组 HDL 台架，集成台架使用合成相机输入与模拟的显示帧号。若想顺带试串口解码，先运行 `--test servo_telemetry`，再运行 `python tools/telemetry_v21.py out/sim/rtl_uart_synthetic.bin --out out/sim/telemetry`。该文件由仿真产生。

## 完整板级工程

板级目标是 HX1P35A / PH1P35MDG324。硬件与连线见[接线图](hardware.md)。归档构建工具为 Anlogic TD 6.2.1 Engineer（6.2.168116）。工具从[安路工具下载](https://www.anlogic.com/support/tools-downloads)获取。

[安路官方赛题指南第 8 页](https://www.anlogic.com/Public/Uploads/ueditor/upload/file/20260826/1787734941769836.pdf#page=8)列出厂商参考包 `HX1P35A_Contest_202606` 和 JTAG、MIPI、HDMI 示例。SC500CS 参考链路从 `7_lab_ex_2026_nosoft/lab_ex1_mipi_hdmi_SC500_720P` 入手。厂商参考包可用于验证基础采集与显示，完整云台构建还依赖下面这两份团队归档中的对应版本。

团队归档含厂商 IP，读者可通过[上板复现 Issue](https://github.com/e-contest-FPGA/huiyan-fpga-gimbal/issues/new?template=board.yml)询问获取路径与可分发范围。已有原始归档后，用下面的命令同时核对归档和固件的 SHA-256 并解包。

```sh
python tools/restore_board_project.py "SC500_720P_V20S2F2_FrameLatency_20261027.zip"
```

在解包目录中打开 `td_project/camera_to_dsi_display.al`。脚本也接受 `SC500_720P_V21F1_帧延迟与性能优化_20261025.zip`。两版的 [归档与固件哈希](source-provenance.json) 可用于核对工程和固件来源。

首次验证按[上板步骤](hardware.md#上电顺序)从 V20S2F2 的 SRAM 下载开始。两版同场对照用[实机验收方案](hardware-validation.md)。

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
