# 参与慧眼随行

欢迎提交可复现的 RTL 问题、兼容性报告和文档修改。代码变更应说明涉及的版本、行为差异和验证范围。英文问题可以直接提交。

## 本地检查

```sh
python tools/check_sources.py
python tools/simulate.py --test servo_system
python tools/simulate.py
```

改动跟踪、曝光、CDC 或控制时，运行相关台架和集成测试，在 PR 中给出命令与结果。若改动针对特定板卡，请附板卡、相机、工具版本和固件 SHA-256。传感器画面、综合时序和机械表现分别记录。

`rtl/` 包含来自历史归档的发布快照。修复涉及这些文件时，保留原始来源和哈希记录，并在 PR 中标注偏离归档的内容。资产应使用已获授权的材料。厂商 IP 和现成固件的权利范围见 [NOTICE](NOTICE.md)。

可以从[待办路线](docs/roadmap.md)或[公开 Issues](https://github.com/e-contest-FPGA/huiyan-fpga-gimbal/issues)开始。
