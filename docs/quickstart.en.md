# Quick start

## Run the pipeline simulation

Install Python 3.9 or later and Icarus Verilog. Both `iverilog` and `vvp` must be on your PATH.

```sh
git clone https://github.com/e-contest-FPGA/huiyan-fpga-gimbal.git
cd huiyan-fpga-gimbal
python tools/simulate.py --test servo_system
python tools/simulate.py
```

The integration test feeds synthetic camera data through target tracking, frame-matched overlay, exposure, and servo control. The full command runs nine HDL suites. Check `out/sim/` for logs. The integrated bench substitutes synthetic inputs for the physical camera, DDR interface, and servos.

## Bring up an HX1P35A board

Use the HX1P35A (PH1P35MDG324), an SC500CS MIPI camera, an HDMI display, and two PWM servos. Use an independent 5 V supply for the servos and connect its negative terminal to the board ground at J1 pin 12. Pan signal is J1 pin 1 (V2) and tilt signal is J1 pin 2 (V1). Both control outputs are 3.3 V. Identify pin 1 on the board before wiring. See the [wiring diagram](hardware.md#连接).

Use the official [Anlogic tool downloads](https://www.anlogic.com/support/tools-downloads) and [contest reference package](https://www.anlogic.com/support/university/competitions/eda/news1/42.html) for the vendor materials. The team's board archives have different contents from the vendor examples. A matching team archive is also required. If you have board access and want to reproduce the build, [open a board reproduction issue](https://github.com/e-contest-FPGA/huiyan-fpga-gimbal/issues/new?template=board.yml) to ask the maintainers about archive availability.

With the team's V20S2F2 archive on disk:

```sh
python tools/restore_board_project.py "SC500_720P_V20S2F2_FrameLatency_20261027.zip"
```

Open the extracted `td_project/camera_to_dsi_display.al` in Anlogic TD 6.2.1 Engineer (6.2.168116). Confirm the PH1P35MDG324 part and select the matching V20S2F2 bitstream for SRAM programming. Keep SW3 down while checking HDMI, put a colored target in the center and activate SW2, then enable servo following with SW3 after verifying mechanical clearance.

| Version | Evidence |
| --- | --- |
| V20S2F | Visible in the original competition video |
| V20S2F2 | Archived board boot and frame-age observation |
| V21F1 | Newer pre-DDR tracking path, with simulation and routed build evidence. Board validation is pending |

Frame age counts capture frame periods to internal rendering. See the [evidence notes](evidence.md) for the endpoints and [hardware protocol](hardware-validation.md) for the planned comparison.
