# RTL map

The design targets selected color objects. V21F1 consumes four packed RGB pixels, samples the stream, and compares connected foreground regions. Tracking decisions are sent to the control side through a clock-domain mailbox. A separate frame ID selects historical information for the overlay.

| Task | RTL entry | First test |
| --- | --- | --- |
| Understand camera stream sampling | [camera_analysis_stream](../rtl/v21f1/camera_analysis_stream.v) | `camera_stream` |
| Change how a candidate is accepted | [servo_stream_tracker](../rtl/v21f1/servo_stream_tracker.v) | `servo_background` |
| Tune follow behavior | [gimbal_servo_ctrl](../rtl/v21f1/gimbal_servo_ctrl.v) | `servo_stable` |
| Trace frame identity and overlay | [video_frame_latency](../rtl/v21f1/video_frame_latency.v), [servo_overlay_history](../rtl/v21f1/servo_overlay_history.v) | `frame_meter`, `servo_overlay` |
| Read or extend telemetry | [servo_telemetry](../rtl/v21f1/servo_telemetry.v), [Python decoder](../tools/telemetry_v21.py) | `servo_telemetry` |
| Follow the combined RTL pipeline | [servo_vision_system](../rtl/v21f1/servo_vision_system.v) | `servo_system` |

Run a named test with `python tools/simulate.py --test <name>`. The integration test provides synthetic RGB, emulated exposure control acknowledgments and a delayed display frame ID. It exercises the pixel, camera, control, and display-record paths without the vendor camera PHY, DDR controller, or HDMI IP.

For adaptation, preserve frame start/end and valid semantics on the analysis stream. Keep the display frame ID associated with the frame actually rendered. Re-run the standalone and integration tests when changing a clock interface, and validate CDC constraints again in the target build. See the [evidence notes](evidence.md) for the hardware and simulation boundary.
