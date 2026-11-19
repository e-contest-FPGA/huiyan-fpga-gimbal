"""Run the FPGA-independent HDL checks with Icarus Verilog."""

import argparse
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
JOBS = {
    "frame_meter": ["video_frame_latency.v"],
    "camera_stream": ["camera_analysis_stream.v"],
    "servo_background": ["servo_stream_tracker.v", "servo_color_bin.v", "video_target_ram.v"],
    "servo_stable": ["gimbal_servo_ctrl.v"],
    "servo_cdc": ["servo_cdc.v"],
    "servo_overlay": ["servo_overlay_history.v"],
    "servo_exposure": ["servo_exposure.v", "servo_cdc.v"],
    "servo_telemetry": ["servo_telemetry.v"],
}


def run_test(name, compiler, simulator, out):
    top = f"tb_{name}"
    target = f"{name}.vvp"
    sources = [f"../../sim/{top}.sv", *[f"../../rtl/v21f1/{module}" for module in JOBS[name]]]
    compile_run = subprocess.run(
        [compiler, "-g2012", "-s", top, "-o", target, *sources],
        cwd=out, input=b"", capture_output=True, timeout=45,
    )
    log = compile_run.stdout + compile_run.stderr
    if compile_run.returncode == 0:
        simulation = subprocess.run(
            [simulator, "-n", target], cwd=out, input=b"", capture_output=True, timeout=120,
        )
        log += simulation.stdout + simulation.stderr
        ran_ok = simulation.returncode == 0
    else:
        ran_ok = False
    (out / f"{name}.log").write_bytes(log)
    passed = ran_ok and b"PASS" in log and b"FAIL" not in log and b"FATAL" not in log
    marker = next((line for line in log.splitlines() if line.startswith(b"PASS")), b"")
    result = marker if passed else log
    print(f"{name}: {result.decode('utf-8', errors='replace').strip()}", flush=True)
    return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test", action="append", choices=JOBS)
    args = parser.parse_args()
    compiler, simulator = shutil.which("iverilog"), shutil.which("vvp")
    if not compiler or not simulator:
        parser.error("Icarus Verilog is required. Put iverilog and vvp on PATH.")
    out = ROOT / "out" / "sim"
    out.mkdir(parents=True, exist_ok=True)
    results = [run_test(name, compiler, simulator, out) for name in (args.test or JOBS)]
    print(f"{sum(results)}/{len(results)} HDL suites passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
