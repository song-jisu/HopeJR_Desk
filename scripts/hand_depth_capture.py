#!/usr/bin/env python3
"""Stage 1 of depth-based hand calibration: capture aligned depth+color from a
RealSense D455 and save a cropped point cloud of the robot hand.

This is the gate for the whole articulated-fit pipeline: if the D455 cannot get
a clean, metric point cloud of the (small) robot hand at close range, the rest
is moot. So this script prints diagnostics — how many points survive the depth
crop, the working distance, and the point spacing on the hand — and saves both
the raw color/depth and the cropped cloud for inspection.

Run in WSL with the D455 attached (usbipd), lehome python:
    python hand_depth_capture.py --out /tmp/handcap --near 0.2 --far 0.6

Then open the saved .ply in the 3D viewer / MeshLab, or run the inspect step.
Nothing here touches the robot; it only reads the camera.
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/handcap", help="output dir")
    ap.add_argument("--near", type=float, default=0.15, help="depth crop near (m)")
    ap.add_argument("--far", type=float, default=0.60, help="depth crop far (m)")
    ap.add_argument("--warmup", type=int, default=30, help="frames to let AE settle")
    ap.add_argument("--tag", default="", help="label for this capture")
    ap.add_argument("--backend", default="http://localhost:8000",
                    help="Desk backend, to log the hand motor state with the cloud")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    import pyrealsense2 as rs

    pipe = rs.pipeline()
    cfg = rs.config()
    # D455: depth + color, both 848x480 @30 is a robust choice for close range.
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    profile = pipe.start(cfg)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()          # metres per unit
    # Align depth INTO the color frame so every colour pixel has a depth.
    align = rs.align(rs.stream.color)

    try:
        for _ in range(args.warmup):                       # let auto-exposure settle
            pipe.wait_for_frames()

        frames = align.process(pipe.wait_for_frames())
        d = frames.get_depth_frame()
        c = frames.get_color_frame()
        if not d or not c:
            print("ERROR: no aligned depth/color frame")
            return

        intr = c.profile.as_video_stream_profile().intrinsics
        depth = np.asanyarray(d.get_data()).astype(np.float32) * depth_scale   # m
        color = np.asanyarray(c.get_data())                                    # BGR

        # deproject every valid, in-range pixel to a 3D point (camera frame)
        h, w = depth.shape
        ys, xs = np.mgrid[0:h, 0:w]
        z = depth
        valid = (z > args.near) & (z < args.far)
        x = (xs - intr.ppx) / intr.fx * z
        y = (ys - intr.ppy) / intr.fy * z
        pts = np.stack([x[valid], y[valid], z[valid]], axis=1)
        rgb = color[valid][:, ::-1] / 255.0                # BGR->RGB, 0..1

        # save
        base = os.path.join(args.out, "cap" + (f"_{args.tag}" if args.tag else ""))
        np.savez(base + ".npz", depth=depth, color=color,
                 fx=intr.fx, fy=intr.fy, ppx=intr.ppx, ppy=intr.ppy,
                 near=args.near, far=args.far)
        _write_ply(base + ".ply", pts, rgb)
        meta = {
            "tag": args.tag, "t": round(time.time(), 1),
            "intrinsics": {"fx": intr.fx, "fy": intr.fy,
                           "ppx": intr.ppx, "ppy": intr.ppy, "w": w, "h": h},
            "depth_scale": depth_scale, "near": args.near, "far": args.far,
            "n_points": int(len(pts)),
            # pair the cloud with the motor state so the fitted joint angles can
            # be regressed against motor values later
            "hand_motors": _read_hand_motors(args.backend),
        }
        with open(base + ".json", "w") as f:
            json.dump(meta, f, indent=2)

        # diagnostics — is this cloud usable?
        print(f"saved {base}.ply  ({len(pts)} points in [{args.near},{args.far}] m)")
        if len(pts):
            zc = pts[:, 2]
            print(f"  working distance: {zc.min():.3f}–{zc.max():.3f} m "
                  f"(median {np.median(zc):.3f})")
            # point spacing at the median distance = pixel size projected
            spac = np.median(zc) / intr.fx * 1000
            print(f"  ~point spacing on the hand: {spac:.2f} mm "
                  f"({'OK, finger detail resolvable' if spac < 2 else 'coarse — move closer'})")
            # rough bounding box of the in-range cloud (the hand, if isolated)
            bb = pts.max(0) - pts.min(0)
            print(f"  cropped bbox: {bb[0]*1000:.0f} x {bb[1]*1000:.0f} x "
                  f"{bb[2]*1000:.0f} mm  (a hand is ~100 x 100 x 40)")
        else:
            print("  NO POINTS in range — adjust --near/--far or move the hand "
                  "into 0.2–0.6 m and clear the background.")
    finally:
        pipe.stop()


def _read_hand_motors(backend: str) -> dict:
    """Current hand motor positions/commands from the running Desk backend, so
    each captured cloud is paired with the motor state. Empty if unreachable."""
    try:
        import urllib.request
        with urllib.request.urlopen(backend + "/api/status", timeout=1.0) as r:
            data = json.load(r)
        return {m["name"]: {"pos": m["position"], "cmd": m["command"]}
                for m in data.get("motors", []) if m.get("unit") == "hand"}
    except Exception:
        return {}


def _write_ply(path: str, pts: np.ndarray, rgb: np.ndarray) -> None:
    n = len(pts)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        col = (np.clip(rgb, 0, 1) * 255).astype(int)
        for p, cc in zip(pts, col):
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {cc[0]} {cc[1]} {cc[2]}\n")


if __name__ == "__main__":
    main()
