#!/usr/bin/env python3
"""Automated depth data collection for articulated hand calibration.

Drives the hand ONE finger at a time through a grid of its motor values (the
others held at the home pose), and at every step captures an aligned RealSense
point cloud paired with the exact motor state. This is the bulk data for fitting
each finger's motor -> joint model against the mesh.

The arm must stay still for the whole run: the hand->camera transform is solved
once from the home-pose capture (saved with tag `home`) and reused for every
pose, so only the fingers move.

Prereqs, all in WSL: D455 attached (usbipd), the Desk backend running on the
SERIAL backend with the hand servo ENABLED, lehome python.

    python hand_depth_collect.py --out /tmp/handrun1 --fingers index,middle \
        --grid 4 --pip 2 --settle 0.8

Safe to Ctrl-C: it stops the camera and leaves the hand where it is. Nothing
here bypasses the backend — every motion goes through /api/hand/motor, which
does the raw-hold enable, so there is no jerk.
"""
from __future__ import annotations

import argparse
import json
import os
import time
import urllib.request

import numpy as np
import cv2

# home pose per motor (normalized %), from hardware.MotorSpec.home
HOME = {
    "thumb_cmc": 0, "thumb_mcp": 0, "thumb_pip": 0, "thumb_dip": 100,
    "index_radial_flexor": 0, "index_ulnar_flexor": 100, "index_pip_dip": 0,
    "middle_radial_flexor": 0, "middle_ulnar_flexor": 100, "middle_pip_dip": 0,
    "ring_radial_flexor": 0, "ring_ulnar_flexor": 100, "ring_pip_dip": 0,
    "pinky_radial_flexor": 0, "pinky_ulnar_flexor": 100, "pinky_pip_dip": 100,
}
FINGER_MOTORS = {
    "thumb": ["thumb_cmc", "thumb_mcp", "thumb_pip", "thumb_dip"],
    "index": ["index_radial_flexor", "index_ulnar_flexor", "index_pip_dip"],
    "middle": ["middle_radial_flexor", "middle_ulnar_flexor", "middle_pip_dip"],
    "ring": ["ring_radial_flexor", "ring_ulnar_flexor", "ring_pip_dip"],
    "pinky": ["pinky_radial_flexor", "pinky_ulnar_flexor", "pinky_pip_dip"],
}


def _post(backend, path, body):
    req = urllib.request.Request(backend + path, json.dumps(body).encode(),
                                 {"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=2.0) as r:
        return json.load(r)


def _hand_motors(backend):
    with urllib.request.urlopen(backend + "/api/status", timeout=1.0) as r:
        data = json.load(r)
    return {m["name"]: {"pos": m["position"], "cmd": m["command"]}
            for m in data.get("motors", []) if m.get("unit") == "hand"}


def _command_pose(backend, pose):
    for name, v in pose.items():
        _post(backend, "/api/hand/motor", {"name": name, "value": float(v)})


def _grid_poses(finger, grid, pip_levels):
    """Grid over this finger's motors, others at home. For the four fingers:
    radial x ulnar (grid x grid) at a few pip_dip levels. For the thumb: cmc x
    (mcp=pip=dip together) so the sweep stays a 2-D grid instead of exploding."""
    lo, hi = 0.0, 100.0
    axis = np.linspace(lo, hi, grid)
    pips = np.linspace(lo, hi, pip_levels)
    poses = []
    if finger == "thumb":
        for cmc in axis:
            for flex in pips:
                p = dict(HOME)
                p.update({"thumb_cmc": cmc, "thumb_mcp": flex,
                          "thumb_pip": flex, "thumb_dip": 100 - flex})
                poses.append(p)
    else:
        rad, uln, pd = FINGER_MOTORS[finger]
        pd_home = HOME[pd]
        for r in axis:
            for u in axis:
                for pv in pips:
                    p = dict(HOME)
                    # ulnar/pip home may be 100 (pinky): sweep across the full
                    # range regardless of which end home is.
                    p[rad] = r
                    p[uln] = 100 - u if HOME[uln] == 100 else u
                    p[pd] = 100 - pv if pd_home == 100 else pv
                    poses.append(p)
    return poses


def _capture(pipe, align, rs, intr_holder):
    frames = align.process(pipe.wait_for_frames())
    d = frames.get_depth_frame()
    c = frames.get_color_frame()
    if not d or not c:
        return None
    if intr_holder[0] is None:
        intr_holder[0] = c.profile.as_video_stream_profile().intrinsics
    scale = intr_holder[1]
    depth = np.asanyarray(d.get_data()).astype(np.float32) * scale
    color = np.asanyarray(c.get_data())
    return depth, color


def _save_cloud(path, depth, color, intr, near, far):
    h, w = depth.shape
    ys, xs = np.mgrid[0:h, 0:w]
    z = depth
    valid = (z > near) & (z < far)
    x = (xs - intr.ppx) / intr.fx * z
    y = (ys - intr.ppy) / intr.fy * z
    pts = np.stack([x[valid], y[valid], z[valid]], axis=1).astype(np.float32)
    rgb = (color[valid][:, ::-1]).astype(np.uint8)
    np.savez_compressed(path, points=pts, colors=rgb)
    return len(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--fingers", default="index,middle,ring,pinky,thumb")
    ap.add_argument("--grid", type=int, default=4, help="steps per motor axis")
    ap.add_argument("--pip", type=int, default=2, help="pip_dip levels")
    ap.add_argument("--settle", type=float, default=0.8, help="s to wait after a move")
    ap.add_argument("--near", type=float, default=0.2)
    ap.add_argument("--far", type=float, default=0.6)
    ap.add_argument("--backend", default="http://localhost:8000")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    fingers = [f.strip() for f in args.fingers.split(",") if f.strip()]

    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    profile = pipe.start(cfg)
    scale = profile.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    intr_holder = [None, scale]

    index = []
    try:
        for _ in range(30):
            pipe.wait_for_frames()

        # 1) home pose — the palm-fit reference for the whole run
        print("home pose (palm reference)…")
        _command_pose(args.backend, HOME)
        time.sleep(max(1.5, args.settle * 2))
        depth, color = _capture(pipe, align, rs, intr_holder)
        p = os.path.join(args.out, "home.npz")
        n = _save_cloud(p, depth, color, intr_holder[0], args.near, args.far)
        cv2.imwrite(os.path.join(args.out, "home.jpg"), color)  # for ArUco T-solve
        index.append({"file": "home.npz", "img": "home.jpg", "finger": "home",
                      "motors": _hand_motors(args.backend), "n": n})
        print(f"  saved home.npz ({n} pts)")

        total = sum(len(_grid_poses(f, args.grid, args.pip)) for f in fingers)
        print(f"sweeping {fingers}: {total} poses "
              f"(~{total * (args.settle + 0.3) / 60:.1f} min)")
        k = 0
        for finger in fingers:
            poses = _grid_poses(finger, args.grid, args.pip)
            for pose in poses:
                _command_pose(args.backend, pose)
                time.sleep(args.settle)
                depth, color = _capture(pipe, align, rs, intr_holder)
                fn = f"{finger}_{k:04d}.npz"
                img = f"{finger}_{k:04d}.jpg"
                n = _save_cloud(os.path.join(args.out, fn), depth, color,
                                intr_holder[0], args.near, args.far)
                cv2.imwrite(os.path.join(args.out, img), color)  # for ArUco tracking
                index.append({"file": fn, "img": img, "finger": finger,
                              "motors": _hand_motors(args.backend), "n": n})
                k += 1
                if k % 10 == 0:
                    print(f"  {k}/{total}  ({fn}, {n} pts)")
        # return home
        _command_pose(args.backend, HOME)
        print(f"done: {k} poses")
    except KeyboardInterrupt:
        print("\ninterrupted — camera stopped, hand left in place")
    finally:
        pipe.stop()
        intr = intr_holder[0]
        meta = {
            "fingers": fingers, "grid": args.grid, "pip": args.pip,
            "near": args.near, "far": args.far,
            "intrinsics": ({"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx,
                            "ppy": intr.ppy, "w": 848, "h": 480,
                            "coeffs": list(intr.coeffs),
                            "model": str(intr.model)} if intr else None),
            "depth_scale": scale, "samples": index,
        }
        with open(os.path.join(args.out, "index.json"), "w") as f:
            json.dump(meta, f, indent=1)
        print(f"index.json written ({len(index)} clouds)")


if __name__ == "__main__":
    main()
