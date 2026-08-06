#!/usr/bin/env python3
"""Record a slow-rotation clip to recover T(marker0 <-> marker1).

The two ArUco markers sit on opposite side faces of the palm, so they are never
co-visible and their relative pose can't be solved from a single frame. But both
are rigid on the same hand, so if we slowly rotate the hand past a fixed camera —
seeing marker 0, then (after ~180 deg) marker 1 — we can chain the two marker
frames through the rigid-body motion between them (frame-to-frame depth ICP).

Run this ONCE (whenever the markers are re-stuck). Fix the D455 ~40 cm from the
hand, start recording, and rotate the hand slowly through its full range
(wrist_roll slider end-to-end, or turn the arm by hand) so the camera sweeps from
one side face to the other. ~15-25 s is plenty.

    python marker_bridge_capture.py --out /tmp/mbridge --secs 20

Saves color jpg + point-cloud npz per frame + index.json. Nothing moves the
robot; you do the rotating.
"""
from __future__ import annotations
import argparse, json, os, time
import numpy as np
import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--secs", type=float, default=20.0, help="record duration")
    ap.add_argument("--hz", type=float, default=6.0, help="frames per second to save")
    ap.add_argument("--near", type=float, default=0.15)
    ap.add_argument("--far", type=float, default=0.6)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    import pyrealsense2 as rs
    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    profile = pipe.start(cfg)
    scale = profile.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    intr = None
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    det = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())

    index = []
    period = 1.0 / args.hz
    try:
        for _ in range(30):
            pipe.wait_for_frames()
        print(f"recording {args.secs:.0f}s — rotate the hand slowly now "
              f"(marker0 side -> marker1 side)…")
        t0 = time.time(); nxt = 0.0; k = 0
        while time.time() - t0 < args.secs:
            frames = align.process(pipe.wait_for_frames())
            d = frames.get_depth_frame(); c = frames.get_color_frame()
            if not d or not c:
                continue
            el = time.time() - t0
            if el < nxt:
                continue
            nxt = el + period
            if intr is None:
                intr = c.profile.as_video_stream_profile().intrinsics
            depth = np.asanyarray(d.get_data()).astype(np.float32) * scale
            color = np.asanyarray(c.get_data())
            h, w = depth.shape
            ys, xs = np.mgrid[0:h, 0:w]
            valid = (depth > args.near) & (depth < args.far)
            x = (xs - intr.ppx) / intr.fx * depth
            y = (ys - intr.ppy) / intr.fy * depth
            pts = np.stack([x[valid], y[valid], depth[valid]], 1).astype(np.float32)
            rgb = color[valid][:, ::-1].astype(np.uint8)
            fn = f"f{k:04d}"
            np.savez_compressed(os.path.join(args.out, fn + ".npz"), points=pts, colors=rgb)
            cv2.imwrite(os.path.join(args.out, fn + ".jpg"), color)
            # live marker report so the user knows the sweep covered both
            g = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            _, ids, _ = det.detectMarkers(g)
            seen = sorted(int(i) for i in ids.flatten()) if ids is not None else []
            index.append({"file": fn + ".npz", "img": fn + ".jpg", "t": round(el, 2),
                          "markers": seen, "n": len(pts)})
            if k % 5 == 0:
                print(f"  {el:4.1f}s  frame {k}  markers {seen}  ({len(pts)} pts)")
            k += 1
        print(f"done: {k} frames")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        pipe.stop()
        meta = {"near": args.near, "far": args.far, "depth_scale": scale,
                "intrinsics": ({"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx,
                                "ppy": intr.ppy, "w": 848, "h": 480,
                                "coeffs": list(intr.coeffs)} if intr else None),
                "frames": index}
        with open(os.path.join(args.out, "index.json"), "w") as f:
            json.dump(meta, f, indent=1)
        m0 = sum(1 for x in index if 0 in x["markers"])
        m1 = sum(1 for x in index if 1 in x["markers"])
        both = sum(1 for x in index if 0 in x["markers"] and 1 in x["markers"])
        print(f"index.json written ({len(index)} frames). "
              f"marker0 in {m0}, marker1 in {m1}, both in {both} frames.")
        if m0 < 3 or m1 < 3:
            print("WARNING: need >=3 frames each of marker0 AND marker1 — "
                  "rotate further so both side faces pass the camera.")


if __name__ == "__main__":
    main()
