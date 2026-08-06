#!/usr/bin/env python3
"""Live ArUco positioning helper for the depth calibration.

Run this at the robot BEFORE a depth collection run. It grabs a few RealSense
color frames, tries every common ArUco dictionary, and for each detected marker
reports its id, how obliquely the camera sees it (tilt of the marker face vs the
view ray) and the corner sharpness. Use it to nudge each of the 3 camera
positions until at least one palm-side marker is detected at a healthy tilt
(< ~55 deg) with sharp corners — that is what makes the per-camera hand->camera
transform solvable from the home frame.

    python aruco_live.py                # auto-detect dictionary, 30 frames
    python aruco_live.py --dict 4X4_50  # force a dictionary
    python aruco_live.py --marker-mm 20 # also print metric distance

Writes an annotated JPG (aruco_live.jpg) you can open to see the outlines.
Nothing moves the robot; camera only.
"""
from __future__ import annotations
import argparse, math, os
import numpy as np, cv2

DICTS = ["DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50",
         "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_36h11"]


def tilt_deg(rvec):
    """Angle between the marker's +Z (face normal) and the camera view ray.
    0 deg = camera looks straight at the marker face; 90 = edge-on."""
    R, _ = cv2.Rodrigues(rvec)
    n = R[:, 2]                       # marker normal in camera frame
    # view ray toward the marker is ~ +Z of camera; use marker's own position dir
    return math.degrees(math.acos(min(1.0, abs(n[2]))))


def corner_sharp(gray, corners):
    """Mean gradient magnitude around the marker corners (rough sharpness)."""
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.hypot(gx, gy)
    vals = []
    for c in corners.reshape(-1, 2):
        x, y = int(round(c[0])), int(round(c[1]))
        y0, y1 = max(0, y-4), min(mag.shape[0], y+5)
        x0, x1 = max(0, x-4), min(mag.shape[1], x+5)
        vals.append(mag[y0:y1, x0:x1].max())
    return float(np.mean(vals)) if vals else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dict", default=None, help="e.g. 4X4_50; default: try all")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--marker-mm", type=float, default=None)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "aruco_live.jpg"))
    args = ap.parse_args()

    import pyrealsense2 as rs
    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    profile = pipe.start(cfg)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.0]])
    dist = np.array(intr.coeffs[:5], float)

    dict_names = [f"DICT_{args.dict}"] if args.dict else DICTS
    detectors = [(n, cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, n)),
        cv2.aruco.DetectorParameters())) for n in dict_names]

    try:
        for _ in range(10):
            pipe.wait_for_frames()
        best_report, best_img = None, None
        seen = {}
        for f in range(args.frames):
            color = np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data())
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            for name, det in detectors:
                corners, ids, _ = det.detectMarkers(gray)
                if ids is None:
                    continue
                anno = color.copy(); cv2.aruco.drawDetectedMarkers(anno, corners, ids)
                lines = []
                mm = args.marker_mm / 1000.0 if args.marker_mm else 0.02
                obj = np.array([[-mm/2, mm/2, 0], [mm/2, mm/2, 0],
                                [mm/2, -mm/2, 0], [-mm/2, -mm/2, 0]], float)
                for c, i in zip(corners, ids.flatten()):
                    ok, rvec, tvec = cv2.solvePnP(obj, c[0], K, dist)
                    t = tilt_deg(rvec) if ok else float("nan")
                    s = corner_sharp(gray, c)
                    dist_m = float(np.linalg.norm(tvec)) if ok else float("nan")
                    lines.append(f"    id {i:>2} [{name.replace('DICT_','')}]  "
                                 f"tilt {t:4.0f}deg  sharp {s:5.0f}"
                                 + (f"  dist {dist_m*100:4.1f}cm" if args.marker_mm else ""))
                    seen.setdefault((name, int(i)), []).append(t)
                    cv2.putText(anno, f"{i}:{t:.0f}deg", tuple(c[0][0].astype(int)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if best_report is None or len(lines) >= len(best_report[1]):
                    best_report = (name, lines); best_img = anno
        if best_img is not None:
            cv2.imwrite(args.out, best_img)
            print("detected markers (best frame):")
            for l in best_report[1]:
                print(l)
            print("\nseen across frames:")
            for (name, i), tl in sorted(seen.items()):
                print(f"    dict {name.replace('DICT_','')}  id {i}: "
                      f"{len(tl)}/{args.frames} frames, tilt ~{np.median(tl):.0f}deg")
            print(f"\nannotated -> {args.out}")
            print("GOAL: at least one marker every frame, tilt < ~55deg, sharp > ~300")
        else:
            print("NO markers detected in any dictionary — check the marker faces "
                  "the camera and is not motion-blurred or too oblique.")
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
