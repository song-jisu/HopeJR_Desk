# HopeJR Desk

A **research Robot Operating Interface** for the HopeJR right arm + hand — a
web dashboard inspired by Franka Research 3's *Desk*, redesigned for HopeJR's
low-cost hardware (Feetech servos, tendon-driven hand, ROS2 + lerobot runtime).

The goal (see [`PLAN.md`](PLAN.md)): let a researcher run most experiments —
monitoring, control, teaching, grasping, dataset recording — **from a browser,
without editing code**.

> Status: **MVP**. Working today — Dashboard, Robot Control, Hand Control,
> Configuration, Logs — driven by a live telemetry stream, over either a **mock
> simulator** (no robot needed) or the **real robot via ROS2**. Teaching, Task
> Editor, and Dataset Recording are scaffolded in the plan and API shape but not
> yet implemented (see [Roadmap](#roadmap)).

---

## Setup

Clone the project, then pull the ROS2 description/bridge packages into
`ros2_ws/src` (they live in their own repos, not vendored here):

```bash
git clone https://github.com/song-jisu/HopeJR_Desk
cd HopeJR_Desk
bash ros2_ws/clone_hopejr_src.sh
```

`clone_hopejr_src.sh` clones — or, if already present, `git pull`s — the five
`hopejr_*` packages (`hopejr_arm_description`, `hopejr_hand_description`,
`hopejr_right_arm_description`, `hopejr_state_publisher`, `hopejr_real_bridge`)
from [`song-jisu`](https://github.com/song-jisu) into `ros2_ws/src`. Re-run it
any time to update; a non-git package dir is backed up to `*.bak_<timestamp>`
before cloning, so local edits are never silently lost.

### Python environment (`.venv`)

The backend needs FastAPI / uvicorn / pydantic / numpy. Create a virtualenv at
the repo root — `.venv/` is already in `.gitignore`:

```bash
# with uv (recommended — works even without the python3-venv system package)
uv venv .venv --python 3.12 --prompt .
uv pip install --python .venv/bin/python -r backend/requirements.txt

# ...or with stock venv (Debian/Ubuntu: sudo apt install python3.12-venv first)
python3 -m venv .venv --prompt .
.venv/bin/pip install -r backend/requirements.txt
```

`--prompt .` makes the shell show `(HopeJR_Desk)` instead of a useless `(.venv)`.

Activate it with `source .venv/bin/activate` (Windows: `.venv\Scripts\activate`),
or just call `.venv/bin/python` directly. Smoke-test it:

```bash
cd backend && HOPEJR_BACKEND=mock ../.venv/bin/python -m uvicorn app.main:app --port 8000
# -> http://localhost:8000/docs
```

> In mock mode `scripts/run_backend.sh` shells out to `uv run --with …`, so it uses its
> own ephemeral env rather than this `.venv`. The command above is the way to run mock
> mode *inside* `.venv`.

This env covers **mock mode**, which is all you need for UI work. It is
deliberately *not* enough for the real-robot paths:

| Need | Not in `.venv` | Where it comes from |
|---|---|---|
| `serial` mode | `lerobot`, `scservo_sdk` | the existing lerobot env — see [serial mode](#real-robot--serial-mode-simplest-no-ros2) |
| `ros` mode | `rclpy`, `std_msgs`, `sensor_msgs` | the system ROS2 install (not pip-installable) |
| `scripts/` helpers | `opencv-python`, `scipy`, `pyrealsense2` | `pip install` them as needed |

For ROS mode in one env, source ROS2 first and build the venv with system
packages visible: `python3 -m venv --system-site-packages .venv`.

---

## Architecture

HopeJR Desk is a thin **web layer + Robot Manager Server** on top of the
**existing HopeJR ROS2 stack** (copied into [`ros2_ws/`](ros2_ws/) from the
`hopejr_right_arm` workspace). It reuses the exact command path that
`teleop_server.py` already listens on, so nothing about the real-robot runtime
changes.

```
   Browser (React, FR3-Desk-style UI)
        │  REST /api/*      WebSocket /ws/telemetry
        ▼
   Robot Manager Server  (FastAPI, backend/)
        │  RobotManager: telemetry loop · finger state machines · config · logs
        │
        ├── mock backend   ── in-process servo simulator (default; no HW)
        │
        ├── serial backend ── lerobot FeetechMotorsBus (no ROS2)
        │        ├─ writes Goal_Position ─► /dev/hopejr_arm (7, proto 0)
        │        │                          /dev/hopejr_hand (16, proto 1)  ─► HopeJR
        │        └─ reads Present_Position/Current/Temperature ◄─ (live feedback)
        │
        └── ros backend ── Float64MultiArray
                 ├──/arm_goals  (7) ──┐
                 └──/hand_goals (16) ──┴─► teleop_server.py ─► Feetech Servo Bus ─► HopeJR
                          ▲
                 /joint_states (feedback, when a publisher runs)
```

The **same REST/WebSocket API** works in every mode — pick with
`HOPEJR_BACKEND=mock|serial|ros`:

| mode | talks to | needs | feedback (current/temp) | use when |
|---|---|---|---|---|
| `mock` | in-process sim | nothing | simulated | dev / demo, no robot |
| `serial` | Feetech bus via **lerobot** | lerobot env, serial port | **live** (arm current, hand load) | robot on the **same machine** as Desk |
| `ros` | `teleop_server.py` over DDS | ROS2 sourced + teleop_server | needs a feedback publisher | robot on a **separate machine** (network) |

> **Serial vs ROS — which for a real-time (PreemptRT) control box?**
> Serial is always the physical layer regardless of mode. If the robot and Desk
> run on the **same** RT machine, use `serial`: the tight write/read control loop
> lives in the Desk process, so the PreemptRT kernel directly reduces its jitter.
> If you **split** Desk (dev laptop) from the control box (RT), run `teleop_server.py`
> on the RT box (it owns the serial loop, protected by RT scheduling) and point
> Desk's `ros` backend at it over the network. Note: Python + pyserial is *soft*
> real-time (GIL/userspace) — fine for Feetech position servos at 50–100 Hz; a
> C++ node is the path only if you later need *hard* RT.

Mock mode simulates position tracking, current (∝ position error) and
temperature so the whole UI and telemetry pipeline can be developed and demoed
with no hardware attached.

### How it relates to the existing `hopejr_right_arm` stack

| Existing (`hopejr_right_arm`) | Role | In Desk |
|---|---|---|
| `hopejr_*_description` | URDF / meshes / RViz | copied to `ros2_ws/src/` (for RViz + robot_description) |
| `hopejr_state_publisher` (+ gui) | Qt sliders → `/joint_states` (motor values) | copied; **the web UI replaces the Qt GUI** as the command source |
| `hopejr_real_bridge` | sim (rad) → real (normalized) goals | copied; still usable for the sim→real path |
| `teleop_server.py` | `/arm_goals`,`/hand_goals` → Feetech bus | copied; **Desk's ros backend publishes to it directly** |
| `mujoco/` (`hopejr_arm.xml`, `compare_live.py`) | MuJoCo model + live sim↔real compare | stays in `hopejr_right_arm`; referenced for future Digital Twin |

The ROS2 packages were copied (nested `.git` dirs stripped) so `HopeJR_Desk` is
a **self-contained workspace + its own git repo**. Original upstream:
`github.com/song-jisu/hopejr_*`.

---

## Features

Mapped to [`PLAN.md`](PLAN.md). ✅ implemented · 🟡 partial · ⬜ planned.

### ✅ Dashboard (PLAN §3, §9)
Live robot state over WebSocket (~20 Hz): connection, servo on/off, e-stop,
current task, per-joint **position / velocity / current / temperature**, servos
online (arm 7 / hand 16), comm delay, packet loss, and per-finger state.

### ✅ Robot Control (PLAN §4)
Servo enable/disable, **Home**, joint reset, **Emergency Stop / Recover**,
per-joint jog (normalized −100..100), servo reboot, servo scan (id/model/online).

### ✅ Hand Control (PLAN §7)
- **Grip patterns** by finger digits — `1`=Thumb `2`=Index `3`=Middle `4`=Ring
  `5`=Pinky, so `"15"` = thumb + pinky, `"123"` = thumb + index + middle.
- **Per-finger** aperture sliders (0 open .. 100 closed).
- **Finger state machine**: `idle → closing → holding(contact) → release`.
- **Contact detection** from servo current — a finger is in contact when it is
  *commanded to keep closing but has stalled while drawing high current*
  (current alone false-triggers during fast free motion). Threshold adjustable.

### ✅ Robot Configuration (PLAN §8)
Per-servo command limits (`cmd_min`/`cmd_max`) and `home` offset, editable live;
applied to command clamping and homing.

### ✅ Logs (PLAN §11)
In-memory event/error log (servo, motion, e-stop, config), polled by the UI.

### 🟡 Diagnostics (PLAN §9)
`GET /api/diagnostics` (max temp/current, errors, offline, comm). Live on real
hardware in **serial** mode (reads `Present_Current`/`Load`/`Temperature`); in
**ros** mode it needs a servo-feedback publisher — see
[backend/README.md](backend/README.md) *Feedback TODO*.

### ⬜ Not yet: Motion Teaching (§5), Task Editor (§6), Dataset Recording (§10), Settings (§12)
Planned in `PLAN.md`. The manager/telemetry structure is designed to host them.

---

## Repository layout

```
HopeJR_Desk/
├── PLAN.md              full product plan (Korean)
├── README.md           this file
├── backend/            FastAPI Robot Manager Server  (see backend/README.md)
│   ├── app/            main, manager, hardware, models, backends/, routers/
│   └── requirements.txt
├── frontend/           React + Vite web UI
│   └── src/            App, api, components, pages/{Dashboard,Control,Hand,Config,Logs}
├── ros2_ws/            self-contained ROS2 workspace (copied from hopejr_right_arm)
│   ├── src/            hopejr_{arm,hand,right_arm}_description, state_publisher, real_bridge
│   └── teleop_server.py
└── scripts/            run_backend / run_frontend  (.sh for WSL, .ps1 for Windows)
```

---

## Quick start

Two processes: **backend** (:8000) and **frontend dev server** (:5173, proxies
`/api` and `/ws` to the backend).

### Mock mode — no robot (works on Windows or WSL)

Windows PowerShell:
```powershell
.\scripts\run_backend.ps1      # terminal 1  (uses uv)
.\scripts\run_frontend.ps1     # terminal 2  (npm)
```
WSL / Linux / macOS:
```bash
./scripts/run_backend.sh       # terminal 1
./scripts/run_frontend.sh      # terminal 2
```
Open **http://localhost:5173**. API docs at **http://localhost:8000/docs**.

> ⚠️ **Mock mode never touches hardware** — enabling servo and jogging only
> moves simulated numbers. To move the real robot, use `serial` (or `ros`) mode.

### Serial device names — `/dev/hopejr_arm` / `/dev/hopejr_hand`

The two servo buses are addressed by these fixed names (defaults in
[`backend/app/backends/serial_bus.py`](backend/app/backends/serial_bus.py);
override with `HOPEJR_ARM_PORT` / `HOPEJR_HAND_PORT`):

| Symlink | Servos | Model | Protocol |
|---|---|---|---|
| `/dev/hopejr_arm` | 7 | sm8512bl / sts3250 | 0 |
| `/dev/hopejr_hand` | 16 | scs0009 | 1 |

They are **udev symlinks**, so a bare `/dev/ttyUSB0` never has to be guessed and
the two buses can't swap on reboot. Create them with the helper — plug in the
adapters **one at a time** and run `detect` after each to see which tty is which:

```bash
bash scripts/hopejr_udev.sh detect                          # identify the adapters
bash scripts/hopejr_udev.sh generate /dev/ttyUSB0 /dev/ttyUSB1   # preview the rule
bash scripts/hopejr_udev.sh install  /dev/ttyUSB0 /dev/ttyUSB1   # write + reload (sudo)
```
(argument order is **arm first, hand second**). It writes
`/etc/udev/rules.d/99-hopejr.rules`, reloads udev, and prints the resulting
symlinks. Serial permission is a separate one-time step:

```bash
sudo usermod -aG dialout $USER   # then log out/in (or: newgrp dialout)
```

The rule pins each adapter by its **USB serial number** when it has one. Adapters
without a serial number are indistinguishable to udev, so the rule falls back to
the **USB port path** — those must always go back into the same physical port.
(The CH340 `1a86:7523` boards ship without a serial number; the CH9102 `1a86:55d3`
ones have one.)

**Coexisting with the system's own rules.** Every rule that names a tty uses
`SYMLINK+=` (append), never `SYMLINK=` (assign), so `/dev/hopejr_*` is added
alongside the `/dev/serial/by-id/…` and `by-path` links rather than replacing
them — an adapter already covered by other rules is fine.

The one system service that *does* interfere is **ModemManager**: it treats any
new serial port as a possible modem (`ID_MM_CANDIDATE=1`) and probes it with AT
commands, which can garble the first servo connection. The generated rule sets
`ENV{ID_MM_DEVICE_IGNORE}="1"` to exempt both buses. If a bus still misbehaves
right after plug-in, confirm the exemption took:

```bash
udevadm info -q property -n /dev/hopejr_hand | grep ID_MM   # want ID_MM_DEVICE_IGNORE=1
systemctl status ModemManager                               # or disable it outright
```

`brltty` (braille terminal support) is the other classic thief of CH340 ports.
It is inactive on this machine; if a `/dev/ttyUSB*` vanishes seconds after being
plugged in, that is the usual culprit — `sudo systemctl mask brltty`.

Don't want udev at all? Skip it and pass the real paths instead; the stable
`/dev/serial/by-id/...` names survive reboots on their own:

```bash
HOPEJR_ARM_PORT=/dev/serial/by-id/usb-...-if00 HOPEJR_HAND_PORT=/dev/ttyUSB1 ...
```

### Real robot — `serial` mode (simplest, no ROS2)

Runs on the machine physically wired to the servos (WSL, or a dedicated Linux
box). Uses the **lerobot** environment (here, the `hopejr_right_arm/.venv`).

```bash
# Prerequisites: the symlinks + dialout membership above, plus an env with
# lerobot + fastapi/uvicorn/pydantic (the existing lerobot .venv has both).

# start the backend with that interpreter, pointing at your ports:
cd backend
HOPEJR_BACKEND=serial \
HOPEJR_ARM_PORT=/dev/hopejr_arm HOPEJR_HAND_PORT=/dev/hopejr_hand \
  /path/to/hopejr_right_arm/.venv/bin/python -m uvicorn app.main:app --host 0.0.0.0 --port 8000

# frontend as usual:
./scripts/run_frontend.sh
```

Then in the UI: **Robot Control → Servo Enable → jog / Home / grip**. The
Dashboard shows *real* joint positions, current (arm) / load (hand), and
temperature. If it still won't move, check the two gotchas above (mock mode /
serial permission) and the **Logs** tab (`backend start failed …` means the port
couldn't be opened).

### Real robot — `ros` mode (distributed / reuse existing stack)

```bash
# 1) source ROS2 + build the copied packages (first time)
cd ros2_ws && colcon build --symlink-install && . install/setup.bash

# 2) start the Feetech servo driver (existing tool; needs serial permission too)
python3 teleop_server.py --type arm  --serial /dev/hopejr_arm &
python3 teleop_server.py --type hand --serial /dev/hopejr_hand &

# 3) start Desk backend in ROS mode + the frontend
HOPEJR_BACKEND=ros ./scripts/run_backend.sh
./scripts/run_frontend.sh
```

The RViz + Qt-slider visualization from the original stack still works
independently:
```bash
ros2 launch hopejr_right_arm_description view_robot.launch.py
```

### Requirements
- Backend: Python 3.10+, FastAPI/uvicorn/pydantic/numpy (`backend/requirements.txt`),
  or `uv` — see [Python environment (`.venv`)](#python-environment-venv).
  ROS mode additionally needs a working ROS2 (`rclpy`, `std_msgs`, `sensor_msgs`).
- Frontend: Node 18+ / npm.

---

## Hardware reference

| Bus | Servos | Model | Norm. range |
|---|---|---|---|
| Arm | 7 (`shoulder_pitch/yaw/roll`, `elbow_flex`, `wrist_roll/yaw/pitch`) | sm8512bl / sts3250 | −100..100 |
| Hand | 16 (thumb ×4, index/middle/ring/pinky ×3) | scs0009 | 0..100 |

Fingers: `1` Thumb · `2` Index · `3` Middle · `4` Ring · `5` Pinky.
Single source of truth: [`backend/app/hardware.py`](backend/app/hardware.py)
(derived from `teleop_server.py` and `hopejr_real_bridge/config.py`).

---

## API summary

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/health` | backend mode / readiness |
| GET | `/api/status` | full Telemetry snapshot |
| GET | `/api/robot`, `/api/diagnostics` | robot status / diagnostics |
| WS  | `/ws/telemetry` | Telemetry stream (~20 Hz) |
| POST | `/api/control/servo/enable`\|`disable` | servo power |
| POST | `/api/control/home`\|`reset`\|`estop`\|`recover` | motions / safety |
| POST | `/api/control/joint` `{name,position}` | jog a joint |
| POST | `/api/control/servo/{name}/reboot`, GET `/api/control/scan` | servo ops |
| GET/POST | `/api/hand/finger`\|`grip`\|`contact-threshold`\|`state` | hand control |
| GET/POST | `/api/config`, GET `/api/logs` | configuration / logs |

---

## Roadmap

Next per `PLAN.md`, roughly in order:

1. **Servo feedback node** (real current/temperature) → live Diagnostics + real contact detection.
2. **Motion Teaching** (§5) — stiffness-down + current/position-error force estimate + trajectory record/replay.
3. **Dataset Recording** (§10) — HDF5/CSV/ROS Bag, VLA/imitation-learning ready.
4. **Task Editor** (§6) — drag-and-drop Move/Grasp/Wait/Loop.
5. **3D viewer / Digital Twin** — reuse the URDF + `mujoco/` model in the browser.
