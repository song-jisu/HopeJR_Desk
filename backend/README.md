# HopeJR Desk — Backend (Robot Manager Server)

FastAPI service that exposes the robot as a REST + WebSocket API. It owns a
**backend** (mock simulator or real ROS2) behind a single `RobotManager`, so the
same API and UI work with or without hardware.

```
app/
├── main.py            FastAPI app + lifespan (starts the manager)
├── manager.py         RobotManager: telemetry loop, finger state machines, config, logs
├── hardware.py        Motor topology / IDs / limits (from teleop_server.py + real_bridge)
├── models.py          Pydantic API schemas
├── backends/
│   ├── base.py        RobotBackend interface
│   ├── mock.py        Servo simulator (default; no robot needed)
│   ├── serial_bus.py  Real robot via lerobot FeetechMotorsBus (no ROS2) + live feedback
│   └── ros.py         Real robot via ROS2 (publishes /arm_goals, /hand_goals)
└── routers/
    ├── dashboard.py   GET /api/status, /api/robot, /api/diagnostics
    ├── control.py     servo enable/disable, home, reset, estop, recover, jog, reboot, scan
    ├── hand.py        finger command, grip patterns, contact threshold, hand state
    ├── config.py      per-servo limits/home + logs
    └── ws.py          /ws/telemetry  (Telemetry snapshot each tick)
```

## Run

```bash
# Mock (no robot) — dev machine
HOPEJR_BACKEND=mock uvicorn app.main:app --reload --port 8000

# Real robot, direct serial (no ROS2) — run with the lerobot interpreter
HOPEJR_BACKEND=serial /path/to/.venv/bin/python -m uvicorn app.main:app --port 8000

# Real robot via ROS2 — ROS2 sourced, teleop_server.py running
HOPEJR_BACKEND=ros uvicorn app.main:app --port 8000
```

Env vars: `HOPEJR_BACKEND` (`mock`|`serial`|`ros`), `HOPEJR_TELEMETRY_HZ`
(default 20), `HOPEJR_CONTACT_MA` (default 150), `HOPEJR_ARM_PORT`
(`/dev/hopejr_arm`), `HOPEJR_HAND_PORT` (`/dev/hopejr_hand`).

Interactive API docs at `http://localhost:8000/docs`.

## Backends

| | mock | serial | ros |
|---|---|---|---|
| Positions | simulated | **live** `Present_Position` | echoed cmd + `/joint_states` if published |
| Current / Temp | simulated (∝ error) | **live** (arm `Present_Current`, hand `Present_Load`) | **0** — not published by current stack |
| Commands | in-memory | lerobot `sync_write("Goal_Position")` | `Float64MultiArray` → `/arm_goals`,`/hand_goals` → `teleop_server.py` |
| Needs | — | lerobot env + serial perms | ROS2 + teleop_server |

Notes:
- **serial** opens the buses itself (proto 0 arm / 1 hand), like `teleop_server.py`.
  It reads feedback in a worker thread and only writes goals / energizes torque
  after **Servo Enable**. scs0009 (hand) has no current register, so
  `Present_Load` stands in for hand contact current.
- **ros** Feedback TODO: `teleop_server.py` only *writes* goals. For live
  Diagnostics/contact over ROS, add a node that publishes `Present_*` from the
  bus (e.g. `/servo_feedback`) and read it in `backends/ros.py`.
