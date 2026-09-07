"""HopeJR Desk backend — FastAPI 'Robot Manager Server'.

Run:
    HOPEJR_BACKEND=mock uvicorn app.main:app --reload --port 8000   # dev (no robot)
    HOPEJR_BACKEND=ros  uvicorn app.main:app --port 8000            # on WSL with ROS2
"""
from __future__ import annotations

import os
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .manager import init_manager, get_manager
from .routers import (calibration, config, control, dashboard, hand,
                      identify, links, teaching, ws)


@asynccontextmanager
async def lifespan(app: FastAPI):
    mgr = init_manager(os.environ.get("HOPEJR_BACKEND", "mock"))
    await mgr.start()
    try:
        yield
    finally:
        await mgr.stop()


app = FastAPI(title="HopeJR Desk", version="0.1.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],           # dev; tighten for deployment
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(dashboard.router)
app.include_router(control.router)
app.include_router(hand.router)
app.include_router(config.router)
app.include_router(calibration.router)
app.include_router(teaching.router)
app.include_router(identify.router)
app.include_router(links.router)
app.include_router(ws.router)


# Serve the ROS2 description packages (URDF + meshes) for the 3D viewer.
# package://<pkg>/... resolves to /assets/<pkg>/... (urdf-loader packages map).
_ASSETS = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "ros2_ws", "src"))
if os.path.isdir(_ASSETS):
    app.mount("/assets", StaticFiles(directory=_ASSETS), name="assets")


@app.get("/api/health")
def health():
    m = get_manager()
    return {"ok": True, "backend": m.backend.mode, "telemetry_ready": m.latest is not None}
