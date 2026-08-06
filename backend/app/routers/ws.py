"""WebSocket telemetry stream — pushes a Telemetry snapshot each tick."""
from __future__ import annotations

import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from ..manager import get_manager

router = APIRouter(tags=["ws"])


@router.websocket("/ws/telemetry")
async def telemetry_ws(ws: WebSocket):
    await ws.accept()
    mgr = get_manager()
    q = mgr.subscribe()
    try:
        # send the latest immediately so the client isn't blank on connect
        if mgr.latest is not None:
            await ws.send_text(mgr.latest.model_dump_json())
        while True:
            snap = await q.get()
            await ws.send_text(snap.model_dump_json())
    except WebSocketDisconnect:
        pass
    except asyncio.CancelledError:
        pass
    finally:
        mgr.unsubscribe(q)
