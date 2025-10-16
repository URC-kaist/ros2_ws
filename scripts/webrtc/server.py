# server.py
import asyncio
from pathlib import Path
from aiohttp import web

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

clients = {"sender": None, "viewer": None}
pending = {"sender": [], "viewer": []}

async def index(_):
    return web.FileResponse(STATIC_DIR / "index.html")

async def ws_handler(request):
    role = request.query.get("role")
    if role not in ("sender", "viewer"):
        return web.Response(status=400, text="role must be sender|viewer")

    ws = web.WebSocketResponse(heartbeat=30)
    await ws.prepare(request)
    clients[role] = ws
    print(f"[server] {role} connected")

    # flush pending messages destined for this role
    queued = pending[role]
    pending[role] = []
    for payload in queued:
        await ws.send_str(payload)

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                other = "viewer" if role == "sender" else "sender"
                target = clients.get(other)
                print(f"[server] {role} -> {other}: {msg.data[:80]}{'...' if len(msg.data) > 80 else ''}")
                if target is not None:
                    await target.send_str(msg.data)
                else:
                    # stash until the peer joins
                    pending[other].append(msg.data)
            elif msg.type == web.WSMsgType.ERROR:
                print(f"[server] WS error from {role}: {ws.exception()}")
    finally:
        if clients.get(role) is ws:
            clients[role] = None
        print(f"[server] {role} disconnected")

    return ws

app = web.Application()
app.router.add_get("/", index)
app.router.add_get("/ws", ws_handler)
app.router.add_static("/static", STATIC_DIR)

if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=8080)
