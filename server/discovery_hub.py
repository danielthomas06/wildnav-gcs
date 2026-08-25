"""
discovery_hub.py — Runs on the LAPTOP (device X). Optional but recommended.
==========================================================================
Purpose: auto-discover every WildNav drone on the WiFi and present a picker.

You do NOT strictly need this — each drone already serves its own GUI at
http://<drone-ip>:8000, and the in-GUI discovery page (served by any drone)
also sniffs beacons. But if you want a single "which drone?" landing page on
the laptop without knowing any IP first, run this:

    python3 discovery_hub.py
    # then open http://localhost:8080

It listens for the UDP beacons every drone_agent broadcasts, lists them live
with their IPs, and lets you click through to a drone's GUI — or type an IP
manually if a drone doesn't appear (e.g. broadcast blocked by AP isolation).

Dependencies: pip install fastapi "uvicorn[standard]"
"""

import json, socket, threading, time, asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

DISCOVERY_PORT = 45454
HUB_PORT       = 8080
STALE_AFTER_S  = 8.0     # drop a drone from the list if unheard this long

_drones = {}             # ip -> {name, ip, port, last_seen}
_lock = threading.Lock()


def _listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except Exception:
        pass
    sock.bind(("", DISCOVERY_PORT))
    while True:
        try:
            data, addr = sock.recvfrom(2048)
            info = json.loads(data.decode())
            if info.get("service") == "wildnav-drone":
                with _lock:
                    _drones[info["ip"]] = {
                        "name": info.get("name", "drone"),
                        "ip": info["ip"],
                        "port": info.get("port", 8000),
                        "last_seen": time.time(),
                    }
        except Exception:
            continue


def _active_drones():
    now = time.time()
    with _lock:
        fresh = {ip: d for ip, d in _drones.items()
                 if now - d["last_seen"] < STALE_AFTER_S}
        # prune
        for ip in list(_drones):
            if now - _drones[ip]["last_seen"] >= STALE_AFTER_S:
                del _drones[ip]
    return list(fresh.values())


app = FastAPI(title="WildNav Discovery Hub")

_LANDING = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>WildNav — Ground Control</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@500;600;700&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
<style>
  :root{--void:#080b0f;--panel:#0e141b;--panel2:#131b24;--edge:#1e2833;
    --edge2:#2a3745;--ink:#dfe7ee;--mute:#6b7885;--phosphor:#35e08a;
    --azure:#3d9bff;--disp:'Space Grotesk',system-ui,sans-serif;
    --mono:'JetBrains Mono',ui-monospace,monospace;}
  *{box-sizing:border-box}
  body{margin:0;background:radial-gradient(1200px 600px at 80% -10%,#0d1720 0%,transparent 60%),var(--void);
    color:var(--ink);font-family:var(--disp);-webkit-font-smoothing:antialiased;min-height:100vh}
  .wrap{max-width:640px;margin:0 auto;padding:20px}
  .top{display:flex;align-items:center;justify-content:space-between;
    padding:20px 4px 22px;border-bottom:1px solid var(--edge);margin-bottom:28px}
  .brand{display:flex;align-items:center;gap:14px}
  .glyph{font-size:30px;color:var(--phosphor);
    text-shadow:0 0 18px rgba(53,224,138,.5);line-height:1}
  .bn{font-weight:700;letter-spacing:.28em;font-size:16px}
  .bs{font-family:var(--mono);font-size:11px;color:var(--mute);
    letter-spacing:.04em;margin-top:2px}
  .scan{display:flex;align-items:center;gap:8px;font-family:var(--mono);
    font-size:12px;color:var(--mute)}
  .scan-dot{width:8px;height:8px;border-radius:50%;background:var(--phosphor);
    box-shadow:0 0 10px var(--phosphor);animation:pulse 1.4s ease infinite}
  @keyframes pulse{50%{opacity:.4}}
  .head{display:flex;align-items:flex-start;gap:18px;margin-bottom:22px}
  .num{font-family:var(--mono);font-size:13px;font-weight:700;color:var(--void);
    background:var(--phosphor);border-radius:7px;padding:8px 11px;line-height:1;
    min-width:38px;text-align:center;box-shadow:0 0 20px rgba(53,224,138,.25)}
  h2{margin:0;font-size:22px;font-weight:600;letter-spacing:.01em}
  .sub{margin:5px 0 0;color:var(--mute);font-size:13.5px}
  .drone{display:flex;align-items:center;justify-content:space-between;
    background:var(--panel);border:1px solid var(--edge);border-radius:12px;
    padding:16px 18px;margin-bottom:10px;text-decoration:none;color:inherit;
    cursor:pointer;transition:.18s}
  .drone:hover{border-color:var(--edge2);background:var(--panel2);
    transform:translateX(3px)}
  .left{display:flex;align-items:center;gap:13px}
  .live-pip{width:9px;height:9px;border-radius:50%;background:var(--phosphor);
    box-shadow:0 0 10px var(--phosphor)}
  .name{font-weight:600;font-size:15px}
  .ip{font-family:var(--mono);font-size:12px;color:var(--mute);margin-top:2px}
  .arrow{color:var(--azure);font-family:var(--mono);font-size:13px}
  .empty{color:var(--mute);font-family:var(--mono);font-size:13px;
    padding:30px;text-align:center;border:1px dashed var(--edge);border-radius:12px}
</style></head><body><div class=wrap>
<div class=top>
  <div class=brand>
    <span class=glyph>◎</span>
    <div><div class=bn>WILDNAV</div>
    <div class=bs>ground control · discovery</div></div>
  </div>
  <div class=scan><span class="scan-dot"></span><span>scanning</span></div>
</div>
<div class=head>
  <div class=num>01</div>
  <div><h2>Select drone</h2>
  <p class=sub>All drones broadcasting on this WiFi appear below.</p></div>
</div>
<div id=list><div class=empty>Scanning for drones…</div></div>
</div>
<script>
const ws=new WebSocket('ws://'+location.host+'/ws');
ws.onmessage=e=>{const d=JSON.parse(e.data);const l=document.getElementById('list');
  if(!d.drones.length){l.innerHTML='<div class=empty>No drones found yet. '
    +'Make sure a drone agent is running and you\\'re on the same WiFi.</div>';return;}
  l.innerHTML=d.drones.map(x=>`<a class=drone href="http://${x.ip}:${x.port}">
    <div class=left><span class=live-pip></span>
      <div><div class=name>${x.name}</div><div class=ip>${x.ip}:${x.port}</div></div></div>
    <span class=arrow>Open →</span></a>`).join('');};
ws.onclose=()=>{document.querySelector('.scan-dot').style.background='#ff5a52';
  document.querySelector('.scan-dot').style.boxShadow='0 0 10px #ff5a52';
  document.querySelector('.scan span:last-child').textContent='disconnected';};
</script></body></html>"""


@app.get("/", response_class=HTMLResponse)
def landing():
    return HTMLResponse(_LANDING)


@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json({"drones": _active_drones()})
            await asyncio.sleep(1.0)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


def main():
    threading.Thread(target=_listener, daemon=True).start()
    print("=" * 60)
    print("  WildNav Discovery Hub")
    print("=" * 60)
    print(f"  Open on this laptop:  http://localhost:{HUB_PORT}")
    print(f"  Listening for drone beacons on UDP :{DISCOVERY_PORT}")
    print("=" * 60)
    uvicorn.run(app, host="0.0.0.0", port=HUB_PORT, log_level="warning")


if __name__ == "__main__":
    main()
