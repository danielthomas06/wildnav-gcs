"""
drone_agent.py — Ground-control server that runs ON the drone (Jetson).
=======================================================================
One instance runs per drone. It:

  1. DISCOVERY (three ways, all active simultaneously):
       - mDNS/Zeroconf service advertisement  (_wildnav._tcp.local.)
       - UDP broadcast beacon on :DISCOVERY_PORT (drones "announce" themselves)
       - manual: the GUI can always be reached directly by typing the drone IP
     The laptop/mobile discovery page listens for both and also lets you
     type an IP.

  2. SERVES THE WEB GUI (static files in ../web) so device X needs zero
     install — just open http://<drone-ip>:8000 in any browser (works on
     phones too).

  3. RUNS SETUP COMMANDS on mission start (the LightGlue clone/pip block),
     streaming their stdout live to the GUI, THEN launches the flight engine.

  4. STREAMS events + telemetry to the GUI over a WebSocket, and exposes a
     STOP endpoint wired to the engine's abort flag.

Run:  python3 drone_agent.py --name drone-alpha
Then open the printed URL on the laptop/phone.

Dependencies (install once on the Jetson, INTO the lightglue conda env):
    conda activate lightglue
    pip install fastapi "uvicorn[standard]" zeroconf python-multipart rasterio pillow
"""

# ─── BOOTSTRAP: re-launch under the lightglue conda env's Python ──────────────
# The flight engine imports torch, cv2, rasterio, lightglue — all of which live
# in the `lightglue` conda env, not base/system Python. If the agent is started
# from the wrong shell (`python3 drone_agent.py` outside the env) it will crash
# at mission time with `No module named 'torch'`. We detect that early and
# re-exec ourselves under the env's Python, preserving CLI args.
def _ensure_env():
    import os, sys
    try:
        import torch  # noqa: F401  — canary; only exists in the lightglue env
        return
    except ImportError:
        pass
    # Locate the env's python. Adjust the search list if your env lives elsewhere.
    for cand in [
        os.path.expanduser("~/miniconda3/envs/lightglue/bin/python3"),
        os.path.expanduser("~/anaconda3/envs/lightglue/bin/python3"),
        "/opt/miniconda3/envs/lightglue/bin/python3",
        "/opt/conda/envs/lightglue/bin/python3",
    ]:
        if os.path.isfile(cand):
            # Guard against exec-loop if the env python is ALSO missing torch.
            if os.path.realpath(cand) == os.path.realpath(sys.executable):
                sys.stderr.write(
                    "ERROR: already running under the env's python but torch "
                    "is missing. Install deps into the lightglue env:\n"
                    "    conda activate lightglue\n"
                    "    pip install fastapi 'uvicorn[standard]' zeroconf "
                    "python-multipart rasterio pillow\n")
                sys.exit(1)
            sys.stderr.write(f"[bootstrap] re-launching under {cand}\n")
            os.execv(cand, [cand] + sys.argv)
    sys.stderr.write(
        "ERROR: lightglue env's python not found. Either activate it manually\n"
        "  conda activate lightglue && python3 drone_agent.py --name ...\n"
        "or edit the search list in drone_agent.py::_ensure_env().\n")
    sys.exit(1)

_ensure_env()
# ── END BOOTSTRAP ────────────────────────────────────────────────────────────

import os, sys, json, time, socket, threading, asyncio, argparse, subprocess, queue
import multiprocessing as mp
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File, Form
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from nav_modes import all_policies, VALID_MODES
from mission_prep import prepare_mission_payload
import event_bus
import cloud_relay

# ── Paths ────────────────────────────────────────────────────────────────────
HERE = Path(__file__).resolve().parent
WEB_DIR = HERE.parent / "web"
UPLOAD_DIR = HERE / "uploads"
UPLOAD_DIR.mkdir(exist_ok=True)

# ── Config ───────────────────────────────────────────────────────────────────
HTTP_PORT       = 8000
DISCOVERY_PORT  = 45454          # UDP broadcast beacon port
BEACON_PERIOD_S = 2.0
SERVICE_TYPE    = "_wildnav._tcp.local."

# The install base is wherever this file lives, one level up (…/wild_opt/server
# -> …/wild_opt). NOTHING is hardcoded to a username or home layout, so the
# same folder works on any machine — install.sh copies it to ~/wild_opt but it
# runs identically from anywhere.
WILDNAV_BASE = str(HERE.parent)

# Setup commands run on mission start. Kept minimal by design: LightGlue
# should be installed ONCE, persistently, into the `lightglue` conda env
# (install.sh handles that) — NOT re-cloned/installed per mission.
#
# We source conda.sh directly from known conda paths instead of using
# `conda info --base`, because a broken pip-installed `conda` shim can shadow
# the real binary and make `conda info --base` return an empty path.
SETUP_SCRIPT = r"""
set -e
CONDA_SH=""
for p in \
    "$HOME/miniconda3/etc/profile.d/conda.sh" \
    "$HOME/anaconda3/etc/profile.d/conda.sh" \
    "/opt/miniconda3/etc/profile.d/conda.sh" \
    "/opt/conda/etc/profile.d/conda.sh"; do
    if [ -f "$p" ]; then CONDA_SH="$p"; break; fi
done
if [ -z "$CONDA_SH" ]; then
    echo "conda.sh not found (looked in ~/miniconda3, ~/anaconda3, /opt/*)"
    exit 1
fi
source "$CONDA_SH"
conda activate lightglue
cd "$WILDNAV_BASE"
echo "SETUP_COMPLETE"
"""

# Working directory where the flight engine runs (the install base).
FLIGHT_CWD = WILDNAV_BASE


def get_lan_ip():
    """Best-effort primary LAN IP (the one on the drone's WiFi)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


# =============================================================================
# DISCOVERY — UDP broadcast beacon + mDNS
# =============================================================================
class DiscoveryBeacon:
    """Broadcasts a small JSON beacon so laptops can auto-find this drone."""
    def __init__(self, drone_name, ip, port):
        self.info = {
            "service": "wildnav-drone",
            "name": drone_name,
            "ip": ip,
            "port": port,
        }
        self.running = False
        self.thread = None
        self.zc = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._beacon_loop, daemon=True)
        self.thread.start()
        self._start_mdns()

    def _beacon_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        payload = json.dumps(self.info).encode()
        while self.running:
            try:
                sock.sendto(payload, ("255.255.255.255", DISCOVERY_PORT))
            except Exception:
                pass
            time.sleep(BEACON_PERIOD_S)
        sock.close()

    def _start_mdns(self):
        try:
            from zeroconf import Zeroconf, ServiceInfo
            self.zc = Zeroconf()
            svc = ServiceInfo(
                SERVICE_TYPE,
                f"{self.info['name']}.{SERVICE_TYPE}",
                addresses=[socket.inet_aton(self.info["ip"])],
                port=self.info["port"],
                properties={"name": self.info["name"]},
            )
            self.zc.register_service(svc)
        except Exception as e:
            print(f"[discovery] mDNS unavailable ({e}); UDP beacon still active")

    def stop(self):
        self.running = False
        if self.zc:
            try: self.zc.close()
            except Exception: pass


# =============================================================================
# MISSION RUNNER — runs setup script, then the flight engine, in a subprocess.
# =============================================================================
class MissionRunner:
    """
    Owns the lifecycle of one mission. Communicates with the flight engine
    (which runs in its own process to isolate the heavy CV/torch work and the
    MAVLink spawn) via an mp.Queue for events/telemetry and an mp.Event for
    abort.
    """
    def __init__(self):
        self.proc = None
        self.event_q = None
        self.abort_flag = None
        self.active = False
        self.last_config = None

    def is_active(self):
        return self.active and self.proc is not None and self.proc.is_alive()

    def start(self, config: dict):
        if self.is_active():
            return False, "a mission is already running"
        self.last_config = config
        self.event_q = mp.Queue(maxsize=1000)
        self.abort_flag = mp.Event()
        # daemon=False is REQUIRED: the flight engine itself spawns a MAVLink
        # subprocess, and Python forbids daemon processes from having children
        # (AssertionError: daemonic processes are not allowed to have children).
        # Non-daemon workers are cleaned up explicitly via shutdown_and_wait()
        # so they don't outlive the agent.
        self.proc = mp.Process(
            target=_mission_worker,
            args=(config, self.event_q, self.abort_flag),
            daemon=False)
        self.proc.start()
        self.active = True
        return True, "mission started"

    def abort(self):
        if self.abort_flag is not None:
            self.abort_flag.set()
        return True

    def shutdown_and_wait(self, timeout=8.0):
        """Signal abort and wait for the worker to exit. Called on agent shutdown
        so a Ctrl-C doesn't leave a mission process running in the background."""
        self.abort()
        if self.proc is not None and self.proc.is_alive():
            self.proc.join(timeout=timeout)
            if self.proc.is_alive():
                # Last resort — engine's own SIGINT handler will run cleanup.
                self.proc.terminate()
                self.proc.join(timeout=3.0)
                if self.proc.is_alive():
                    self.proc.kill()

    def drain_events(self):
        """Non-blocking: pull all pending events/telemetry."""
        out = []
        if self.event_q is None:
            return out
        while True:
            try:
                out.append(self.event_q.get_nowait())
            except Exception:
                break
        if self.proc is not None and not self.proc.is_alive():
            self.active = False
        return out


def _mission_worker(config, event_q, abort_flag):
    """
    Runs in a separate process. First runs the setup shell script (streaming
    its output as events), then constructs and runs the FlightEngine.
    """
    # If cloud_relay installed a stdout/stderr Tee in the parent, this forked
    # child inherited that same object — undo it here so flight_engine's own
    # prints (esp. its high-rate per-frame debug output) go straight to the
    # terminal, uncaptured. setup_log lines and telemetry still reach the
    # cloud via event_q -> event_bus regardless; this only affects raw prints.
    cloud_relay.restore_std_streams()

    def emit(kind, **kw):
        try:
            event_q.put({"kind": kind, "ts": time.time(), **kw})
        except Exception:
            pass

    # ── 1. Setup script ──────────────────────────────────────────────────────
    emit("setup_start")
    try:
        proc = subprocess.Popen(
            ["bash", "-lc", SETUP_SCRIPT],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
            env=dict(os.environ, WILDNAV_BASE=WILDNAV_BASE))
        for line in proc.stdout:
            line = line.rstrip()
            if line:
                emit("setup_log", line=line)
                if abort_flag.is_set():
                    proc.terminate()
                    emit("abort", reason="operator STOP during setup")
                    return
        proc.wait()
        if proc.returncode != 0:
            emit("abort", reason=f"setup script failed (exit {proc.returncode})")
            return
        emit("setup_complete")
    except Exception as e:
        emit("abort", reason=f"setup exception: {e}")
        return

    if abort_flag.is_set():
        emit("abort", reason="operator STOP after setup")
        return

    # ── 2. Flight engine ─────────────────────────────────────────────────────
    # Import here (after setup pip-installed LightGlue) and run in-process.
    try:
        os.chdir(FLIGHT_CWD)
    except Exception as e:
        emit("engine_warn", msg=f"could not chdir to {FLIGHT_CWD}: {e}")

    try:
        from flight_engine import FlightEngine, MissionConfig
    except Exception as e:
        emit("abort", reason=f"failed to import flight_engine: {e}")
        return

    def on_event(e):
        try: event_q.put(e)
        except Exception: pass

    def on_telem(t):
        try: event_q.put({"kind": "telem", **t})
        except Exception: pass

    try:
        cfg = MissionConfig(config)
    except Exception as e:
        emit("abort", reason=f"invalid mission config: {e}")
        return

    engine = FlightEngine(cfg, on_event=on_event, on_telem=on_telem,
                          should_abort=abort_flag.is_set)
    engine.run()


# =============================================================================
# FASTAPI APP
# =============================================================================
app = FastAPI(title="WildNav Drone Agent")
RUNNER = MissionRunner()
DRONE_NAME = "drone"
DRONE_IP = get_lan_ip()
UPLOADED_TIF = None   # server-side path of the georeferenced map for this session

# ── Persistent device config ──────────────────────────────────────────────────
CONFIG_PATH = HERE / "config.json"
SERVICE_NAME = "wildnav-agent"


def load_config():
    try:
        return json.loads(CONFIG_PATH.read_text())
    except Exception:
        return {}


def save_config(cfg: dict):
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2))


def autostart_enabled() -> bool:
    try:
        r = subprocess.run(
            ["sudo", "-n", "systemctl", "is-enabled", SERVICE_NAME],
            capture_output=True, text=True, timeout=5)
        return r.stdout.strip() == "enabled"
    except Exception:
        return False


# ── TRT conversion runner (streams tool output over the same WS) ─────────────
class TRTRunner:
    def __init__(self):
        self.proc = None
        self.thread = None
        self.log_q = []          # drained by the WS loop
        self.lock = threading.Lock()
        self.running = False

    def is_running(self):
        return self.running

    def start(self, width: int, height: int):
        if self.running:
            return False, "a conversion is already running"
        if RUNNER.is_active():
            return False, "cannot convert while a mission is active"
        self.running = True

        def _work():
            script = HERE / "tools" / "step3_convert_tensorrt.py"
            env = dict(os.environ, WILDNAV_BASE=WILDNAV_BASE)
            try:
                self.proc = subprocess.Popen(
                    [sys.executable, str(script),
                     "--width", str(width), "--height", str(height)],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, env=env,
                    cwd=str(HERE / "tools"))
                for line in self.proc.stdout:
                    line = line.rstrip()
                    if line:
                        with self.lock:
                            self.log_q.append(line)
                self.proc.wait()
                with self.lock:
                    self.log_q.append(
                        f"[trt] finished (exit {self.proc.returncode})")
            except Exception as e:
                with self.lock:
                    self.log_q.append(f"[trt] failed: {e}")
            finally:
                self.running = False

        self.thread = threading.Thread(target=_work, daemon=True)
        self.thread.start()
        return True, f"conversion started at {width}x{height}"

    def drain(self):
        with self.lock:
            out, self.log_q = self.log_q, []
        return out


TRT = TRTRunner()


@app.get("/", response_class=HTMLResponse)
def index():
    idx = WEB_DIR / "index.html"
    return HTMLResponse(idx.read_text())


@app.get("/cloud.html", response_class=HTMLResponse)
def cloud_dashboard():
    page = WEB_DIR / "cloud.html"
    return HTMLResponse(page.read_text())


@app.get("/api/info")
def info():
    return {
        "name": DRONE_NAME,
        "ip": DRONE_IP,
        "port": HTTP_PORT,
        "mission_active": RUNNER.is_active(),
        "modes": all_policies(),
        "valid_modes": list(VALID_MODES),
    }


@app.post("/api/upload_map")
async def upload_map(file: UploadFile = File(...)):
    """
    Accept the mission map. This SINGLE artifact is used for BOTH operator
    waypoint-picking AND the drone's LightGlue localisation, so it must be a
    georeferenced GeoTIFF (CRS + affine transform baked in). We open it, verify
    the geotags, and return the footprint corners reprojected to WGS84 so the
    GUI can map canvas pixels <-> lat/lon exactly as the drone will.

    On any problem (not a TIF, no CRS, no transform) we reject with a message
    the GUI shows inline; no mission can launch without a valid map.
    """
    global UPLOADED_TIF
    dest = UPLOAD_DIR / file.filename
    data = await file.read()
    dest.write_bytes(data)
    result = _validate_and_activate_map(dest, file.filename)
    if not result["ok"]:
        # A fresh bad upload gets dropped and clears the active map — unlike
        # _do_select_map re-validating an already-known-good pre-staged file,
        # here there's nothing worth keeping around.
        try: dest.unlink()
        except Exception: pass
        UPLOADED_TIF = None
        return JSONResponse(result, status_code=400)
    _publish_maps_list()
    return result


def _validate_and_activate_map(dest: Path, orig_filename: str) -> dict:
    """Shared by /api/upload_map (a just-written file) and _do_select_map
    (an already-pre-staged file) — same rasterio validation, same
    UPLOADED_TIF activation, same response shape either way."""
    global UPLOADED_TIF

    try:
        import rasterio
        from rasterio.warp import transform_bounds
        from rasterio.enums import Resampling
        import numpy as np
    except Exception as e:
        return {"ok": False, "error": f"server missing rasterio/numpy: {e}"}

    preview_url = None
    try:
        with rasterio.open(str(dest)) as src:
            if src.crs is None:
                raise ValueError("no CRS — not a georeferenced GeoTIFF")
            if src.transform is None or src.transform.is_identity:
                raise ValueError("no geotransform — not georeferenced")
            # Reproject the dataset bounds to WGS84 (lat/lon) for the GUI.
            left, bottom, right, top = src.bounds
            w, s, e, n = transform_bounds(src.crs, "EPSG:4326",
                                          left, bottom, right, top,
                                          densify_pts=21)
            width_px, height_px = src.width, src.height
            crs_str = str(src.crs)

            # ── Render a downsampled RGB PNG for in-browser display ──────────
            # The browser can't paint GeoTIFF pixels; we rasterise here (where
            # rasterio lives) to a web image. Display-only — the drone still
            # matches the full-resolution TIF. North-up raster => PNG row 0 is
            # north, matching the tlLat=n corner mapping, so it overlays the
            # graticule directly.
            MAX_EDGE = 1600
            scale = min(1.0, MAX_EDGE / max(width_px, height_px))
            out_w = max(1, int(width_px * scale))
            out_h = max(1, int(height_px * scale))
            n_bands = src.count
            read_bands = [1, 2, 3] if n_bands >= 3 else [1]
            arr = src.read(read_bands,
                           out_shape=(len(read_bands), out_h, out_w),
                           resampling=Resampling.bilinear)
            if len(read_bands) == 1:
                arr = np.repeat(arr, 3, axis=0)   # grayscale -> RGB
            img = np.transpose(arr, (1, 2, 0))     # (H,W,3)
            # Normalise to 8-bit if needed (handles uint16 / float rasters).
            if img.dtype != np.uint8:
                fmax = float(np.nanmax(img)) or 1.0
                fmin = float(np.nanmin(img))
                rng = (fmax - fmin) or 1.0
                img = ((img - fmin) / rng * 255.0).clip(0, 255).astype(np.uint8)
            try:
                from PIL import Image
                preview_name = dest.stem + "_preview.png"
                Image.fromarray(img, "RGB").save(str(UPLOAD_DIR / preview_name))
                preview_url = f"/uploads/{preview_name}"
            except Exception as pe:
                # PNG preview is optional; picking still works via the graticule.
                preview_url = None
    except ValueError as ex:
        # Invalid file — validation-only here, doesn't touch UPLOADED_TIF or
        # delete anything; the caller decides what "bad" means for its own
        # context (a fresh upload vs. re-validating an already-stored file).
        return {"ok": False,
                "error": f"invalid map: {ex}. Upload a georeferenced GeoTIFF."}
    except Exception as ex:
        return {"ok": False, "error": f"could not read map: {ex}"}

    UPLOADED_TIF = str(dest)
    # Corner mapping for the GUI: top-left = (n, w), bottom-right = (s, e).
    return {
        "ok": True,
        "path": str(dest),
        "filename": orig_filename,
        "size": dest.stat().st_size,
        "crs": crs_str,
        "width_px": width_px,
        "height_px": height_px,
        "bounds": {"tlLat": n, "tlLon": w, "brLat": s, "brLon": e},
        "preview_url": preview_url,
    }


def _do_select_map(filename: str):
    """Activate an already-uploaded (pre-staged over local WiFi) GeoTIFF by
    filename, without re-uploading it — the cloud command version of picking
    a map. Same validation as a fresh upload; on failure, the previously
    active map (if any) is left untouched rather than cleared."""
    if not filename:
        return False, "no filename provided"
    dest = UPLOAD_DIR / filename
    if not dest.is_file():
        return False, f"no such pre-staged map: {filename}"
    result = _validate_and_activate_map(dest, filename)
    if not result["ok"]:
        return False, result["error"]
    _publish_maps_list()
    return True, f"active map set to {filename}"


def _publish_maps_list():
    """Retained topic (see cloud_relay.py) so a dashboard connecting after
    the fact immediately sees what's pre-staged, same pattern as status/
    lwt/mission — not a live-only event."""
    files = sorted(p.name for p in UPLOAD_DIR.glob("*.tif")) + \
            sorted(p.name for p in UPLOAD_DIR.glob("*.tiff"))
    active = Path(UPLOADED_TIF).name if UPLOADED_TIF else None
    event_bus.bus.publish({
        "kind": "maps_list", "files": files, "active": active, "ts": time.time(),
    })


@app.post("/api/start")
async def start(payload: dict):
    """
    Payload from the GUI:
      {
        "waypoints": [[lat,lon],...],   # first = takeoff/approx-start location,
                                         # rest = destinations (in order) —
                                         # see mission_prep.py
        "mode": "safety"|"cold_start"|"safe_start"|"full_gps_denied",
        "takeoff_alt": 120.0,
        "nav_speed": 2.5,
        "min_valid_alt_m": 90.0,
        "approx_start": [lat,lon] | null   # explicit override; usually omitted
      }
    The map is NOT in the payload — it's the GeoTIFF uploaded via /api/upload_map,
    whose server-side path we inject here as tif_path. No map -> no launch.
    """
    ok, msg = _do_start_mission(payload)
    if not ok:
        return JSONResponse({"ok": False, "error": msg}, status_code=400)
    return {"ok": True, "message": msg}


def _do_start_mission(payload: dict):
    """Shared by /api/start (local WiFi) and the cloud cmd handler (see
    cloud_relay.py) — same validation, same RUNNER, same mission_config
    broadcast, regardless of which transport the command arrived on.
    Returns (ok, message_or_error)."""
    if UPLOADED_TIF is None:
        return False, "no map uploaded — upload a GeoTIFF first"
    if not payload.get("waypoints"):
        return False, "no waypoints provided"
    if payload.get("mode") not in VALID_MODES:
        return False, f"mode must be one of {VALID_MODES}"
    try:
        payload = prepare_mission_payload(payload)
    except ValueError as e:
        return False, str(e)
    # Inject the uploaded map as the localisation TIF.
    payload["tif_path"] = UPLOADED_TIF
    ok, msg = RUNNER.start(payload)
    if ok:
        # Waypoints never otherwise leave this endpoint — flight_engine only
        # emits live position telemetry, not the plan it's flying against.
        # Goes through the same bus as everything else so cloud_relay can
        # also retain-publish it (see cloud_relay.py) for the map view.
        event_bus.bus.publish({
            "kind": "mission_config",
            "waypoints": payload["waypoints"],
            "mode": payload["mode"],
            "takeoff_alt": payload.get("takeoff_alt"),
            "ts": time.time(),
        })
    return ok, msg


@app.post("/api/stop")
async def stop():
    ok, msg = _do_stop_mission()
    return {"ok": ok, "message": msg}


def _do_stop_mission():
    """Shared by /api/stop and the cloud cmd handler."""
    RUNNER.abort()
    return True, "abort signal sent -> LOITER/LAND"


def _handle_cloud_cmd(action: str, params: dict):
    """Callback passed into cloud_relay.relay.start() — invoked when a
    wildnav/<id>/cmd message arrives. Same RUNNER calls as the local HTTP
    API, just a different transport in. Returns (ok, message)."""
    if action == "start_mission":
        return _do_start_mission(params or {})
    if action == "stop":
        return _do_stop_mission()
    if action == "select_map":
        return _do_select_map((params or {}).get("filename"))
    return False, f"unknown action: {action}"


@app.get("/api/cloud_status")
def cloud_status():
    """No secrets — just enough for the local UI to gate the cloud-mode
    toggle on cloud_relay actually being connected right now."""
    return {
        "enabled": bool(os.environ.get("WILDNAV_MQTT_HOST")),
        "connected": cloud_relay.relay.is_connected(),
    }


@app.get("/api/cloud_config")
def cloud_config():
    """Host/port only — never credentials. Lets the local UI pre-fill the
    broker connect form instead of the operator retyping the host every
    time; they still enter their own operator credential client-side."""
    return {
        "host": os.environ.get("WILDNAV_MQTT_HOST", ""),
        "ws_port": os.environ.get("WILDNAV_MQTT_WS_PORT", "8884"),
        "path": os.environ.get("WILDNAV_MQTT_WS_PATH", "/mqtt"),
        "drone_id": os.environ.get("WILDNAV_DRONE_ID", DRONE_NAME),
    }


# ── Device settings ───────────────────────────────────────────────────────────
@app.get("/api/settings")
def get_settings():
    trt_path = Path(WILDNAV_BASE) / "weights" / "superpoint_trt.pth"
    return {
        "name": DRONE_NAME,
        "autostart": autostart_enabled(),
        "trt_exists": trt_path.exists(),
        "trt_running": TRT.is_running(),
        "mission_active": RUNNER.is_active(),
    }


@app.post("/api/settings")
async def set_settings(payload: dict):
    """Persist device settings. Currently: drone_name.
    Name is used at agent startup (beacon + mDNS), so a rename takes effect
    after the agent restarts (or immediately for the HTTP-visible name)."""
    global DRONE_NAME
    name = (payload.get("name") or "").strip()
    if not name:
        return JSONResponse({"ok": False, "error": "empty name"}, status_code=400)
    if not all(c.isalnum() or c in "-_" for c in name):
        return JSONResponse(
            {"ok": False, "error": "name must be letters/digits/-/_ only"},
            status_code=400)
    cfg = load_config()
    cfg["drone_name"] = name
    save_config(cfg)
    DRONE_NAME = name
    return {"ok": True, "name": name,
            "note": "beacon/mDNS pick up the new name on next agent restart"}


@app.post("/api/convert_trt")
async def convert_trt(payload: dict):
    """Re-run SuperPoint TensorRT conversion at a custom resolution.
    Output streams to the GUI via the websocket as 'tool_log' events."""
    try:
        width = int(payload.get("width", 1280))
        height = int(payload.get("height", 720))
    except Exception:
        return JSONResponse({"ok": False, "error": "width/height must be ints"},
                            status_code=400)
    if not (64 <= width <= 4096 and 64 <= height <= 4096):
        return JSONResponse({"ok": False, "error": "resolution out of range"},
                            status_code=400)
    ok, msg = TRT.start(width, height)
    status = 200 if ok else 409
    return JSONResponse({"ok": ok, "message": msg}, status_code=status)


@app.post("/api/autostart")
async def set_autostart(payload: dict):
    """Enable/disable launching the agent automatically at boot (systemd).
    Requires the sudoers rule installed by install.sh."""
    enable = bool(payload.get("enabled"))
    action = "enable" if enable else "disable"
    try:
        r = subprocess.run(
            ["sudo", "-n", "systemctl", action, SERVICE_NAME],
            capture_output=True, text=True, timeout=10)
        if r.returncode != 0:
            return JSONResponse(
                {"ok": False,
                 "error": f"systemctl {action} failed: {r.stderr.strip() or 'run install.sh to set up the service + sudoers rule'}"},
                status_code=500)
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
    return {"ok": True, "autostart": enable}


# =============================================================================
# EVENT PUMP — the ONE place that drains RUNNER/TRT, fanning out to the bus.
# Both the local WebSocket and cloud_relay subscribe to the bus instead of
# draining these single-consumer sources directly, so neither one starves
# the other (see event_bus.py).
# =============================================================================
async def _event_pump():
    while True:
        for e in RUNNER.drain_events():
            event_bus.bus.publish(e)
        for line in TRT.drain():
            event_bus.bus.publish({"kind": "tool_log", "line": line, "ts": time.time()})
        event_bus.bus.publish({
            "kind": "heartbeat",
            "mission_active": RUNNER.is_active(),
            "trt_running": TRT.is_running(),
            "ts": time.time(),
        })
        await asyncio.sleep(0.1)


@app.on_event("startup")
async def _on_startup():
    asyncio.create_task(_event_pump())
    # No-ops unless WILDNAV_MQTT_HOST is set — see cloud_relay.py.
    cloud_relay.relay.start(DRONE_NAME, on_cmd=_handle_cloud_cmd)
    # So a dashboard connecting right after a restart sees pre-staged maps
    # immediately (retained topic), not just ones uploaded during this run.
    _publish_maps_list()


@app.on_event("shutdown")
async def _on_shutdown():
    cloud_relay.relay.stop()


@app.websocket("/ws")
async def ws(websocket: WebSocket):
    """Streams engine events + telemetry to the GUI (via the shared event bus)."""
    await websocket.accept()
    sub_q = event_bus.bus.subscribe()
    try:
        while True:
            try:
                e = sub_q.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.1)
                continue
            await websocket.send_json(e)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        event_bus.bus.unsubscribe(sub_q)


# Serve uploaded maps and static assets.
app.mount("/uploads", StaticFiles(directory=str(UPLOAD_DIR)), name="uploads")
app.mount("/static", StaticFiles(directory=str(WEB_DIR / "static")),
          name="static")


# =============================================================================
# MAIN
# =============================================================================
def main():
    global DRONE_NAME, DRONE_IP
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=None,
                    help="override the drone name (else config.json, else hostname)")
    ap.add_argument("--port", type=int, default=HTTP_PORT)
    args = ap.parse_args()

    # Name priority: explicit --name > config.json > hostname.
    # config.json is the persistent home for the name — set it once (via the
    # GUI settings panel or install.sh) and never pass --name again.
    DRONE_NAME = (args.name
                  or load_config().get("drone_name")
                  or socket.gethostname())
    DRONE_IP = get_lan_ip()

    beacon = DiscoveryBeacon(DRONE_NAME, DRONE_IP, args.port)
    beacon.start()

    print("=" * 64)
    print(f"  WildNav Drone Agent  —  '{DRONE_NAME}'")
    print("=" * 64)
    print(f"  Open this on your laptop or phone (same WiFi):")
    print(f"      http://{DRONE_IP}:{args.port}")
    print(f"  Discovery beacon: UDP :{DISCOVERY_PORT}  +  mDNS {SERVICE_TYPE}")
    print("=" * 64)

    try:
        uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")
    finally:
        beacon.stop()
        # If a mission was in flight when the agent was Ctrl-C'd, signal abort
        # and wait for the worker to exit so we don't leak processes.
        try:
            RUNNER.shutdown_and_wait()
        except Exception:
            pass


if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    main()
