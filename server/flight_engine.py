"""
flight_engine.py — Multi-waypoint, mode-aware GNSS-denied navigation engine.
============================================================================
Refactor of navigate_north.py. Key differences from the original:

  1. MULTI-WAYPOINT: flies through an ordered list of (lat,lon) waypoints
     instead of a single hardcoded POINT_B. Legs are flown sequentially;
     on arrival at waypoint i, the navigator retargets to waypoint i+1.

  2. VERTICAL-FIRST TAKEOFF + FIRST-FIX ACQUISITION:
     The drone climbs vertically to takeoff altitude BEFORE any horizontal
     motion. Waypoint_1 (the physical start location) is then obtained by:
        - GPS         (modes: safety, safe_start)   -> seed EKF from GPS
        - FIRST VISUAL FIX (modes: cold_start, full_gps_denied)
                      -> climb vertical, localise against TIF at altitude,
                         that fix IS waypoint_1. Optional approx-start pin
                         narrows the search; else full-map tile search.

  3. MODE MATRIX (see nav_modes.py): seed_from_gps x gps_correction.
     gps_correction=False disables GPS_RESET / sustained-bias snap /
     GPS-arrival backstop entirely (pure vision).

  4. MIN VALID ALTITUDE FLOOR: any localisation below min_valid_alt_m is
     discarded as invalid (default 90 m, GUI-settable). This guarantees the
     first-fix-at-altitude logic can't latch onto a low-altitude match during
     the vertical climb.

  5. NO stdin prompts. Driven entirely by a MissionConfig object and
     controlled/observed via callbacks (the agent wires these to WebSocket).

This module is imported and run by drone_agent.py. It is NOT meant to be run
standalone (though a __main__ demo harness is provided at the bottom).

All the heavy lifting — SIYI gimbal, LightGlue localisation, NavigationEKF,
MAVLink subprocess, camera thread — is carried over from navigate_north.py
with minimal changes. Search for "CHANGED:" comments to see the deltas.
"""

import sys, os, math, warnings, time, threading, datetime, signal, glob
import subprocess, struct, socket, queue, json, traceback
warnings.filterwarnings("ignore")

import multiprocessing as mp
import numpy as np
import cv2
import torch
import rasterio
from rasterio.transform import rowcol, xy
from pyproj import Transformer
import pandas as pd

from lightglue import LightGlue, SuperPoint
from lightglue.utils import rbd

from nav_modes import get_policy


# =============================================================================
# MISSION CONFIG — replaces the USER CONFIG block. Populated from the GUI.
# =============================================================================
class MissionConfig:
    """Everything the GUI sends to define a mission."""
    def __init__(self, d: dict):
        self.mode = d.get("mode", "safety")
        self.policy = get_policy(self.mode)

        self.takeoff_alt = float(d.get("takeoff_alt", 120.0))
        self.nav_speed = float(d.get("nav_speed", 2.5))   # default / fallback speed
        self.min_valid_alt_m = float(d.get("min_valid_alt_m", 90.0))

        # Ordered waypoints. Each waypoint carries the params for the leg that
        # ARRIVES at it. Accepts two shapes:
        #   - legacy: [lat, lon]
        #   - full:   {"lat":.., "lon":.., "speed":.., "alt":..}
        # 'alt' is the altitude to fly the arriving leg at (step-change at the
        # PREVIOUS waypoint, then level). The FIRST leg's altitude is always
        # locked to takeoff_alt for safe first-fix localisation, so any 'alt' on
        # waypoint[0] is ignored. Missing speed/alt fall back to nav_speed /
        # takeoff_alt.
        self.waypoints = []          # [(lat, lon)]
        self.leg_speed = []          # per-waypoint arriving-leg speed
        self.leg_alt = []            # per-waypoint arriving-leg altitude
        for i, wp in enumerate(d["waypoints"]):
            if isinstance(wp, dict):
                lat, lon = float(wp["lat"]), float(wp["lon"])
                spd = float(wp.get("speed", self.nav_speed))
                alt = float(wp.get("alt", self.takeoff_alt))
            else:
                lat, lon = float(wp[0]), float(wp[1])
                spd, alt = self.nav_speed, self.takeoff_alt
            # First leg altitude is locked to takeoff altitude.
            if i == 0:
                alt = self.takeoff_alt
            self.waypoints.append((lat, lon))
            self.leg_speed.append(spd)
            self.leg_alt.append(alt)

        # Optional approx-start pin (lat,lon) — search prior for first visual
        # fix in GPS-denied seed modes. None -> full-map tile search.
        aps = d.get("approx_start", None)
        self.approx_start = tuple(aps) if aps else None

        # Georeferenced GeoTIFF the drone localises against. This is the SAME
        # map the operator uploaded and picked waypoints on — injected by the
        # agent as the uploaded file's path. Required; there is no default.
        self.tif_path = d.get("tif_path")
        if not self.tif_path:
            raise ValueError("tif_path is required (uploaded GeoTIFF map)")

        # Safety / tuning knobs (sane defaults from navigate_north.py)
        self.cte_threshold = float(d.get("cte_threshold", 8.0))
        self.arrival_radius = float(d.get("arrival_radius", 7.0))
        self.gps_fallback_time = float(d.get("gps_fallback_time", 7.0))
        self.lock_yaw_north = bool(d.get("lock_yaw_north", True))

    def to_dict(self):
        return {
            "waypoints": [
                {"lat": w[0], "lon": w[1], "speed": s, "alt": a}
                for w, s, a in zip(self.waypoints, self.leg_speed, self.leg_alt)
            ],
            "mode": self.mode,
            "takeoff_alt": self.takeoff_alt,
            "nav_speed": self.nav_speed,
            "min_valid_alt_m": self.min_valid_alt_m,
            "approx_start": list(self.approx_start) if self.approx_start else None,
            "tif_path": self.tif_path,
        }


# =============================================================================
# SYSTEM CONFIG (carried from navigate_north.py)
# =============================================================================
TIF_CRS       = "EPSG:3857"
OUTPUT_DIR    = "webui_logs"
AGL_THRESH    = 85.0
HFOV_DEG      = 84.0

# CHANGED: shared ArduCopter mode-name lookup, used by the periodic
# AGL/mode telemetry logging added to the takeoff/yaw-lock/first-fix/nav
# phases below (same table mavlink_nav_process already uses internally).
COPTER_MODE_NAMES = {0:"STABILIZE",1:"ACRO",2:"ALT_HOLD",3:"AUTO",4:"GUIDED",
                     5:"LOITER",6:"RTL",7:"CIRCLE",9:"LAND",16:"POSHOLD",
                     17:"BRAKE",18:"THROW",19:"AVOID_ADSB",20:"GUIDED_NOGPS",
                     21:"SMART_RTL"}


def mode_name(custom_mode):
    return COPTER_MODE_NAMES.get(custom_mode, f"#{custom_mode}")
PATCH_SCALE   = 2.5
MATCH_SIZE    = 512
DEVICE        = "cuda" if torch.cuda.is_available() else "cpu"
MAVLINK_CONN  = None

SIYI_IP            = "192.168.144.25"
SIYI_UDP_PORT      = 37260
RTSP_URL           = f"rtsp://{SIYI_IP}:8554/main.264"
CAMERA_FPS         = 7
CROP_W             = 1280
CROP_H             = 720
CROP_X_OFFSET      = 0
CROP_Y_OFFSET      = 0
MAX_FRAME_QUEUE    = 30
MAX_RECONNECTS     = 999
RECONNECT_DELAY    = 3.0

# CHANGED: preflight frame check, run on the ground before GUIDED/ARM/TAKEOFF.
# Confirms the camera is actually delivering images, not just that the RTSP
# process launched — a stalled/blank feed should abort here, not surface
# later as a silent hang during first-fix acquisition at altitude.
CAMERA_PREFLIGHT_FRAMES          = 5
CAMERA_PREFLIGHT_FRAME_TIMEOUT_S = 3.0

MAX_HEADING_CHANGE_DEG = 10.0
MAX_CONSEC_FAILURES    = 30
TELEM_TIMEOUT_S        = 5.0
OUTLIER_REJECT_M       = 25.0
GPS_ARRIVAL_RADIUS_M   = 8.0
GPS_SANITY_ERROR_M     = 10.0
GPS_SANITY_DURATION_S  = 15.0

# CHANGED: speed ramp-down on final approach to a waypoint. Flying the full
# leg speed all the way to a tight arrival_radius risks overshooting it
# outright given real control-loop latency and cross-track drift -- slow
# down for capture instead, like any normal waypoint-navigation approach.
SLOWDOWN_RADIUS_M      = 20.0    # m -- start ramping down inside this distance
MIN_APPROACH_SPEED_MPS = 0.5     # m/s -- speed floor right at arrival_radius

YAW_EMA_ALPHA          = 0.3
YAW_QUALITY_MAX_COND   = 1.5
YAW_DIFF_WARN_DEG      = 25.0
YAW_DIFF_WARN_HOLD_S   = 3.0

LOCK_YAW_NORTH_DEG     = 0.0
YAW_REASSERT_PERIOD_S  = 10.0
YAW_LOCK_RATE_DEG_S    = 25.0

# CHANGED: first-visual-fix acquisition tuning
FIRST_FIX_MIN_INLIERS  = 25      # need a confident match to accept as start
FIRST_FIX_TIMEOUT_S    = 45.0    # give up acquiring start after this
FIRST_FIX_TILE_STRIDE  = 0.6     # full-map search: tile stride as frac of footprint

os.makedirs(OUTPUT_DIR, exist_ok=True)

CSV_COLS = [
    "frame_idx", "timestamp", "state", "leg",
    "gps_lat", "gps_lon", "ekf_lat", "ekf_lon",
    "pred_lat", "pred_lon", "target_lat", "target_lon",
    "cte_m", "dist_to_target_m", "error_m",
    "heading_cmd_deg", "speed_cmd", "vn_cmd", "ve_cmd",
    "yaw_visual_deg", "n_inliers", "n_matches", "mahal",
    "agl_m", "latency_ms", "status", "gps_only",
]


# =============================================================================
# SIYI GIMBAL (verbatim from navigate_north.py)
# =============================================================================
def crc16_ccitt(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
        crc &= 0xFFFF
    return crc


def build_packet(cmd_id, data=b"", seq=1):
    body = (bytes([0x55, 0x66, 0x01])
          + struct.pack("<H", len(data))
          + struct.pack("<H", seq)
          + bytes([cmd_id])
          + data)
    return body + struct.pack("<H", crc16_ccitt(body))


class GimbalController:
    def __init__(self, ip, port):
        self.ip = ip; self.port = port; self.seq = 1
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1)
        self.running = False; self.thread = None

    def _send(self, cmd_id, data=b""):
        try:
            self.sock.sendto(build_packet(cmd_id, data, self.seq),
                             (self.ip, self.port))
            self.seq += 1
            resp, _ = self.sock.recvfrom(1024)
            return resp
        except Exception:
            return None

    def set_angle(self, pitch_deg, yaw_deg=0.0):
        return self._send(0x0E, struct.pack("<hh",
                          int(yaw_deg * 10), int(pitch_deg * 10)))

    def lock_down(self):
        self.running = True
        def _loop():
            while self.running:
                self.set_angle(-90.0)
                time.sleep(0.5)
        self.thread = threading.Thread(target=_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.sock.close()


# =============================================================================
# NAVIGATION EKF (verbatim)
# =============================================================================
class NavigationEKF:
    M_PER_LAT = 111_320.0

    def __init__(self, ref_lat=17.6, base_meas_err_m=3.5):
        self.m_per_lon = self.M_PER_LAT * math.cos(math.radians(ref_lat))
        self.x = np.zeros(4)
        pos_noise_deg = 1e-6
        vel_noise     = 0.3
        self.Q = np.diag([pos_noise_deg**2, pos_noise_deg**2,
                          vel_noise**2, vel_noise**2])
        err_deg = base_meas_err_m / self.M_PER_LAT
        self.R_base = np.diag([err_deg**2, err_deg**2])
        self.R_base_inliers = 187.0
        init_pos = (10.0 / self.M_PER_LAT)**2
        self.P = np.diag([init_pos, init_pos, 1.0, 1.0])
        self.initialized = False
        self.last_predict_t = None

    def initialize(self, lat, lon, t=None):
        self.x = np.array([lat, lon, 0.0, 0.0])
        self.last_predict_t = t or time.time()
        self.initialized = True

    def reset_to_gps(self, gps_lat, gps_lon):
        self.x[0] = gps_lat; self.x[1] = gps_lon
        reset_pos = (15.0 / self.M_PER_LAT)**2
        self.P[0, 0] = reset_pos; self.P[1, 1] = reset_pos

    def predict(self, vn_fc, ve_fc, t=None):
        now = t or time.time()
        if self.last_predict_t is None:
            self.last_predict_t = now; return
        dt = now - self.last_predict_t
        if dt <= 0 or dt > 2.0:
            self.last_predict_t = now; return
        self.last_predict_t = now
        F = np.eye(4)
        F[0, 2] = dt / self.M_PER_LAT
        F[1, 3] = dt / self.m_per_lon
        self.x = F @ self.x
        alpha = 0.8
        self.x[2] = alpha * vn_fc + (1 - alpha) * self.x[2]
        self.x[3] = alpha * ve_fc + (1 - alpha) * self.x[3]
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, lat_meas, lon_meas, n_inliers=187):
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float64)
        scale = (self.R_base_inliers / max(n_inliers, 10))**2
        R = self.R_base * scale
        z = np.array([lat_meas, lon_meas])
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        S_inv = np.linalg.inv(S)
        mahal = float(y @ S_inv @ y)
        if mahal > 9.21:
            return False, mahal
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        return True, mahal

    def get_position(self):
        return float(self.x[0]), float(self.x[1])

    def get_velocity(self):
        return float(self.x[2]), float(self.x[3])


# =============================================================================
# MULTI-WAYPOINT NAVIGATOR
# CHANGED: was StraightLineNavigator(A,B). Now walks an ordered waypoint list.
# =============================================================================
class WaypointNavigator:
    """
    Sequential leg navigator. Holds an ordered list of waypoints and flies
    A->wp0->wp1->...  On arrival at the current target it advances to the next.
    Per-leg logic is identical to the original StraightLineNavigator: cross-
    track dead band -> CRUISE; outside -> CORRECT; stuck -> GPS_RESET (only if
    gps_correction policy allows). Emits LAND only after the FINAL waypoint.
    """
    def __init__(self, start, waypoints, cfg: MissionConfig):
        self.cfg = cfg
        self.legs = [start] + list(waypoints)   # [start, wp0, wp1, ...]
        self.idx = 1                            # index of current TARGET in self.legs
        self.state = "IDLE"
        self.correct_start_time = None
        self._recompute_leg()

    def _recompute_leg(self):
        self.A = self.legs[self.idx - 1]
        self.B = self.legs[self.idx]
        self.bearing_ab = self._bearing(self.A, self.B)
        self.leg_dist = haversine_m(*self.A, *self.B)

    @property
    def target(self):
        return self.B

    @property
    def is_final_leg(self):
        return self.idx == len(self.legs) - 1

    @property
    def leg_number(self):
        return self.idx            # 1-based leg index

    @property
    def total_legs(self):
        return len(self.legs) - 1

    @property
    def leg_speed(self):
        """Speed for the current leg (arriving at legs[idx])."""
        return self.cfg.leg_speed[self.idx - 1]

    @property
    def leg_alt(self):
        """Altitude to fly the current leg at (arriving at legs[idx])."""
        return self.cfg.leg_alt[self.idx - 1]

    @staticmethod
    def _bearing(p1, p2):
        lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
        lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2)
             - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        return math.atan2(x, y) % (2 * math.pi)

    def cross_track_error(self, lat, lon):
        d_ap = haversine_m(self.A[0], self.A[1], lat, lon)
        brg_ap = self._bearing(self.A, (lat, lon))
        return d_ap * math.sin(brg_ap - self.bearing_ab)

    def distance_to_target(self, lat, lon):
        return haversine_m(lat, lon, self.B[0], self.B[1])

    def approach_speed(self, dist_b):
        """
        Ramp speed down from leg_speed to MIN_APPROACH_SPEED_MPS as the
        drone closes inside SLOWDOWN_RADIUS_M of the target, reaching the
        floor right at arrival_radius. Flying the full cruise speed all
        the way to a tight (e.g. 5m) capture radius risks overshooting it
        outright given real control-loop latency and cross-track drift.
        """
        cfg = self.cfg
        if dist_b >= SLOWDOWN_RADIUS_M:
            return self.leg_speed
        span = max(SLOWDOWN_RADIUS_M - cfg.arrival_radius, 1e-6)
        frac = (dist_b - cfg.arrival_radius) / span
        frac = min(max(frac, 0.0), 1.0)
        return MIN_APPROACH_SPEED_MPS + frac * (self.leg_speed - MIN_APPROACH_SPEED_MPS)

    def compute(self, lat, lon, current_time):
        """Returns (heading, speed, state, cte, dist_to_target, advanced_bool)."""
        cfg = self.cfg
        dist_b = self.distance_to_target(lat, lon)
        cte    = self.cross_track_error(lat, lon)
        advanced = False

        if dist_b < cfg.arrival_radius:
            if self.is_final_leg:
                self.state = "LAND"
                self.correct_start_time = None
                return 0.0, 0.0, "LAND", cte, dist_b, advanced
            # Advance to the next leg.
            self.idx += 1
            self._recompute_leg()
            self.correct_start_time = None
            advanced = True
            self.state = "WAYPOINT"
            # recompute against new leg immediately
            dist_b = self.distance_to_target(lat, lon)
            cte    = self.cross_track_error(lat, lon)

        speed = self.approach_speed(dist_b)

        if abs(cte) < cfg.cte_threshold:
            self.state = "CRUISE"
            self.correct_start_time = None
            return self.bearing_ab, speed, "CRUISE", cte, dist_b, advanced

        # Outside dead band.
        if cfg.policy.gps_correction:
            # GPS backstop allowed -> may escalate to GPS_RESET if stuck.
            if self.correct_start_time is None:
                self.correct_start_time = current_time
            if current_time - self.correct_start_time > cfg.gps_fallback_time:
                self.state = "GPS_RESET"
                self.correct_start_time = None
                return (self.bearing_ab, speed, "GPS_RESET",
                        cte, dist_b, advanced)

        heading = self._bearing((lat, lon), self.B)
        self.state = "CORRECT"
        return heading, speed, "CORRECT", cte, dist_b, advanced


# =============================================================================
# COORDINATE HELPERS (verbatim)
# =============================================================================
wgs84_to_3857  = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
_3857_to_wgs84 = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2)**2
    return 2 * R * math.asin(math.sqrt(a))


def footprint_metres(agl_m, hfov_deg=HFOV_DEG, img_w=CROP_W, img_h=CROP_H):
    w = 2 * agl_m * math.tan(math.radians(hfov_deg) / 2)
    return w, w * (img_h / img_w)


def tif_res_m(tif_src):
    return abs(tif_src.transform.a)


def latlon_to_tif_pixel(lat, lon, tif_src):
    x, y = wgs84_to_3857.transform(lon, lat)
    row, col = rowcol(tif_src.transform, x, y)
    return int(row), int(col)


def tif_pixel_to_latlon(row, col, tif_src):
    x, y = xy(tif_src.transform, row, col)
    lon_out, lat_out = _3857_to_wgs84.transform(x, y)
    return lat_out, lon_out


def extract_tif_patch(tif_src, cr, cc, ph, pw):
    h, w = tif_src.shape
    r0, r1 = cr - ph // 2, cr + ph // 2
    c0, c1 = cc - pw // 2, cc + pw // 2
    r0, r1 = max(0, r0), min(h, r1)
    c0, c1 = max(0, c0), min(w, c1)
    if (r1 - r0) < ph // 4 or (c1 - c0) < pw // 4:
        return None, None, None
    window = rasterio.windows.Window(c0, r0, c1 - c0, r1 - r0)
    bands  = tif_src.read([1, 2, 3], window=window)
    return np.transpose(bands, (1, 2, 0)), r0, c0


def angle_wrap_180(deg):
    return (deg + 180) % 360 - 180


def visual_yaw_from_homography(H):
    H2 = H[:2, :2].astype(np.float64)
    U, S, Vt = np.linalg.svd(H2)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R[:, 1] *= -1
    cond = float(S.max() / max(S.min(), 1e-9))
    yaw_deg = math.degrees(math.atan2(R[1, 0], R[0, 0])) % 360.0
    return yaw_deg, cond


# =============================================================================
# AUTO PORT DETECTION (verbatim)
# =============================================================================
def detect_mavlink_connection():
    from pymavlink import mavutil
    candidates = []
    for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyAMA*"]:
        candidates += [f"{p},115200" for p in sorted(glob.glob(pattern))]
        candidates += [f"{p},57600"  for p in sorted(glob.glob(pattern))]
    candidates += ["udpin:0.0.0.0:14550", "udpin:0.0.0.0:14551",
                   "udpin:0.0.0.0:18570", "udpin:0.0.0.0:14560"]
    for cs in candidates:
        try:
            m = mavutil.mavlink_connection(cs, autoreconnect=False)
            hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3.0)
            m.close()
            if hb:
                return cs
        except Exception:
            pass
    return None


# =============================================================================
# MAVLINK SUBPROCESS (verbatim from navigate_north.py — telemetry + commands)
# =============================================================================
def mavlink_nav_process(conn_str, telem_queue, cmd_queue, stop_flag):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        from pymavlink import mavutil
    except ImportError:
        telem_queue.put({"error": "pymavlink not installed"}); return

    try:
        mav = mavutil.mavlink_connection(conn_str)
    except Exception as e:
        telem_queue.put({"error": str(e)}); return

    hb = None
    deadline = time.time() + 30
    while not stop_flag.is_set() and time.time() < deadline:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if hb:
            break
    if hb is None:
        telem_queue.put({"error": "no heartbeat within 30 s"}); return

    for sid, cid in [(mav.target_system, mav.target_component), (0, 0)]:
        try:
            mav.mav.request_data_stream_send(
                sid, cid, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        except Exception:
            pass
    for msg_id in [mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                   mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                   mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD]:
        try:
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, 100_000, 0, 0, 0, 0, 0)
        except Exception:
            pass

    _COPTER_MODES = {0:"STABILIZE",1:"ACRO",2:"ALT_HOLD",3:"AUTO",4:"GUIDED",
                     5:"LOITER",6:"RTL",7:"CIRCLE",9:"LAND",16:"POSHOLD",
                     17:"BRAKE",18:"THROW",19:"AVOID_ADSB",20:"GUIDED_NOGPS",
                     21:"SMART_RTL"}

    ground_alt_m = None
    ground_alt_samples = []
    GROUND_ALT_SETTLE_S = 2.0
    ground_alt_first_t = None

    lat = lon = alt = 0.0
    vn = ve = vd = 0.0
    heading_deg = 0.0
    is_armed = False
    custom_mode = -1

    while not stop_flag.is_set():
        try:
            msg = mav.recv_match(
                type=["GLOBAL_POSITION_INT","GPS_RAW_INT","VFR_HUD",
                      "HEARTBEAT","STATUSTEXT","COMMAND_ACK"],
                blocking=True, timeout=0.05)
        except Exception:
            msg = None

        if msg is not None:
            mtype = msg.get_type()
            if mtype == "GLOBAL_POSITION_INT":
                a = msg.alt / 1000.0
                if -1000 < a < 10_000:
                    lat, lon, alt = msg.lat/1e7, msg.lon/1e7, a
                    vn = msg.vx/100.0; ve = msg.vy/100.0; vd = msg.vz/100.0
                    heading_deg = (msg.hdg/100.0 if msg.hdg != 65535 else heading_deg)
            elif mtype == "GPS_RAW_INT" and msg.fix_type >= 2:
                a = msg.alt / 1000.0
                if -1000 < a < 10_000:
                    lat, lon, alt = msg.lat/1e7, msg.lon/1e7, a
            elif mtype == "VFR_HUD":
                a = float(msg.alt)
                if -1000 < a < 10_000:
                    alt = a
                heading_deg = float(msg.heading)
            elif mtype == "HEARTBEAT":
                is_armed = bool(msg.base_mode &
                                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                custom_mode = msg.custom_mode
            elif mtype == "STATUSTEXT":
                txt = msg.text
                if isinstance(txt, bytes):
                    txt = txt.decode(errors="ignore")
                txt = txt.rstrip("\x00 ").strip()
                if txt:
                    sev = getattr(msg, "severity", 6)
                    telem_queue.put({"statustext": txt, "severity": sev})

            if ground_alt_m is None and lat != 0 and alt != 0:
                now = time.time()
                if ground_alt_first_t is None:
                    ground_alt_first_t = now
                ground_alt_samples.append(alt)
                if (now - ground_alt_first_t) >= GROUND_ALT_SETTLE_S and \
                        len(ground_alt_samples) >= 3:
                    ground_alt_m = float(np.median(ground_alt_samples))

            if lat != 0:
                telem_queue.put({
                    "lat": lat, "lon": lon,
                    "altitude_m": alt,
                    "ground_alt_m": ground_alt_m or alt,
                    "agl_m": alt - (ground_alt_m or alt),
                    "vn": vn, "ve": ve, "vd": vd,
                    "heading_deg": heading_deg,
                    "is_armed": is_armed,
                    "custom_mode": custom_mode,
                    "ts": time.time(),
                    "ts_iso": datetime.datetime.utcnow().isoformat(),
                })

        while not cmd_queue.empty():
            try:
                cmd = cmd_queue.get_nowait()
            except Exception:
                break
            ctype = cmd.get("type")

            if ctype == "velocity":
                type_mask = (
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
                mav.mav.set_position_target_global_int_send(
                    0, mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask, 0, 0, 0,
                    cmd["vn"], cmd["ve"], cmd["vd"],
                    0, 0, 0, cmd["yaw"], 0)

            elif ctype == "goto_alt":
                # Hold horizontal velocity at zero and drive to a target relative
                # altitude. Used for the level-change step at a waypoint before
                # flying the next leg. Position X/Y ignored (velocity held 0),
                # altitude Z active as a setpoint.
                type_mask = (
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
                mav.mav.set_position_target_global_int_send(
                    0, mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask, 0, 0, float(cmd["alt"]),
                    0.0, 0.0, 0.0,
                    0, 0, 0, cmd["yaw"], 0)

            elif ctype == "set_mode":
                mode_map = {"GUIDED":4,"LOITER":5,"LAND":9,"RTL":6}
                mode_id = mode_map.get(cmd["mode"])
                if mode_id is not None:
                    mav.mav.command_long_send(
                        mav.target_system, mav.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_id, 0, 0, 0, 0, 0)

            elif ctype == "arm":
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1, 0, 0, 0, 0, 0, 0)

            elif ctype == "takeoff":
                a = cmd["alt"]
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                    0, 0, 0, 0, lat, lon, a)

            elif ctype == "condition_yaw":
                tgt_deg = float(cmd.get("yaw_deg", 0.0))
                rate    = float(cmd.get("rate_deg_s", 25.0))
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                    tgt_deg, rate, 0, 0, 0, 0, 0)


# =============================================================================
# LIGHTGLUE MODELS (verbatim)
# =============================================================================
extractor = None
matcher = None


def _load_models():
    global extractor, matcher
    if extractor is None:
        extractor = SuperPoint(max_num_keypoints=1024).eval().to(DEVICE)
        matcher = LightGlue(features="superpoint").eval().to(DEVICE)


def img_to_tensor(img):
    img = cv2.resize(img, (MATCH_SIZE, MATCH_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return torch.tensor(img / 255.0, dtype=torch.float32)[None, None].to(DEVICE)


def run_lightglue(img0, img1):
    h0, w0 = img0.shape[:2]
    h1, w1 = img1.shape[:2]
    with torch.no_grad():
        f0    = extractor.extract(img_to_tensor(img0))
        f1    = extractor.extract(img_to_tensor(img1))
        m_out = matcher({"image0": f0, "image1": f1})
        f0, f1, m_out = rbd(f0), rbd(f1), rbd(m_out)
    matches = m_out["matches"]
    if matches.shape[0] < 8:
        return None, None
    k0 = f0["keypoints"][matches[:, 0]].cpu().numpy()
    k1 = f1["keypoints"][matches[:, 1]].cpu().numpy()
    k0[:, 0] *= w0 / MATCH_SIZE; k0[:, 1] *= h0 / MATCH_SIZE
    k1[:, 0] *= w1 / MATCH_SIZE; k1[:, 1] *= h1 / MATCH_SIZE
    return k0, k1


def localize_image(drone_img, tif_src, search_lat, search_lon, agl_m):
    res_m  = tif_res_m(tif_src)
    fw, fh = footprint_metres(agl_m)
    pw     = int(fw * PATCH_SCALE / res_m)
    ph     = int(fh * PATCH_SCALE / res_m)
    cr, cc = latlon_to_tif_pixel(search_lat, search_lon, tif_src)
    patch, pr0, pc0 = extract_tif_patch(tif_src, cr, cc, ph, pw)
    if patch is None:
        return None
    patch_bgr = cv2.cvtColor(patch.astype(np.uint8), cv2.COLOR_RGB2BGR)
    dh, dw = drone_img.shape[:2]
    k0, k1 = run_lightglue(drone_img, patch_bgr)
    if k0 is None:
        return None
    H, mask = cv2.findHomography(k0, k1, cv2.RANSAC, 5.0)
    if H is None or mask.sum() < 8:
        return None
    corners = np.float32([[0,0],[dw,0],[dw,dh],[0,dh]]).reshape(-1,1,2)
    cp  = cv2.perspectiveTransform(corners, H).reshape(-1,2)
    ctr = cp.mean(axis=0)
    loc_lat, loc_lon = tif_pixel_to_latlon(pr0 + ctr[1], pc0 + ctr[0], tif_src)
    yaw_deg, yaw_cond = visual_yaw_from_homography(H)
    return {"loc_lat": loc_lat, "loc_lon": loc_lon,
            "n_inliers": int(mask.sum()), "n_matches": len(k0),
            "yaw_visual_deg": yaw_deg, "yaw_cond": yaw_cond}


def localize_full_map(drone_img, tif_src, agl_m, stride_frac=FIRST_FIX_TILE_STRIDE):
    """
    CHANGED: NEW. Zero-prior localisation for GPS-denied first fix.
    Tiles the whole TIF at the current footprint scale, matches each tile,
    returns the best (most inliers) fix. Slow (seconds) — only used ONCE to
    acquire waypoint_1 when no approx-start pin is provided.
    """
    res_m  = tif_res_m(tif_src)
    fw, fh = footprint_metres(agl_m)
    pw = int(fw * PATCH_SCALE / res_m)
    ph = int(fh * PATCH_SCALE / res_m)
    H_img, W_img = tif_src.shape
    step_r = max(1, int(ph * stride_frac))
    step_c = max(1, int(pw * stride_frac))
    best = None
    for cr in range(ph // 2, H_img - ph // 2, step_r):
        for cc in range(pw // 2, W_img - pw // 2, step_c):
            lat, lon = tif_pixel_to_latlon(cr, cc, tif_src)
            res = localize_image(drone_img, tif_src, lat, lon, agl_m)
            if res and res["n_inliers"] >= FIRST_FIX_MIN_INLIERS:
                if best is None or res["n_inliers"] > best["n_inliers"]:
                    best = res
    return best


# =============================================================================
# SIYI CAMERA THREAD
# =============================================================================
camera_stop = threading.Event()


def camera_thread_fn(rtsp_url, frame_q, log_fn=print):
    """
    CHANGED: restored the [CAM]/[FFmpeg] diagnostics that navigate_north.py
    had (they were dropped when this became a callback-driven engine with
    no print()s). log_fn defaults to plain print() for standalone/demo use;
    FlightEngine.run() passes self._log_raw so these land in the same txt
    log + terminal as everything else, instead of being silently discarded.
    """
    def probe_resolution(url):
        out = subprocess.check_output([
            "ffprobe","-v","error","-select_streams","v:0",
            "-show_entries","stream=width,height","-of","csv=p=0",url
        ], timeout=10).decode().strip()
        w, h = map(int, out.split(","))
        return w, h

    attempt = 0
    while not camera_stop.is_set() and attempt < MAX_RECONNECTS:
        attempt += 1
        if attempt > 1:
            log_fn(f"[CAM] Reconnect attempt {attempt} in {RECONNECT_DELAY}s...")
            time.sleep(RECONNECT_DELAY)
        proc = None
        try:
            w, h = probe_resolution(rtsp_url)
            frame_size = w * h * 3
            x1 = max(0, min((w - CROP_W)//2 + CROP_X_OFFSET, w - CROP_W))
            y1 = max(0, min((h - CROP_H)//2 + CROP_Y_OFFSET, h - CROP_H))
            log_fn(f"[CAM] {w}x{h} -> crop {CROP_W}x{CROP_H} @ ({x1},{y1}) "
                   f"attempt={attempt}")
            cmd = ["ffmpeg","-nostdin","-rtsp_transport","tcp",
                   "-fflags","nobuffer","-flags","low_delay",
                   "-stimeout","5000000",
                   "-c:v","hevc_nvv4l2dec",  # CHANGED: hardware HEVC decode
                   "-i",rtsp_url,
                   "-vf",f"fps={CAMERA_FPS}","-pix_fmt","bgr24",
                   "-vcodec","rawvideo","-an","-f","rawvideo","pipe:1"]
            proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, bufsize=10**8)

            def _log_stderr(p):
                for line in p.stderr:
                    line = line.decode(errors="ignore").strip()
                    if line and "frame=" not in line:
                        log_fn(f"[FFmpeg] {line}")
            threading.Thread(target=_log_stderr, args=(proc,),
                             daemon=True).start()

            time.sleep(3)
            if proc.poll() is not None:
                log_fn(f"[CAM] FFmpeg exited early (code {proc.returncode})")
                continue
            log_fn("[CAM] Stream started.")
            consecutive_errors = 0
            while not camera_stop.is_set():
                raw = proc.stdout.read(frame_size)
                if not raw or len(raw) != frame_size:
                    consecutive_errors += 1
                    if consecutive_errors >= 10:
                        log_fn("[CAM] Stream lost — will reconnect.")
                        break
                    continue
                consecutive_errors = 0
                frame = np.frombuffer(raw, np.uint8).reshape((h, w, 3))
                frame = frame[y1:y1+CROP_H, x1:x1+CROP_W].copy()
                if frame_q.full():
                    try: frame_q.get_nowait()
                    except queue.Empty: pass
                frame_q.put((frame, time.time()))
        except Exception as e:
            log_fn(f"[CAM] Error: {e}")
        finally:
            if proc is not None:
                try: proc.terminate()
                except Exception: pass
    log_fn("[CAM] Thread exiting.")
    camera_stop.set()


def get_latest_frame(frame_q, timeout=2.0):
    try:
        frame, ts = frame_q.get(timeout=timeout)
    except queue.Empty:
        return None
    while True:
        try:
            frame, ts = frame_q.get_nowait()
        except queue.Empty:
            break
    return frame


# =============================================================================
# FLIGHT ENGINE — the orchestrator. Driven by MissionConfig + callbacks.
# =============================================================================
class FlightEngine:
    """
    Runs one mission. The agent constructs this with a MissionConfig and three
    callbacks:
        on_event(dict)   — structured events (state changes, waypoint reached,
                           first-fix acquired, aborts) -> WebSocket to GUI.
        on_telem(dict)   — high-rate position/nav telemetry -> WebSocket.
        should_abort()   — polled each loop; return True to abort (GUI STOP btn).

    Call .run() (blocking). Safe to run in a background thread.
    """
    def __init__(self, cfg: MissionConfig, on_event=None, on_telem=None,
                 should_abort=None):
        self.cfg = cfg
        self.on_event = on_event or (lambda e: None)
        self.on_telem = on_telem or (lambda t: None)
        self.should_abort = should_abort or (lambda: False)
        self.gimbal = None
        self.mav_proc = None
        self.mav_stop = None
        self.log_fh = None

    def _event(self, kind, **kw):
        e = {"kind": kind, "ts": time.time(), **kw}
        try: self.on_event(e)
        except Exception: pass
        if self.log_fh is not None:
            try:
                ts_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                detail = " ".join(f"{k}={v}" for k, v in kw.items())
                self.log_fh.write(f"[{ts_str}] {kind:20s} {detail}\n")
                self.log_fh.flush()
            except Exception:
                pass

    def _log_raw(self, line):
        """
        Dense, per-frame debug line -> txt log AND terminal (not on_event/
        GUI, so the browser's live event stream isn't flooded at camera
        fps). Mirrors navigate_north_auto.py's per-frame console print so a
        stalled/misbehaving flight can be diagnosed frame-by-frame either
        from the log file or live from the terminal running drone_agent.py.
        """
        ts_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        print(f"[{ts_str}] {line}", flush=True)
        if self.log_fh is None:
            return
        try:
            self.log_fh.write(f"[{ts_str}] {line}\n")
            self.log_fh.flush()
        except Exception:
            pass

    def run(self):
        cfg = self.cfg
        camera_stop.clear()

        # Open the txt/csv log pair FIRST so every event — including an abort
        # during model loading, gimbal, or MAVLink connect — is captured on
        # disk even if the GUI never sees it.
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(OUTPUT_DIR, f"nav_log_{ts}.csv")
        txt_path = os.path.join(OUTPUT_DIR, f"nav_log_{ts}.txt")
        self.log_fh = open(txt_path, "w")
        pd.DataFrame(columns=CSV_COLS).to_csv(csv_path, index=False)

        _load_models()
        self._event("engine_start", mode=cfg.mode, config=cfg.to_dict(),
                    log_txt=txt_path, log_csv=csv_path)

        # ── Gimbal ───────────────────────────────────────────────────────────
        self.gimbal = GimbalController(SIYI_IP, SIYI_UDP_PORT)
        self.gimbal.set_angle(-90.0)
        time.sleep(1.5)
        self.gimbal.lock_down()
        self._event("gimbal_locked")

        # ── MAVLink ──────────────────────────────────────────────────────────
        conn_str = MAVLINK_CONN or detect_mavlink_connection()
        if conn_str is None:
            self._event("abort", reason="no MAVLink connection found")
            self._cleanup(); return
        self._event("mavlink_connecting", conn=conn_str)

        # ── Camera ───────────────────────────────────────────────────────────
        frame_q = queue.Queue(maxsize=MAX_FRAME_QUEUE)
        threading.Thread(target=camera_thread_fn,
                         args=(RTSP_URL, frame_q, self._log_raw),
                         daemon=True).start()
        time.sleep(2.0)
        if camera_stop.is_set():
            self._event("abort", reason="camera failed to open")
            self._cleanup(); return

        # CHANGED: preflight frame check — verify real frames are actually
        # flowing through frame_q (not just that ffmpeg/RTSP connected)
        # before arming/taking off. Collects CAMERA_PREFLIGHT_FRAMES
        # successes within a wall-clock budget (not a fixed attempt count):
        # a fresh RTSP connection with software HEVC decode can take a
        # few seconds to produce its first frame, so an early timeout
        # must NOT permanently cost one of the required successes.
        frames_seen = 0
        attempt = 0
        preflight_deadline = (time.time()
                              + CAMERA_PREFLIGHT_FRAMES * CAMERA_PREFLIGHT_FRAME_TIMEOUT_S)
        while frames_seen < CAMERA_PREFLIGHT_FRAMES and time.time() < preflight_deadline:
            attempt += 1
            if camera_stop.is_set():
                self._log_raw(f"[cam-preflight] camera_stop set, aborting "
                              f"early ({frames_seen}/{CAMERA_PREFLIGHT_FRAMES} "
                              f"frames seen)")
                break
            remaining = max(0.1, preflight_deadline - time.time())
            try:
                frame_q.get(timeout=min(CAMERA_PREFLIGHT_FRAME_TIMEOUT_S, remaining))
                frames_seen += 1
                self._log_raw(f"[cam-preflight] frame {frames_seen}/"
                              f"{CAMERA_PREFLIGHT_FRAMES} received "
                              f"(attempt {attempt})")
            except queue.Empty:
                self._log_raw(f"[cam-preflight] attempt {attempt} timed out, "
                              f"retrying ({frames_seen}/"
                              f"{CAMERA_PREFLIGHT_FRAMES} so far, "
                              f"{preflight_deadline - time.time():.1f}s left)")
        if frames_seen < CAMERA_PREFLIGHT_FRAMES:
            self._event("abort",
                        reason=f"camera preflight failed: only {frames_seen}/"
                               f"{CAMERA_PREFLIGHT_FRAMES} frames received "
                               f"(<= {CAMERA_PREFLIGHT_FRAME_TIMEOUT_S:.0f}s "
                               f"each)")
            self._cleanup(); return
        self._event("camera_ready", preflight_frames=frames_seen)

        telem_queue = mp.Queue(maxsize=50)
        cmd_queue   = mp.Queue(maxsize=10)
        self.mav_stop = mp.Event()
        self.mav_proc = mp.Process(target=mavlink_nav_process,
                                   args=(conn_str, telem_queue, cmd_queue,
                                         self.mav_stop), daemon=True)
        self.mav_proc.start()

        # ── Wait first telemetry ─────────────────────────────────────────────
        latest = None
        deadline = time.time() + 40
        while time.time() < deadline:
            while not telem_queue.empty():
                item = telem_queue.get_nowait()
                if "error" in item:
                    self._event("abort", reason=f"MAVLink: {item['error']}")
                    self._cleanup(); return
                if "statustext" in item:
                    self._event("ap_status", text=item["statustext"],
                                severity=item.get("severity", 6))
                    continue
                latest = item
            if latest:
                break
            time.sleep(0.2)
        if latest is None:
            self._event("abort", reason="no telemetry within timeout")
            self._cleanup(); return
        self._event("telemetry_ok", agl=latest["agl_m"],
                    lat=latest["lat"], lon=latest["lon"])

        def drain():
            nonlocal latest
            while not telem_queue.empty():
                try:
                    item = telem_queue.get_nowait()
                    if "statustext" in item:
                        self._event("ap_status", text=item["statustext"],
                                    severity=item.get("severity", 6))
                    elif "error" not in item:
                        latest = item
                except Exception:
                    break

        def wait_for(pred, timeout_s, desc):
            dl = time.time() + timeout_s
            while time.time() < dl:
                drain()
                if latest is not None and pred(latest):
                    return True
                if self.should_abort():
                    return False
                time.sleep(0.2)
            self._event("timeout", desc=desc)
            return False

        # ── GUIDED -> ARM -> TAKEOFF (vertical) ──────────────────────────────
        self._event("phase", phase="GUIDED")
        cmd_queue.put({"type": "set_mode", "mode": "GUIDED"})
        if not wait_for(lambda t: t.get("custom_mode") == 4, 5.0, "GUIDED"):
            self._event("abort", reason="did not enter GUIDED")
            self._cleanup(); return

        self._event("phase", phase="ARM")
        cmd_queue.put({"type": "arm"})
        if not wait_for(lambda t: t.get("is_armed"), 8.0, "ARM"):
            self._event("abort", reason="did not arm (check pre-arm/GPS/safety)")
            cmd_queue.put({"type": "set_mode", "mode": "LOITER"})
            self._cleanup(); return
        time.sleep(1.0)

        # CHANGED: vertical takeoff, then FIRST-FIX acquisition before any
        # horizontal motion.
        self._event("phase", phase="TAKEOFF", target_alt=cfg.takeoff_alt)
        cmd_queue.put({"type": "takeoff", "alt": cfg.takeoff_alt})

        takeoff_deadline = time.time() + 120
        alt_stable_start = None
        ALT_TOLERANCE = 7.0
        agl_at_cmd = latest["agl_m"]
        lifted = False
        while time.time() < takeoff_deadline:
            drain()
            if self.should_abort():
                self._abort_land(cmd_queue, "operator STOP during takeoff"); return
            agl_now = latest["agl_m"]
            if not lifted and agl_now - agl_at_cmd > 2.0:
                lifted = True
                self._event("climbing", agl=agl_now)
            if abs(agl_now - cfg.takeoff_alt) < ALT_TOLERANCE:
                if alt_stable_start is None:
                    alt_stable_start = time.time()
                elif time.time() - alt_stable_start > 3.0:
                    break
            else:
                alt_stable_start = None
            if camera_stop.is_set() or not self.mav_proc.is_alive():
                self._abort_land(cmd_queue, "lost camera/MAVLink in takeoff"); return
            self.on_telem({"phase": "takeoff", "agl_m": agl_now,
                           "target_alt": cfg.takeoff_alt})
            # CHANGED: per-second AGL/mode telemetry so a real-time cause
            # (e.g. an unexpected mode change or altitude loss) is visible
            # in the log even if nothing else happens to log it.
            self._log_raw(f"[telemetry] agl={agl_now:.1f}m "
                          f"mode={mode_name(latest.get('custom_mode'))} "
                          f"armed={latest.get('is_armed')} (takeoff)")
            time.sleep(1.0)

        agl_final = latest["agl_m"]
        self._event("takeoff_complete", agl=agl_final)

        # ── Yaw lock north ───────────────────────────────────────────────────
        if cfg.lock_yaw_north:
            cmd_queue.put({"type": "condition_yaw",
                           "yaw_deg": LOCK_YAW_NORTH_DEG,
                           "rate_deg_s": YAW_LOCK_RATE_DEG_S})
            # CHANGED: stepped 1s wait (still 5s total) so AGL/mode is
            # logged every second here too -- this settle window previously
            # had zero telemetry visibility.
            for _ in range(5):
                drain()
                self._log_raw(f"[telemetry] agl={latest['agl_m']:.1f}m "
                              f"mode={mode_name(latest.get('custom_mode'))} "
                              f"armed={latest.get('is_armed')} (yaw-lock settle)")
                time.sleep(1.0)
            self._event("yaw_locked_north")

        # ── ACQUIRE START (waypoint_1) ───────────────────────────────────────
        # This is the heart of the mode logic.
        try:
            with rasterio.open(cfg.tif_path) as tif_src:
                start = self._acquire_start(cfg, tif_src, frame_q, latest, drain)
                if start is None:
                    self._abort_land(cmd_queue, "failed to acquire start location")
                    return
                self._event("start_acquired", lat=start[0], lon=start[1],
                            source=("gps" if cfg.policy.seed_from_gps
                                    else "visual"))

                # EKF seeded at the acquired start.
                ekf = NavigationEKF(ref_lat=start[0], base_meas_err_m=3.5)
                ekf.initialize(start[0], start[1])
                nav = WaypointNavigator(start, cfg.waypoints, cfg)
                self._event("navigator_ready",
                            legs=nav.total_legs,
                            waypoints=[list(w) for w in cfg.waypoints])

                # ── MAIN NAV LOOP ────────────────────────────────────────────
                self._nav_loop(cfg, tif_src, ekf, nav, frame_q, cmd_queue,
                               telem_queue, latest, csv_path)
        except Exception as e:
            self._event("abort", reason=f"engine exception: {e}",
                        traceback=traceback.format_exc())
            try: cmd_queue.put({"type": "set_mode", "mode": "LOITER"})
            except Exception: pass
        finally:
            self._cleanup()

    # -------------------------------------------------------------------------
    def _acquire_start(self, cfg, tif_src, frame_q, latest, drain):
        """
        Returns (lat,lon) start location or None.

          seed_from_gps  -> just read current GPS (safety, safe_start).
          else           -> vertical-climb already done; localise at altitude.
                            approx_start pin narrows search; else full-map.
                            Reject any fix below min_valid_alt_m.
        """
        if cfg.policy.seed_from_gps:
            drain()
            self._log_raw(f"[first-fix] seeded from GPS: "
                          f"({latest['lat']:.6f},{latest['lon']:.6f})")
            return (latest["lat"], latest["lon"])

        # Visual first fix.
        self._event("phase", phase="ACQUIRE_START_VISUAL",
                    approx_start=list(cfg.approx_start) if cfg.approx_start else None)
        search_desc = (f"approx_start=({cfg.approx_start[0]:.6f},"
                       f"{cfg.approx_start[1]:.6f})"
                       if cfg.approx_start is not None else "full-map search")
        t_start = time.time()
        attempt = 0
        while time.time() - t_start < FIRST_FIX_TIMEOUT_S:
            drain()
            if self.should_abort():
                self._log_raw("[first-fix] aborted by operator STOP")
                return None
            agl_m = latest["agl_m"]
            elapsed = time.time() - t_start
            # CHANGED: unconditional per-iteration AGL/mode telemetry --
            # previously this whole phase (up to 45s) only logged when a
            # frame/localisation attempt happened, leaving no visibility
            # into altitude/mode during that window otherwise.
            self._log_raw(f"[telemetry] agl={agl_m:.1f}m "
                          f"mode={mode_name(latest.get('custom_mode'))} "
                          f"armed={latest.get('is_armed')} "
                          f"elapsed={elapsed:.1f}s (first-fix)")
            if agl_m < cfg.min_valid_alt_m:
                # Below the floor -> invalid. Keep waiting/climbing.
                self._event("first_fix_wait",
                            agl=agl_m, floor=cfg.min_valid_alt_m,
                            reason="below min valid altitude")
                self._log_raw(f"[first-fix] waiting for altitude: "
                              f"agl={agl_m:.1f}m < floor={cfg.min_valid_alt_m:.1f}m "
                              f"elapsed={elapsed:.1f}s")
                time.sleep(0.5)
                continue
            attempt += 1
            t0 = time.time()
            frame = get_latest_frame(frame_q, timeout=2.0)
            if frame is None:
                self._log_raw(f"[first-fix #{attempt}] no camera frame received "
                              f"(agl={agl_m:.1f}m elapsed={elapsed:.1f}s/"
                              f"{FIRST_FIX_TIMEOUT_S:.0f}s)")
                continue
            if cfg.approx_start is not None:
                res = localize_image(frame, tif_src,
                                     cfg.approx_start[0], cfg.approx_start[1],
                                     agl_m)
            else:
                self._event("first_fix_fullmap_search")
                res = localize_full_map(frame, tif_src, agl_m)
            lat_ms = (time.time() - t0) * 1000
            n_inl = res["n_inliers"] if res else 0
            n_match = res["n_matches"] if res else 0
            if res and n_inl >= FIRST_FIX_MIN_INLIERS:
                self._event("first_fix_ok", inliers=n_inl, agl=agl_m)
                self._log_raw(f"[first-fix #{attempt}] OK agl={agl_m:.1f}m "
                              f"{search_desc} inliers={n_inl} matches={n_match} "
                              f"-> ({res['loc_lat']:.6f},{res['loc_lon']:.6f}) "
                              f"{lat_ms:.0f}ms")
                return (res["loc_lat"], res["loc_lon"])
            self._event("first_fix_retry", inliers=n_inl)
            self._log_raw(f"[first-fix #{attempt}] retry agl={agl_m:.1f}m "
                          f"{search_desc} inliers={n_inl} matches={n_match} "
                          f"{lat_ms:.0f}ms elapsed={elapsed:.1f}s/"
                          f"{FIRST_FIX_TIMEOUT_S:.0f}s")
        self._log_raw(f"[first-fix] TIMEOUT after {attempt} attempts, "
                      f"{FIRST_FIX_TIMEOUT_S:.0f}s elapsed")
        return None

    # -------------------------------------------------------------------------
    def _change_altitude(self, target_alt, from_alt, cfg, cmd_queue,
                         telem_queue, latest, yaw_cmd_rad):
        """
        Climb/descend to target_alt (relative altitude) while holding horizontal
        position, then return. Called at a waypoint before flying the next leg.
        Blocks (with a timeout) until AGL is within tolerance of target_alt.
        Aborts early on operator STOP; safety-critical loss handled by caller.
        """
        direction = "climb" if target_alt > from_alt else "descend"
        self._event("alt_change_start", direction=direction,
                    from_alt=round(from_alt, 1), to_alt=round(target_alt, 1))
        ALT_TOL = 3.0
        deadline = time.time() + 90.0
        stable_start = None
        while time.time() < deadline:
            if self.should_abort():
                self._event("alt_change_abort", reason="operator STOP")
                return
            # Refresh telemetry.
            while not telem_queue.empty():
                try:
                    item = telem_queue.get_nowait()
                    if "statustext" in item:
                        self._event("ap_status", text=item["statustext"],
                                    severity=item.get("severity", 6))
                    elif "error" not in item:
                        latest.update(item)
                except Exception:
                    break
            if camera_stop.is_set() or not self.mav_proc.is_alive():
                self._event("alt_change_abort", reason="lost camera/MAVLink")
                return
            # Command the target altitude, horizontal velocity zero.
            cmd_queue.put({"type": "goto_alt", "alt": target_alt,
                           "yaw": yaw_cmd_rad})
            agl = latest["agl_m"]
            self.on_telem({"phase": "alt_change", "agl_m": agl,
                           "target_alt": target_alt})
            if abs(agl - target_alt) < ALT_TOL:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start > 2.0:
                    break
            else:
                stable_start = None
            time.sleep(0.5)
        self._event("alt_change_done", agl=round(latest["agl_m"], 1),
                    to_alt=round(target_alt, 1))

    # -------------------------------------------------------------------------
    def _nav_loop(self, cfg, tif_src, ekf, nav, frame_q, cmd_queue,
                  telem_queue, latest, csv_path):
        results = []
        frame_idx = 0
        consec_fail = 0
        last_telem_ts = time.time()
        prev_heading = nav.bearing_ab
        last_accepted_pred = None
        gps_reset_count = 0
        high_error_start = None
        gps_only_mode = False
        yaw_visual_smooth = None
        last_yaw_assert_t = time.time()
        nav_active = True
        # Leg 1 (start->wp1) is flown at takeoff altitude for safe first-fix.
        current_leg_alt = cfg.takeoff_alt
        yaw_cmd_rad = math.radians(LOCK_YAW_NORTH_DEG if cfg.lock_yaw_north else 0.0)
        def safe_loiter(reason):
            self._event("safety_loiter", reason=reason)
            try: cmd_queue.put({"type": "set_mode", "mode": "LOITER"})
            except Exception: pass
            time.sleep(3.0)

        self._event("phase", phase="NAVIGATING")

        while nav_active:
            if self.should_abort():
                safe_loiter("operator STOP"); break

            telem_fresh = False
            while not telem_queue.empty():
                try:
                    item = telem_queue.get_nowait()
                    if "statustext" in item:
                        self._event("ap_status", text=item["statustext"],
                                    severity=item.get("severity", 6))
                    elif "error" not in item:
                        latest = item; telem_fresh = True
                except Exception:
                    break
            if telem_fresh:
                last_telem_ts = time.time()

            if time.time() - last_telem_ts > TELEM_TIMEOUT_S:
                safe_loiter(f"no telemetry {TELEM_TIMEOUT_S}s"); break
            if camera_stop.is_set():
                safe_loiter("camera lost"); break
            if not self.mav_proc.is_alive():
                safe_loiter("MAVLink process died"); break
            if latest["agl_m"] < AGL_THRESH:
                safe_loiter(f"AGL {latest['agl_m']:.1f} < {AGL_THRESH}"); break

            ekf.predict(latest["vn"], latest["ve"])

            frame = get_latest_frame(frame_q, timeout=2.0)
            if frame is None:
                consec_fail += 1
                self._log_raw(f"[{frame_idx:4d}] no camera frame received "
                              f"(consec_fail={consec_fail}/{MAX_CONSEC_FAILURES}) "
                              f"agl={latest['agl_m']:.1f}m "
                              f"mode={mode_name(latest.get('custom_mode'))}")
                if consec_fail >= MAX_CONSEC_FAILURES:
                    safe_loiter(f"{consec_fail} frame failures"); break
                continue

            search_lat, search_lon = ekf.get_position()
            agl_m = latest["agl_m"]

            t0 = time.time()
            result = localize_image(frame, tif_src, search_lat, search_lon, agl_m)
            lat_ms = (time.time() - t0) * 1000

            pred_lat = pred_lon = float("nan")
            n_matches = n_inliers = 0
            mahal = float("nan")
            yaw_visual = yaw_cond = float("nan")
            loc_status = "failed"

            # CHANGED: min-valid-altitude floor also enforced in cruise. A fix
            # taken below the floor is discarded regardless of quality.
            if agl_m < cfg.min_valid_alt_m:
                loc_status = "below_alt_floor"
                consec_fail += 1
            elif result is None:
                consec_fail += 1
            else:
                pred_lat = result["loc_lat"]; pred_lon = result["loc_lon"]
                n_matches = result["n_matches"]; n_inliers = result["n_inliers"]
                yaw_visual = result["yaw_visual_deg"]; yaw_cond = result["yaw_cond"]

                if yaw_cond <= YAW_QUALITY_MAX_COND:
                    if yaw_visual_smooth is None:
                        yaw_visual_smooth = yaw_visual
                    else:
                        diff = angle_wrap_180(yaw_visual - yaw_visual_smooth)
                        yaw_visual_smooth = (yaw_visual_smooth
                                             + YAW_EMA_ALPHA * diff) % 360.0

                jumped = False
                if last_accepted_pred is not None:
                    jump_m = haversine_m(*last_accepted_pred, pred_lat, pred_lon)
                    if jump_m > OUTLIER_REJECT_M:
                        jumped = True
                        loc_status = f"jump_reject({jump_m:.1f}m)"
                        consec_fail += 1
                if not jumped:
                    accepted, mahal = ekf.update(pred_lat, pred_lon, n_inliers)
                    if accepted:
                        loc_status = "accepted"; consec_fail = 0
                        last_accepted_pred = (pred_lat, pred_lon)
                    else:
                        loc_status = f"mahal_reject({mahal:.1f})"
                        consec_fail += 1

            if consec_fail >= MAX_CONSEC_FAILURES:
                safe_loiter(f"{consec_fail} localisation fails"); break

            ekf_lat, ekf_lon = ekf.get_position()
            now = time.time()

            # ── GPS course-correction backstop — ONLY if policy allows ───────
            cur_error_m = haversine_m(latest["lat"], latest["lon"],
                                      ekf_lat, ekf_lon)
            if cfg.policy.gps_correction and not gps_only_mode:
                if cur_error_m > GPS_SANITY_ERROR_M:
                    if high_error_start is None:
                        high_error_start = now
                    elif now - high_error_start > GPS_SANITY_DURATION_S:
                        self._event("gps_takeover",
                                    error_m=cur_error_m,
                                    reason="sustained EKF-vs-GPS bias")
                        ekf.reset_to_gps(latest["lat"], latest["lon"])
                        last_accepted_pred = (latest["lat"], latest["lon"])
                        gps_only_mode = True
                else:
                    high_error_start = None
            if gps_only_mode:
                ekf.reset_to_gps(latest["lat"], latest["lon"])
                ekf_lat, ekf_lon = ekf.get_position()

            heading, speed, state, cte, dist_b, advanced = nav.compute(
                ekf_lat, ekf_lon, now)

            if advanced:
                self._event("waypoint_reached",
                            leg=nav.leg_number - 1,
                            total=nav.total_legs,
                            next_target=list(nav.target))
                # Per-leg altitude: step-change at the waypoint, then level.
                new_alt = nav.leg_alt
                if abs(new_alt - current_leg_alt) > 1.0:
                    self._change_altitude(new_alt, current_leg_alt, cfg,
                                          cmd_queue, telem_queue, latest,
                                          yaw_cmd_rad)
                    current_leg_alt = new_alt
                self._event("leg_started", leg=nav.leg_number,
                            speed=nav.leg_speed, alt=current_leg_alt)

            # ── GPS-arrival backstop — ONLY if policy allows ─────────────────
            if cfg.policy.gps_correction:
                gps_dist_b = haversine_m(latest["lat"], latest["lon"],
                                         nav.target[0], nav.target[1])
                if (gps_dist_b < GPS_ARRIVAL_RADIUS_M and state != "LAND"
                        and nav.is_final_leg):
                    self._event("gps_arrival", dist=gps_dist_b)
                    state = "LAND"

            if state == "GPS_RESET" and cfg.policy.gps_correction:
                gps_reset_count += 1
                self._event("gps_reset", count=gps_reset_count)
                ekf.reset_to_gps(latest["lat"], latest["lon"])
                last_accepted_pred = (latest["lat"], latest["lon"])
                ekf_lat, ekf_lon = ekf.get_position()
                heading, speed, state, cte, dist_b, _ = nav.compute(
                    ekf_lat, ekf_lon, now)

            if state == "LAND":
                self._event("landing", dist_to_final=dist_b)
                cmd_queue.put({"type": "set_mode", "mode": "LAND"})
                land_deadline = time.time() + 90.0
                while time.time() < land_deadline:
                    while not telem_queue.empty():
                        try:
                            item = telem_queue.get_nowait()
                            if "error" not in item and "statustext" not in item:
                                latest = item
                        except Exception:
                            break
                    if latest["agl_m"] < 1.0:
                        self._event("landed", agl=latest["agl_m"])
                        break
                    self.on_telem({"phase": "landing", "agl_m": latest["agl_m"]})
                    # CHANGED: same per-second AGL/mode logging as the other
                    # phases -- this loop already ran at 1s cadence but only
                    # sent telemetry to the GUI (on_telem), never the txt log.
                    self._log_raw(f"[telemetry] agl={latest['agl_m']:.1f}m "
                                  f"mode={mode_name(latest.get('custom_mode'))} "
                                  f"armed={latest.get('is_armed')} (landing)")
                    time.sleep(1.0)
                nav_active = False
                break

            # ── Heading rate limiter ─────────────────────────────────────────
            max_change = math.radians(MAX_HEADING_CHANGE_DEG)
            hdiff = heading - prev_heading
            hdiff = (hdiff + math.pi) % (2*math.pi) - math.pi
            if abs(hdiff) > max_change:
                heading = prev_heading + max_change * (1 if hdiff > 0 else -1)
                heading %= (2*math.pi)
            prev_heading = heading

            # ── Velocity command (NED + locked yaw north) ────────────────────
            vn_cmd = speed * math.cos(heading)
            ve_cmd = speed * math.sin(heading)
            cmd_queue.put({"type": "velocity", "vn": vn_cmd, "ve": ve_cmd,
                           "vd": 0.0,
                           "yaw": math.radians(LOCK_YAW_NORTH_DEG
                                               if cfg.lock_yaw_north else 0.0)})

            if cfg.lock_yaw_north and (now - last_yaw_assert_t) > YAW_REASSERT_PERIOD_S:
                cmd_queue.put({"type": "condition_yaw",
                               "yaw_deg": LOCK_YAW_NORTH_DEG,
                               "rate_deg_s": YAW_LOCK_RATE_DEG_S})
                last_yaw_assert_t = now

            # ── Telemetry to GUI ─────────────────────────────────────────────
            self.on_telem({
                "phase": "nav", "frame": frame_idx, "state": state,
                "leg": nav.leg_number, "total_legs": nav.total_legs,
                "ekf_lat": ekf_lat, "ekf_lon": ekf_lon,
                "gps_lat": latest["lat"], "gps_lon": latest["lon"],
                "pred_lat": pred_lat, "pred_lon": pred_lon,
                "target_lat": nav.target[0], "target_lon": nav.target[1],
                "cte_m": cte, "dist_to_target_m": dist_b,
                "error_m": cur_error_m, "agl_m": agl_m,
                "heading_deg": math.degrees(heading),
                "n_inliers": n_inliers, "status": loc_status,
                "latency_ms": lat_ms, "gps_only": gps_only_mode,
            })

            results.append({
                "frame_idx": frame_idx, "timestamp": latest["ts_iso"],
                "state": state, "leg": nav.leg_number,
                "gps_lat": latest["lat"], "gps_lon": latest["lon"],
                "ekf_lat": ekf_lat, "ekf_lon": ekf_lon,
                "pred_lat": pred_lat, "pred_lon": pred_lon,
                "target_lat": nav.target[0], "target_lon": nav.target[1],
                "cte_m": cte, "dist_to_target_m": dist_b, "error_m": cur_error_m,
                "heading_cmd_deg": math.degrees(heading), "speed_cmd": speed,
                "vn_cmd": vn_cmd, "ve_cmd": ve_cmd,
                "yaw_visual_deg": yaw_visual, "n_inliers": n_inliers,
                "n_matches": n_matches, "mahal": mahal, "agl_m": agl_m,
                "latency_ms": lat_ms, "status": loc_status,
                "gps_only": gps_only_mode,
            })
            frame_idx += 1

            pred_str = (f"pred=({pred_lat:.6f},{pred_lon:.6f})"
                        if not math.isnan(pred_lat) else "pred=(nan,nan)")
            self._log_raw(
                f"[{frame_idx:4d}] {state:9s} | leg={nav.leg_number}/"
                f"{nav.total_legs} | CTE={cte:+6.1f}m | dist={dist_b:6.1f}m | "
                f"{pred_str} | EKF=({ekf_lat:.6f},{ekf_lon:.6f}) | "
                f"hdg={math.degrees(heading):5.1f} deg vn={vn_cmd:+5.2f} "
                f"ve={ve_cmd:+5.2f} | err={cur_error_m:.1f}m | "
                f"n_inl={n_inliers} n_match={n_matches} | {loc_status} | "
                f"agl={agl_m:.1f}m mode={mode_name(latest.get('custom_mode'))} | "
                f"{lat_ms:.0f}ms"
                + (" | GPS_ONLY" if gps_only_mode else ""))

            if frame_idx % 5 == 0:
                pd.DataFrame(results)[CSV_COLS].to_csv(csv_path, index=False)

        if results:
            pd.DataFrame(results)[CSV_COLS].to_csv(csv_path, index=False)
        self._event("mission_complete", frames=len(results),
                    gps_resets=gps_reset_count, csv=csv_path)

    # -------------------------------------------------------------------------
    def _abort_land(self, cmd_queue, reason):
        self._event("abort", reason=reason)
        try: cmd_queue.put({"type": "set_mode", "mode": "LAND"})
        except Exception: pass
        self._cleanup()

    def _cleanup(self):
        if self.mav_stop is not None:
            self.mav_stop.set()
        camera_stop.set()
        if self.mav_proc is not None:
            self.mav_proc.join(timeout=3.0)
            if self.mav_proc.is_alive():
                self.mav_proc.kill(); self.mav_proc.join()
        if self.gimbal is not None:
            self.gimbal.stop()
        self._event("engine_stopped")
        if self.log_fh is not None:
            try: self.log_fh.close()
            except Exception: pass
            self.log_fh = None


if __name__ == "__main__":
    # Standalone demo harness (won't fly without hardware).
    mp.set_start_method("spawn", force=True)
    demo = {
        "waypoints": [[17.602611, 78.126857], [17.603200, 78.127500]],
        "mode": "safety", "takeoff_alt": 120.0, "nav_speed": 2.5,
        "min_valid_alt_m": 90.0,
        "tif_path": sys.argv[1] if len(sys.argv) > 1 else "map.tif",
    }
    eng = FlightEngine(MissionConfig(demo),
                       on_event=lambda e: print("EVENT", e.get("kind"), e),
                       on_telem=lambda t: None)
    eng.run()
