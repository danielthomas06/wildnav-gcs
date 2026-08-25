# WildNav Ground Control

A client/server ground-control layer on top of your single-file GNSS-denied
navigation script. A web GUI (laptop or phone, zero install) discovers drones
on the WiFi, plans multi-waypoint missions with four GPS-usage modes, and
launches flights on the Jetson.

```
┌─────────────────┐      WiFi       ┌──────────────────────────────┐
│  Laptop / Phone │◄───────────────►│  Jetson (on the drone)       │
│  browser (GUI)  │  HTTP + WS      │  drone_agent.py (FastAPI)    │
└─────────────────┘                 │    ├─ discovery beacon       │
                                    │    ├─ serves web GUI         │
                                    │    ├─ runs setup script      │
                                    │    └─ flight_engine.py       │
                                    │         └─ MAVLink + LightGlue│
                                    └──────────────────────────────┘
```

## Files

**server/** (runs on the Jetson)
- `drone_agent.py` — FastAPI server: discovery, serves GUI, runs setup, launches flight, streams telemetry over WebSocket. **This is what you run on the drone.**
- `flight_engine.py` — multi-waypoint, mode-aware refactor of `navigate_north.py`. Vertical-first takeoff, first-fix acquisition, sequential waypoints, all safety layers preserved.
- `nav_modes.py` — the four-mode 2×2 policy matrix (seed_from_gps × gps_correction).
- `discovery_hub.py` — *optional* laptop-side drone picker (only if you want a landing page before knowing any IP).

**web/** (served by the agent — you never run these directly)
- `index.html`, `static/app.css`, `static/app.js` — the mission-control GUI.

## The four modes

Two independent GPS uses: **(a)** how the start location is obtained, **(b)** whether GPS may take over course correction.

| Mode | Start seed | Course correction |
|---|---|---|
| **safety** | GPS | GPS backstop on (= original `navigate_north.py`) |
| **cold_start** | first **visual** fix @ 120 m | GPS backstop on |
| **safe_start** | GPS | vision-only |
| **full_gps_denied** | first **visual** fix @ 120 m | vision-only |

**First-fix acquisition** (cold_start / full_gps_denied): the drone climbs
*vertically* to 120 m directly above its physical start, then the first valid
visual localization becomes waypoint₁. An optional "approx start" pin narrows
that search; without it, a full-map tile search runs. Any localization below
`min_valid_alt_m` (default 90 m, GUI-settable) is discarded — so the first fix
is guaranteed to be taken at cruise altitude.

## Deploy on the Jetson

1. Copy the `server/` files into your working dir:
   ```bash
   cp server/*.py ~/Ashutosh/wildnav_opt/
   cd ~/Ashutosh/wildnav_opt/
   ```
   (The agent also needs the `web/` folder next to it — keep the repo layout,
   or set `WEB_DIR` in `drone_agent.py` to wherever you put `web/`.)

2. Install the agent deps (one time):
   ```bash
   pip install fastapi "uvicorn[standard]" zeroconf python-multipart
   ```

3. Run the agent:
   ```bash
   python3 drone_agent.py --name drone-alpha
   ```
   It prints a URL like `http://192.168.1.42:8000`.

   The mission map is uploaded through the GUI at flight-planning time — there
   is no hardcoded TIF. That single georeferenced GeoTIFF is used for **both**
   operator waypoint-picking and the drone's LightGlue localisation.

> The agent runs your exact setup block on mission start (cd wild_opt →
> `conda activate lightglue` → clone+`pip install -e .` LightGlue → cd
> wildnav_opt), streaming the output to the GUI, then starts the flight.

## Fly

1. On your laptop or phone (same WiFi), open the agent URL — or run
   `python3 discovery_hub.py` and open `http://localhost:8080` to auto-find
   drones.
2. **01 Select drone** → pick it from the list (or type its IP).
3. **02 Plan mission**:
   - Upload your georeferenced **GeoTIFF** of the area. It's validated on the
     server (must have a CRS + geotransform); a bad file is rejected inline and
     no mission can launch. The footprint is drawn as a coordinate-accurate
     plane you drop waypoints onto.
   - Drop waypoints on the footprint, and/or type lat/lon and **Add**.
   - Pick a mode (the flag pills show exactly what each does).
   - For visual-seed modes, optionally set an **approx start** pin.
   - Set speed and min-valid-altitude; altitude is 120 m.
   - **Start mission.**
4. **03 Mission live** — telemetry cards + event log stream in real time.
   **STOP** commands the drone to LOITER/LAND.

## Notes / things to confirm

- **One map, two jobs:** the GeoTIFF you upload is the *same* raster the drone
  localizes against — there is no separate hardcoded TIF and no manual corner
  calibration. The server reads the CRS + transform, reprojects the footprint
  corners to WGS84, and hands them to the GUI so a pixel you click maps to the
  exact lat/lon the drone will compute.
- **Browser can't paint GeoTIFF pixels:** the map panel shows the TIF's
  footprint + lat/lon graticule (coordinate-accurate), not the raster imagery.
  The drone does the actual image matching server-side where rasterio lives. If
  you want the imagery visible in-browser too, that needs a client-side decoder
  (geotiff.js) — say the word and I'll add it.
- **Upload format:** must be a georeferenced GeoTIFF. Anything else (plain
  PNG/JPG, or a TIF with no geotags) is rejected with an inline error.
- **Discovery across subnets / AP isolation:** if the UDP beacon is blocked by
  your access point, the drone still won't appear automatically — use the
  manual IP box. mDNS is also advertised as a fallback.
- The engine runs in its own process (isolates torch/CV + the MAVLink spawn);
  `STOP` sets an abort flag the loop polls every iteration.
