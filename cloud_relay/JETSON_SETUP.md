# Setting up the cloud relay on the Jetson Orin

Operational runbook for turning on `cloud_relay.py` on a Jetson that's
already running WildNav locally (i.e. `install.sh` has been run before, the
`lightglue` conda env exists, and `drone_agent.py` already works over local
WiFi per the top-level [README.md](../README.md)). If any of that isn't true
yet, do that first — this doc only covers the *cloud* piece added in
[DESIGN.md](DESIGN.md).

Assumes the 5G unit is already mounted and the Jetson joins it as a normal
WiFi network (per DESIGN.md §9) — nothing below configures cellular hardware,
only the software that uses whatever internet connection is already there.

## 0. Pre-flight checks

```bash
# Confirm the Jetson actually has internet through the 5G WiFi unit:
ping -c 3 8.8.8.8
curl -sI https://www.google.com | head -1
```
If these fail, fix connectivity first (join the 5G unit's WiFi network,
`nmcli device wifi connect <ssid>` or however you normally do it) — nothing
past this point will work without it.

## 1. Get the new code onto the Jetson

The files added: `server/event_bus.py`, `server/cloud_relay.py`,
`server/cloud_relay.env.example`, `web/cloud.html`, `web/static/cloud.js`,
plus edits to `server/drone_agent.py` and `install.sh`.

- **If you're editing directly on the Jetson** (this repo checkout *is* the
  Jetson), skip to step 2.
- **Otherwise**, get these files onto the Jetson the same way you got the
  rest of the project there (`git pull`, `scp`, USB, etc.), into the same
  source tree `install.sh` runs from.

Then run the installer again — it's idempotent, safe to re-run, and its
existing copy step (`cp -r server/ web/ -> $BASE`, default `$BASE=~/wild_opt`)
will pick up all the new files automatically, plus installs the new
`paho-mqtt` dependency now on the `PIP_PKGS` list:
```bash
bash install.sh
```
If you'd rather not re-run the whole installer, just do the two things it
would've done for this feature:
```bash
conda activate lightglue
pip install paho-mqtt
cp server/event_bus.py server/cloud_relay.py server/cloud_relay.env.example "$BASE/server/"
cp web/cloud.html "$BASE/web/"
cp web/static/cloud.js "$BASE/web/static/"
cp server/drone_agent.py "$BASE/server/"   # picks up the wiring changes
```
(`$BASE` = wherever `install.sh` installs to — default `~/wild_opt`, or
whatever you set `WILDNAV_BASE` to.)

## 2. Broker credentials

Already done if you followed the earlier setup — you should have from
HiveMQ Cloud: a **cluster host**, and two credential sets
(`drone-alpha-publisher`, `dashboard-readonly`). If not, see the HiveMQ Cloud
steps from earlier in this conversation before continuing.

## 3. Create `cloud_relay.env` — **directly on the Jetson, not in your dev tree**

This file holds real secrets (the publisher password). Create it fresh at
the *installed* location so it never ends up in whatever you use to sync
code to/from the Jetson:
```bash
cd "$BASE/server"      # e.g. ~/wild_opt/server
cp cloud_relay.env.example cloud_relay.env
nano cloud_relay.env   # or vim/whatever
```
Fill in:
```
WILDNAV_MQTT_HOST=xxxxxxxx.s1.eu.hivemq.cloud
WILDNAV_MQTT_PORT=8883
WILDNAV_MQTT_USERNAME=drone-alpha-publisher
WILDNAV_MQTT_PASSWORD=<that credential's real password>
WILDNAV_DRONE_ID=drone-alpha
```

## 4. Run it manually first — verify before trusting autostart

```bash
cd "$BASE/server"
set -a; source cloud_relay.env; set +a
conda activate lightglue
python3 drone_agent.py --name drone-alpha
```
Expected output within a couple seconds:
```
[cloud_relay] enabled — shipping telemetry/logs to wildnav/drone-alpha/events on xxxxxxxx.s1.eu.hivemq.cloud:8883
[cloud_relay] connected to broker
```
If you *don't* see the first line at all, `WILDNAV_MQTT_HOST` isn't set in
the running process's environment (env file not sourced, or you `source`d it
in a different shell than the one running `python3`). If you see the first
line but not "connected", it's a broker/network problem — see Troubleshooting.

## 5. Deploy the dashboard so it's reachable from anywhere (one-time)

The dashboard has no server-side dependency — it only talks directly to the
broker from the browser — so it doesn't need to be served by the Jetson at
all. Serving it from `drone_agent.py` (`http://<drone-ip>:8000/cloud.html`)
only works on the drone's local WiFi, which defeats the point. It lives in
its own top-level **`dashboard/`** folder (separate from `web/`, which stays
the local-WiFi mission-control app) specifically so it deploys as a clean
standalone site with no path/redirect gymnastics — `dashboard/index.html` is
already the site's root page.

1. Go to [app.netlify.com/drop](https://app.netlify.com/drop) (sign up free
   if you haven't).
2. Drag the **`dashboard/`** folder (not `web/`) from this project onto the
   page.
3. Netlify gives you a public URL (e.g. `random-name.netlify.app`) — that's
   your dashboard, reachable from any internet connection, no VPN or local
   WiFi needed, and the root URL loads it directly.

Nothing sensitive is in these files — broker credentials are typed into the
page and stored only in that browser's `localStorage`, never embedded in the
deployed files — so it's safe to make this URL public.

`http://<drone-ip>:8000/cloud.html` still works too, as a local-WiFi
fallback (same content, served by `drone_agent.py` from `web/cloud.html`) —
it's just no longer the primary way to reach the dashboard. If you already
deployed the `web/` folder to Netlify earlier and hit a redirect issue,
redeploy with `dashboard/` instead — it doesn't depend on a `_redirects`
rule at all.

## 6. Verify

Open your new Netlify URL. Gear icon → fill in:
- Host: same cluster host
- Port: `8884`, path `/mqtt`, TLS checked
- Credentials: `dashboard-readonly`

Connect. You should see the connection dot go green and `LINK: ONLINE` /
last-seen update almost immediately (heartbeat runs at 5Hz) even with no
mission running. Run a mission (or just the setup script) and you should see
live telemetry cards and a scrolling event/log panel — from any network, not
just the drone's WiFi.

## 7. Make it survive a reboot (autostart)

If you'd already installed the `wildnav-agent` systemd service **before**
today's `install.sh` change, re-run `install.sh` (or just the systemd step)
so the service picks up the new `EnvironmentFile=-$SERVER_DIR/cloud_relay.env`
line — without that, the autostart path would silently never load your
`cloud_relay.env` even though manual runs work fine.
```bash
bash install.sh          # re-installs the systemd unit with the new line
sudo systemctl restart wildnav-agent
sudo systemctl status wildnav-agent   # should be active (running)
journalctl -u wildnav-agent -n 30 --no-pager   # look for the [cloud_relay] lines
```
Or toggle it from the GUI's settings panel ("Autostart on boot") — same
service either way.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| No `[cloud_relay]` lines at all | `WILDNAV_MQTT_HOST` not in the environment the process actually saw — check you sourced the file in the *same* shell, or for systemd check `EnvironmentFile` path is correct and the file exists with correct permissions |
| `paho-mqtt not installed` | `pip install paho-mqtt` into the `lightglue` env specifically, not system python |
| Connects then immediately disconnects, repeatedly | Wrong port/TLS combination (8883 needs TLS, code auto-enables `tls_set()` only when `WILDNAV_MQTT_PORT=8883` exactly — a nonstandard port will skip TLS) |
| Auth failure / connection refused | Username/password typo, or credential's permissions don't include publish on `wildnav/#` |
| Dashboard shows "connecting…" forever | Port 8884/wss blocked by the network the *browser* is on (some corporate/campus WiFi blocks nonstandard ports) — try from a different network, or mobile data |
| Dashboard connects but never goes ONLINE | Drone-side isn't actually connected (check step 4's terminal first) or `WILDNAV_DRONE_ID` doesn't match what you picked in the dashboard's drone selector |
| TLS handshake fails only on the Jetson | Check the Jetson's system clock — TLS cert validation fails if the clock is badly wrong (common right after a fresh boot with no RTC/NTP sync yet) |
