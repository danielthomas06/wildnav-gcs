"""
cloud_relay.py — ships the same events the local WebSocket already shows
(telemetry, setup/tool logs, mission events) plus this process's own
stdout/stderr to a cloud MQTT broker, so they're visible on a dashboard
from anywhere, without local WiFi range or an SSH session.

Monitoring only (v1) — see cloud_relay/DESIGN.md. Nothing subscribes to a
command topic in this version; the drone accepts no input from the cloud.

Safe by default: if WILDNAV_MQTT_HOST isn't set, start() no-ops and prints
one line. Existing local-only deployments are unaffected either way, and
paho-mqtt is only imported lazily inside start() so it isn't a hard
dependency until the relay is actually enabled.

Env vars:
    WILDNAV_MQTT_HOST       broker hostname — unset disables the relay entirely
    WILDNAV_MQTT_PORT       default 8883 (TLS)
    WILDNAV_MQTT_USERNAME
    WILDNAV_MQTT_PASSWORD
    WILDNAV_DRONE_ID        defaults to the agent's drone name
"""
import json
import os
import queue
import sys
import threading
import time

import event_bus

TOPIC_EVENTS = "wildnav/{id}/events"
TOPIC_STATUS = "wildnav/{id}/status"
TOPIC_LWT = "wildnav/{id}/lwt"
TOPIC_MISSION = "wildnav/{id}/mission"

_HEARTBEAT_PERIOD_S = 0.2  # 5 Hz


def _json_default(o):
    """Best-effort fallback for numpy scalars etc. that show up in telemetry
    dicts — never let a serialization hiccup take down the publish loop."""
    for attr in ("item", "tolist"):
        if hasattr(o, attr):
            try:
                return getattr(o, attr)()
            except Exception:
                pass
    return str(o)


class _TeeWriter:
    """Duplicates writes to the real stream and to a line callback.

    Only ever installed on THIS process's stdout/stderr (the FastAPI agent).
    Deliberately NOT installed inside the mission subprocess — see
    restore_std_streams() — so flight_engine's high-rate per-frame debug
    prints (_log_raw) never reach the cloud, matching the existing local
    design that already keeps that firehose out of the WS event stream.
    """
    def __init__(self, real, stream_name, on_line):
        self._real = real
        self._stream = stream_name
        self._on_line = on_line
        self._buf = ""

    def write(self, s):
        self._real.write(s)
        self._buf += s
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line:
                try:
                    self._on_line(self._stream, line)
                except Exception:
                    pass
        return len(s)

    def flush(self):
        self._real.flush()

    def isatty(self):
        return self._real.isatty()


def restore_std_streams():
    """Call at the top of the mission subprocess, before flight_engine is
    imported — undoes the Tee installed in the parent (inherited via fork)
    so the subprocess's own prints go straight to the terminal, uncaptured."""
    if isinstance(sys.stdout, _TeeWriter):
        sys.stdout = sys.stdout._real
    if isinstance(sys.stderr, _TeeWriter):
        sys.stderr = sys.stderr._real


class CloudRelay:
    def __init__(self):
        self._client = None
        self._publish_thread = None
        self._heartbeat_thread = None
        self._running = False
        self._sub_q = None
        self._drone_id = None
        self._seq = 0

    def _next_seq(self):
        self._seq += 1
        return self._seq

    def start(self, drone_id: str):
        host = os.environ.get("WILDNAV_MQTT_HOST")
        if not host:
            print("[cloud_relay] WILDNAV_MQTT_HOST not set — cloud relay disabled")
            return False
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            print("[cloud_relay] paho-mqtt not installed (pip install paho-mqtt) "
                  "— cloud relay disabled")
            return False

        self._drone_id = os.environ.get("WILDNAV_DRONE_ID", drone_id)
        port = int(os.environ.get("WILDNAV_MQTT_PORT", "8883"))
        username = os.environ.get("WILDNAV_MQTT_USERNAME")
        password = os.environ.get("WILDNAV_MQTT_PASSWORD")
        client_id = f"wildnav-{self._drone_id}"

        try:
            client = mqtt.Client(
                client_id=client_id,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                clean_session=False,
            )
        except AttributeError:
            client = mqtt.Client(client_id=client_id, clean_session=False)  # paho-mqtt <2.0

        if username:
            client.username_pw_set(username, password)
        if port == 8883:
            client.tls_set()
        client.reconnect_delay_set(min_delay=1, max_delay=30)
        client.max_queued_messages_set(2000)

        lwt_topic = TOPIC_LWT.format(id=self._drone_id)
        client.will_set(lwt_topic, payload=json.dumps({"status": "offline"}),
                         qos=1, retain=True)
        client.on_connect = self._make_on_connect(client)
        client.on_disconnect = lambda *a, **k: print(
            "[cloud_relay] disconnected from broker (will retry)")
        # Without this, a failed/stuck connection attempt (bad DNS, network
        # unreachable, TLS handshake failure...) is completely silent — no
        # on_connect (never fires), no on_disconnect (only fires after a
        # previously-established connection drops, not a failed first
        # attempt). Filtered to warning/error only: this process's stdout is
        # captured and shipped to the cloud (see _install_stdout_capture),
        # so full debug-level MQTT chatter would flood that stream.
        client.on_log = self._make_on_log()

        try:
            client.connect_async(host, port, keepalive=20)
        except Exception as e:
            print(f"[cloud_relay] connect_async failed: {e}")
            return False
        client.loop_start()
        self._client = client
        threading.Thread(target=self._connect_watchdog, args=(client,), daemon=True).start()

        self._sub_q = event_bus.bus.subscribe(maxsize=4000)
        self._running = True
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

        self._install_stdout_capture()

        print(f"[cloud_relay] enabled — shipping telemetry/logs to "
              f"wildnav/{self._drone_id}/events on {host}:{port}")
        return True

    def _make_on_connect(self, client):
        def _on_connect(*args, **kwargs):
            print(f"[cloud_relay] connected to broker")
            payload = json.dumps({"status": "online", "ts": time.time()})
            status_topic = TOPIC_STATUS.format(id=self._drone_id)
            client.publish(status_topic, payload, qos=1, retain=True)
            # "Birth" message on the SAME topic the Will ("death" message)
            # uses. A prior ungraceful disconnect leaves a retained offline
            # LWT message sitting on the broker forever — nothing else ever
            # overwrites it, so a fresh dashboard subscribe would see stale
            # "offline" even though we just connected fine. Publishing here
            # too means the LWT topic always reflects current truth: this
            # "online" on a clean (re)connect, or the broker's own "offline"
            # if we ever drop off uncleanly again.
            lwt_topic = TOPIC_LWT.format(id=self._drone_id)
            client.publish(lwt_topic, payload, qos=1, retain=True)
        return _on_connect

    def _make_on_log(self):
        import paho.mqtt.client as mqtt

        def _on_log(client, userdata, level, buf):
            if level in (mqtt.MQTT_LOG_WARNING, mqtt.MQTT_LOG_ERR):
                print(f"[cloud_relay][mqtt] {buf}")
        return _on_log

    def _connect_watchdog(self, client, timeout_s=8.0):
        """If nothing's connected within timeout_s of start(), say so — the
        alternative is total silence (see the on_log comment above), which
        is indistinguishable from 'still trying' and 'gave up'."""
        time.sleep(timeout_s)
        if self._running and not client.is_connected():
            print(f"[cloud_relay] still not connected after {timeout_s:.0f}s — "
                  f"check the Jetson has internet right now (ping/curl a known "
                  f"host), and that host/port/credentials in cloud_relay.env "
                  f"are correct")

    def _install_stdout_capture(self):
        def on_line(stream, line):
            event_bus.bus.publish({
                "kind": "proc_log", "stream": stream, "line": line, "ts": time.time(),
            })
        sys.stdout = _TeeWriter(sys.stdout, "stdout", on_line)
        sys.stderr = _TeeWriter(sys.stderr, "stderr", on_line)

    def _publish_loop(self):
        events_topic = TOPIC_EVENTS.format(id=self._drone_id)
        while self._running:
            try:
                event = self._sub_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if event.get("kind") == "heartbeat":
                # drone_agent.py's _event_pump publishes this to the bus at
                # 10Hz purely for the local WebSocket's UI (mission_active /
                # trt_running indicator) — cloud_relay has its own much
                # cheaper heartbeat on the retained `status` topic (every
                # _HEARTBEAT_PERIOD_S) that already covers "is it alive" for
                # the cloud side. Forwarding this one too would ship 10
                # msg/s continuously, 24/7, for no benefit.
                continue
            envelope = {"ts": time.time(), "seq": self._next_seq(), "event": event}
            try:
                self._client.publish(events_topic, json.dumps(envelope, default=_json_default),
                                      qos=1)
            except Exception:
                pass
            if event.get("kind") == "mission_config":
                # Also retained, on its own topic: a dashboard connecting
                # mid-mission needs the waypoint list immediately, not just
                # whenever the next telemetry tick happens to mention it —
                # events itself is retained=false (see DESIGN.md §5.2).
                mission_topic = TOPIC_MISSION.format(id=self._drone_id)
                try:
                    self._client.publish(mission_topic,
                                          json.dumps(event, default=_json_default),
                                          qos=1, retain=True)
                except Exception:
                    pass

    def _heartbeat_loop(self):
        status_topic = TOPIC_STATUS.format(id=self._drone_id)
        while self._running:
            try:
                self._client.publish(status_topic, json.dumps({
                    "status": "online", "ts": time.time(),
                }), qos=1, retain=True)
            except Exception:
                pass
            time.sleep(_HEARTBEAT_PERIOD_S)

    def stop(self):
        self._running = False
        if self._client is not None:
            try:
                payload = json.dumps({"status": "offline", "ts": time.time()})
                status_topic = TOPIC_STATUS.format(id=self._drone_id)
                lwt_topic = TOPIC_LWT.format(id=self._drone_id)
                self._client.publish(status_topic, payload, qos=1, retain=True)
                self._client.publish(lwt_topic, payload, qos=1, retain=True)
                self._client.disconnect()
                self._client.loop_stop()
            except Exception:
                pass
        if self._sub_q is not None:
            event_bus.bus.unsubscribe(self._sub_q)


relay = CloudRelay()
