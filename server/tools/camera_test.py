"""
camera_test.py — Standalone SIYI RTSP camera diagnostic. NO drone, NO MAVLink,
NO gimbal, NO takeoff — just the camera pipeline in isolation (plus, if
--stress is used, a synthetic CPU/GPU load to reproduce mission-time
contention), so a "Stream lost" / dropout problem can be reproduced and
logged on its own, separate from everything else that runs during a real
mission.

Reproduces the exact same ffmpeg pipeline flight_engine.py's camera_thread_fn
uses (software HEVC decode), and optionally a hardware-decode variant
(Jetson NVDEC via nvv4l2dec) for side-by-side comparison, since the SIYI
stream is HEVC despite the "main.264" URL, and this ffmpeg build has
--enable-nvv4l2dec compiled in but never uses it in production.

An unloaded camera-only test (no --stress) was already clean in both decode
modes -- zero drops over 3 minutes each. --stress exists to test the real
suspect: does the camera destabilize once something else is competing for
CPU/GPU at the same time, the way SuperPoint+LightGlue inference does during
an actual mission?

Usage:
    python3 camera_test.py                          # software decode, 120s
    python3 camera_test.py --mode hardware           # hardware decode, 120s
    python3 camera_test.py --mode both --duration 180  # both, back to back
    python3 camera_test.py --stress cpu              # + pure CPU busy-loop load
    python3 camera_test.py --stress vision           # + real SuperPoint+LightGlue
                                                      #   inference loop (needs
                                                      #   torch/lightglue/cv2 --
                                                      #   run in the same conda
                                                      #   env flight_engine.py uses)
    python3 camera_test.py --mode both --stress vision --duration 180

Output (in --out-dir, default ./camera_test_logs/):
    camera_test_<mode>_<timestamp>.txt   — detailed event log (same style as
                                            flight_engine.py's [CAM]/[FFmpeg])
    camera_test_<mode>_<timestamp>.csv   — one row per frame received
                                            (timestamp, frame_number, gap_s)
    A summary is printed at the end of each mode and, if --mode both, a
    side-by-side comparison at the very end.

Share the .txt (and .csv if useful) files back for analysis, same as the
mission nav_log_*.txt/.csv pairs.
"""
import argparse
import datetime
import os
import subprocess
import sys
import threading
import time

# ── Defaults — match flight_engine.py's production config ───────────────────
SIYI_IP       = "192.168.144.25"
RTSP_URL      = f"rtsp://{SIYI_IP}:8554/main.264"
CAMERA_FPS    = 7
CROP_W        = 1280
CROP_H        = 720
MAX_RECONNECTS = 999
RECONNECT_DELAY = 3.0
GAP_WARN_S    = 1.0     # log any inter-frame gap longer than this


def ts():
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


class Logger:
    """Mirrors flight_engine.py's _log_raw: writes to both file and terminal."""
    def __init__(self, path):
        self.fh = open(path, "w")
        self._closed = False

    def log(self, line):
        full = f"[{ts()}] {line}"
        print(full, flush=True)
        # A background stderr-reader thread can still be alive (draining a
        # just-terminated ffmpeg's final output) after this mode's log file
        # has been closed and the next mode has started -- drop those lines
        # instead of crashing the thread.
        if self._closed:
            return
        try:
            self.fh.write(full + "\n")
            self.fh.flush()
        except ValueError:
            pass

    def close(self):
        self._closed = True
        self.fh.close()


def cpu_stress_worker(stop_event):
    """
    Pure CPU busy-loop, no GPU/torch needed. Tests whether generic CPU
    scheduling contention (independent of any GPU/CUDA involvement) alone
    is enough to starve the camera-reading thread.
    """
    import numpy as np
    while not stop_event.is_set():
        a = np.random.rand(400, 400)
        b = np.random.rand(400, 400)
        _ = a @ b


def vision_stress_worker(stop_event, log):
    """
    Continuously runs real SuperPoint+LightGlue inference on throwaway
    images, matching the actual per-frame cadence/cost flight_engine.py
    pays during navigation (~350-500ms per match). This is the most
    faithful reproduction of real mission-time load -- reproduces it if
    the camera drops observed in real flights are caused by vision
    processing contending with the camera-reading thread for CPU/GPU.
    Needs torch/lightglue/cv2 -- run this in the same conda env
    flight_engine.py uses.
    """
    import cv2
    import numpy as np
    import torch
    from lightglue import LightGlue, SuperPoint

    device = "cuda" if torch.cuda.is_available() else "cpu"
    log.log(f"[STRESS] loading SuperPoint+LightGlue on {device}...")
    extractor = SuperPoint(max_num_keypoints=1024).eval().to(device)
    matcher = LightGlue(features="superpoint").eval().to(device)
    log.log("[STRESS] models loaded, starting continuous inference loop")

    def img_to_tensor(img_bgr):
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (512, 512))
        return torch.tensor(resized / 255.0, dtype=torch.float32)[None, None].to(device)

    count = 0
    t_report = time.time()
    while not stop_event.is_set():
        img0 = np.random.randint(0, 255, (CROP_H, CROP_W, 3), dtype=np.uint8)
        img1 = np.random.randint(0, 255, (CROP_H, CROP_W, 3), dtype=np.uint8)
        with torch.no_grad():
            f0 = extractor.extract(img_to_tensor(img0))
            f1 = extractor.extract(img_to_tensor(img1))
            matcher({"image0": f0, "image1": f1})
        count += 1
        if time.time() - t_report > 5.0:
            log.log(f"[STRESS] {count} vision inference iterations so far")
            t_report = time.time()
    log.log(f"[STRESS] stopped after {count} total iterations")


def start_stress(stress_mode, stress_workers, log):
    """Returns (stop_event, threads) -- caller stops via stop_event.set()."""
    stop_event = threading.Event()
    threads = []
    if stress_mode == "cpu":
        log.log(f"[STRESS] starting {stress_workers} CPU busy-loop worker(s)")
        for _ in range(stress_workers):
            t = threading.Thread(target=cpu_stress_worker, args=(stop_event,),
                                 daemon=True)
            t.start()
            threads.append(t)
    elif stress_mode == "vision":
        t = threading.Thread(target=vision_stress_worker,
                             args=(stop_event, log), daemon=True)
        t.start()
        threads.append(t)
    return stop_event, threads


def probe_resolution(rtsp_url, log):
    # NOTE: ffprobe's csv output order follows its own internal field
    # ordering, not the order fields are listed in -show_entries -- so we
    # query width/height in their own call (matches production exactly,
    # output is just "1280,720") and codec_name separately, purely for the
    # log line. Mixing them in one call made "hevc,1280,720" and broke
    # position-based parsing.
    out = subprocess.check_output([
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "csv=p=0", rtsp_url
    ], timeout=10).decode().strip()
    w, h = (int(x) for x in out.split(","))

    try:
        codec = subprocess.check_output([
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=codec_name",
            "-of", "csv=p=0", rtsp_url
        ], timeout=10).decode().strip()
    except Exception:
        codec = "?"
    log.log(f"[PROBE] {w}x{h} codec={codec}")
    return w, h


def build_ffmpeg_cmd(mode, rtsp_url, fps):
    common_in = ["-nostdin", "-rtsp_transport", "tcp",
                 "-fflags", "nobuffer", "-flags", "low_delay",
                 "-stimeout", "5000000"]
    common_out = ["-vf", f"fps={fps}", "-pix_fmt", "bgr24",
                  "-vcodec", "rawvideo", "-an", "-f", "rawvideo", "pipe:1"]
    if mode == "software":
        # Exactly what flight_engine.py's camera_thread_fn runs today.
        return ["ffmpeg"] + common_in + ["-i", rtsp_url] + common_out
    elif mode == "hardware":
        # Force the Jetson's hardware HEVC decoder (NVDEC) instead of the
        # CPU/software decoder. If this decoder name doesn't match your
        # ffmpeg build, this mode will fail fast and log the real error --
        # try `ffmpeg -decoders | grep nvv4l2` on the Jetson to find the
        # right name if so.
        return ["ffmpeg"] + common_in + ["-c:v", "hevc_nvv4l2dec",
                "-i", rtsp_url] + common_out
    raise ValueError(mode)


def run_one_mode(mode, rtsp_url, duration_s, out_dir, stress_mode="none",
                 stress_workers=2):
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    txt_path = os.path.join(out_dir, f"camera_test_{mode}_{stamp}.txt")
    csv_path = os.path.join(out_dir, f"camera_test_{mode}_{stamp}.csv")
    log = Logger(txt_path)
    csv_fh = open(csv_path, "w")
    csv_fh.write("frame_number,timestamp,gap_s\n")

    log.log(f"[TEST] mode={mode} rtsp_url={rtsp_url} duration={duration_s}s "
            f"stress={stress_mode}")
    log.log(f"[TEST] txt_log={txt_path}")
    log.log(f"[TEST] csv_log={csv_path}")

    stress_stop = None
    if stress_mode != "none":
        stress_stop, _stress_threads = start_stress(stress_mode, stress_workers, log)

    stop = threading.Event()
    stats = {
        "frames": 0, "reconnects": 0, "errors": 0,
        "gaps": [],          # list of (timestamp, gap_s) for gaps > GAP_WARN_S
        "attempt_started_at": [],
        "start_time": time.time(),
    }

    def _log_stderr(proc):
        for line in proc.stderr:
            line = line.decode(errors="ignore").strip()
            if line and "frame=" not in line:
                log.log(f"[FFmpeg] {line}")

    test_deadline = time.time() + duration_s
    attempt = 0
    last_frame_t = None

    while not stop.is_set() and time.time() < test_deadline and attempt < MAX_RECONNECTS:
        attempt += 1
        if attempt > 1:
            stats["reconnects"] += 1
            log.log(f"[CAM] Reconnect attempt {attempt} in {RECONNECT_DELAY}s...")
            time.sleep(RECONNECT_DELAY)
        stats["attempt_started_at"].append(time.time())

        proc = None
        try:
            w, h = probe_resolution(rtsp_url, log)
            frame_size = w * h * 3
            cmd = build_ffmpeg_cmd(mode, rtsp_url, CAMERA_FPS)
            log.log(f"[CAM] {w}x{h} attempt={attempt} cmd={' '.join(cmd)}")
            proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, bufsize=10**8)
            threading.Thread(target=_log_stderr, args=(proc,), daemon=True).start()

            time.sleep(3)
            if proc.poll() is not None:
                log.log(f"[CAM] FFmpeg exited early (code {proc.returncode}) "
                        f"-- mode={mode} likely unsupported on this build")
                if mode == "hardware":
                    log.log("[CAM] Try `ffmpeg -decoders | grep nvv4l2` on "
                            "the Jetson to find the correct hw decoder name.")
                break
            log.log("[CAM] Stream started.")

            consecutive_errors = 0
            while not stop.is_set() and time.time() < test_deadline:
                raw = proc.stdout.read(frame_size)
                now = time.time()
                if not raw or len(raw) != frame_size:
                    consecutive_errors += 1
                    stats["errors"] += 1
                    if consecutive_errors >= 10:
                        log.log("[CAM] Stream lost -- will reconnect.")
                        break
                    continue
                consecutive_errors = 0
                stats["frames"] += 1
                gap = (now - last_frame_t) if last_frame_t else 0.0
                if gap > GAP_WARN_S:
                    stats["gaps"].append((now, gap))
                    log.log(f"[CAM] gap {gap:.2f}s before frame "
                            f"#{stats['frames']}")
                last_frame_t = now
                csv_fh.write(f"{stats['frames']},{now:.3f},{gap:.3f}\n")
                csv_fh.flush()
        except Exception as e:
            log.log(f"[CAM] Error: {e}")
        finally:
            if proc is not None:
                try: proc.terminate()
                except Exception: pass

    if stress_stop is not None:
        stress_stop.set()

    elapsed = time.time() - stats["start_time"]
    csv_fh.close()

    log.log("=" * 60)
    log.log(f"[SUMMARY] mode={mode} stress={stress_mode}")
    log.log(f"  duration        : {elapsed:.1f}s")
    log.log(f"  frames received : {stats['frames']}")
    log.log(f"  effective fps   : {stats['frames']/elapsed:.2f}"
            if elapsed > 0 else "  effective fps   : n/a")
    log.log(f"  reconnects      : {stats['reconnects']}")
    log.log(f"  short/bad reads : {stats['errors']}")
    log.log(f"  gaps > {GAP_WARN_S}s   : {len(stats['gaps'])}")
    if stats["gaps"]:
        longest = max(g for _, g in stats["gaps"])
        log.log(f"  longest gap     : {longest:.2f}s")
    log.log("=" * 60)
    log.close()
    return stats, elapsed, txt_path, csv_path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["software", "hardware", "both"],
                    default="software",
                    help="which ffmpeg decode path to test (default: software, "
                         "matching current production behaviour)")
    ap.add_argument("--duration", type=float, default=120.0,
                    help="seconds to run per mode (default: 120)")
    ap.add_argument("--rtsp-url", default=RTSP_URL,
                    help=f"RTSP URL (default: {RTSP_URL})")
    ap.add_argument("--out-dir", default="camera_test_logs",
                    help="output directory (default: ./camera_test_logs)")
    ap.add_argument("--stress", choices=["none", "cpu", "vision"],
                    default="none",
                    help="synthetic load to run concurrently with the camera "
                         "test: 'cpu' = pure busy-loop (no torch needed), "
                         "'vision' = real SuperPoint+LightGlue inference loop "
                         "(needs torch/lightglue/cv2 -- run in the lightglue "
                         "conda env). Default: none (camera alone).")
    ap.add_argument("--stress-workers", type=int, default=2,
                    help="number of CPU busy-loop threads for --stress cpu "
                         "(default: 2; ignored for --stress vision, which "
                         "always uses 1 continuous inference loop)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    modes = ["software", "hardware"] if args.mode == "both" else [args.mode]

    results = {}
    for mode in modes:
        print(f"\n{'#'*60}\n# Testing mode={mode} for {args.duration:.0f}s "
              f"stress={args.stress}\n{'#'*60}\n")
        stats, elapsed, txt_path, csv_path = run_one_mode(
            mode, args.rtsp_url, args.duration, args.out_dir,
            stress_mode=args.stress, stress_workers=args.stress_workers)
        results[mode] = (stats, elapsed)

    if len(results) > 1:
        print(f"\n{'='*60}\nCOMPARISON (stress={args.stress})\n{'='*60}")
        for mode, (stats, elapsed) in results.items():
            fps = stats["frames"] / elapsed if elapsed > 0 else 0
            print(f"  {mode:10s}: frames={stats['frames']:5d}  "
                  f"fps={fps:5.2f}  reconnects={stats['reconnects']}  "
                  f"gaps>{GAP_WARN_S}s={len(stats['gaps'])}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[TEST] Interrupted by user.")
        sys.exit(0)
