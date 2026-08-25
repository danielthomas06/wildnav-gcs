"""
zed_vio_live.py — Run the ZED 2i's built-in visual-inertial odometry
(stereo + IMU fused positional tracking) and watch it live.

Opens the camera, enables positional tracking (stereo images fused with the
camera's onboard IMU by the ZED SDK), and shows two windows:
  - "camera" : the left camera feed with the current pose (position in
    meters, roll/pitch/yaw in degrees, tracking state, fps) overlaid.
  - "trajectory (top-down)" : a live top-down (ground-plane) trace of where
    the camera thinks it has moved since start, in meters.

This only exercises the camera's own VIO — it is independent of MAVLink /
flight_engine.py. It's meant to let you confirm tracking quality (drift,
recovery after occlusion, behavior under vibration) before wiring the fused
pose into the flight stack as a VISION_POSITION_ESTIMATE feed.

Requires: ZED SDK + pyzed installed (run `python3 get_python_api.py` from
your ZED SDK install dir, typically /usr/local/zed/), opencv-python, numpy.
Must run on a machine with the SDK and a ZED 2i attached (e.g. the Jetson) —
it will not run on a dev machine without the SDK installed.

Usage:
    python3 zed_vio_live.py
    python3 zed_vio_live.py --resolution HD1080 --fps 30 --scale 60
    python3 zed_vio_live.py --log run1.csv

Controls (with a display window focused):
    q / ESC  - quit
    r        - reset tracking origin back to the current pose
"""
import argparse
import csv
import time

import cv2
import numpy as np
import pyzed.sl as sl

RESOLUTIONS = {
    "HD2K": sl.RESOLUTION.HD2K,
    "HD1080": sl.RESOLUTION.HD1080,
    "HD720": sl.RESOLUTION.HD720,
    "VGA": sl.RESOLUTION.VGA,
}

DEPTH_MODES = {
    "PERFORMANCE": sl.DEPTH_MODE.PERFORMANCE,
    "QUALITY": sl.DEPTH_MODE.QUALITY,
    "ULTRA": sl.DEPTH_MODE.ULTRA,
    "NEURAL": sl.DEPTH_MODE.NEURAL,
}


def open_camera(resolution, fps, depth_mode):
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = RESOLUTIONS[resolution]
    init_params.camera_fps = fps
    # Positional tracking needs depth computed internally even though we
    # never retrieve a depth map ourselves; NONE disables that pipeline.
    init_params.depth_mode = DEPTH_MODES[depth_mode]
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"zed.open failed: {status}")

    tracking_params = sl.PositionalTrackingParameters()
    if hasattr(tracking_params, "enable_imu_fusion"):
        tracking_params.enable_imu_fusion = True  # fuse the onboard IMU (the 2i has one; the original ZED does not)
    status = zed.enable_positional_tracking(tracking_params)
    if status != sl.ERROR_CODE.SUCCESS:
        zed.close()
        raise RuntimeError(f"enable_positional_tracking failed: {status}")

    return zed


class TrajectoryCanvas:
    """Live top-down (X vs. Z ground plane) trace, in RIGHT_HANDED_Y_UP world coords."""

    def __init__(self, size=700, scale=40):
        self.size = size
        self.scale = scale  # pixels per meter
        self.center = size // 2
        self.points = []  # list of (screen_x, screen_y)
        self.canvas = np.full((size, size, 3), 255, np.uint8)
        self._draw_grid()

    def _draw_grid(self):
        step = self.scale  # one grid line per meter
        for p in range(0, self.size, step):
            cv2.line(self.canvas, (p, 0), (p, self.size), (230, 230, 230), 1)
            cv2.line(self.canvas, (0, p), (self.size, p), (230, 230, 230), 1)
        cv2.drawMarker(self.canvas, (self.center, self.center), (0, 0, 0),
                        cv2.MARKER_CROSS, 12, 1)

    def add(self, x, z):
        sx = int(self.center + x * self.scale)
        sy = int(self.center + z * self.scale)
        if self.points:
            cv2.line(self.canvas, self.points[-1], (sx, sy), (200, 0, 0), 2)
        self.points.append((sx, sy))

    def reset(self):
        self.points = []
        self.canvas[:] = 255
        self._draw_grid()

    def render(self):
        frame = self.canvas.copy()
        if self.points:
            cv2.circle(frame, self.points[-1], 5, (0, 0, 255), -1)
        return frame


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--resolution", choices=RESOLUTIONS, default="HD720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--depth-mode", choices=DEPTH_MODES, default="PERFORMANCE")
    ap.add_argument("--scale", type=int, default=40, help="pixels per meter in the trajectory view")
    ap.add_argument("--canvas-size", type=int, default=700)
    ap.add_argument("--log", default=None, help="optional CSV path to record pose every frame")
    args = ap.parse_args()

    zed = open_camera(args.resolution, args.fps, args.depth_mode)
    runtime_params = sl.RuntimeParameters()
    image = sl.Mat()
    camera_pose = sl.Pose()
    translation = sl.Translation()
    trajectory = TrajectoryCanvas(args.canvas_size, args.scale)

    log_file = log_writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["timestamp", "state", "x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg"])

    last_fps_t = time.time()
    frame_count = 0
    fps_display = 0.0

    try:
        while True:
            if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)

            state = zed.get_position(camera_pose, sl.REFERENCE_FRAME.WORLD)
            tx, ty, tz = camera_pose.get_translation(translation).get()
            roll, pitch, yaw = camera_pose.get_euler_angles(False)  # degrees

            if state == sl.POSITIONAL_TRACKING_STATE.OK:
                trajectory.add(tx, tz)

            frame_count += 1
            now = time.time()
            if now - last_fps_t >= 1.0:
                fps_display = frame_count / (now - last_fps_t)
                frame_count = 0
                last_fps_t = now

            lines = [
                f"state: {state}",
                f"pos (m):  x={tx:+.2f}  y={ty:+.2f}  z={tz:+.2f}",
                f"rot (deg): roll={roll:+.1f}  pitch={pitch:+.1f}  yaw={yaw:+.1f}",
                f"fps: {fps_display:.1f}",
            ]
            for i, text in enumerate(lines):
                cv2.putText(frame, text, (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2, cv2.LINE_AA)

            if log_writer:
                log_writer.writerow([now, str(state), tx, ty, tz, roll, pitch, yaw])

            cv2.imshow("camera", frame)
            cv2.imshow("trajectory (top-down)", trajectory.render())

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                zed.reset_positional_tracking(sl.Transform())
                trajectory.reset()
    finally:
        if log_file:
            log_file.close()
        zed.disable_positional_tracking()
        zed.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
