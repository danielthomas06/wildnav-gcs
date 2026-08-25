"""
mission_prep.py — Waypoint convention: first point = takeoff/approx-start,
rest = destinations.
=============================================================================
The GUI lets an operator click points on the map into a single ordered list
("waypoints"). This module defines the convention used to turn that raw list
into what FlightEngine/MissionConfig actually need:

    waypoints[0]   -> the drone's takeoff / physical start location. It is
                      NEVER itself flown to as a destination — it becomes
                      cfg.approx_start, the search hint that narrows the
                      first-visual-fix acquisition (cold_start /
                      full_gps_denied modes only; ignored otherwise).
    waypoints[1:]  -> the actual destinations, flown in order exactly as
                      before (leg 1 target, leg 2 target, ...).

This means an operator no longer needs to separately drop a "start" pin
before launching a cold_start / full_gps_denied mission: dropping two
waypoints (takeoff spot, then destination) is enough. An explicit
"approx_start" already present in the payload (e.g. from the GUI's separate
start-pin feature) takes precedence and is left untouched.
"""


def _lat_lon(wp):
    """Accepts either [lat, lon] or {"lat":.., "lon":.., ...}."""
    if isinstance(wp, dict):
        return float(wp["lat"]), float(wp["lon"])
    return float(wp[0]), float(wp[1])


def prepare_mission_payload(payload: dict) -> dict:
    """
    Mutates and returns payload: splits payload["waypoints"] into
    approx_start (first entry) + destinations (the rest), per the
    convention above.

    Raises ValueError if fewer than 2 waypoints are given — there must be
    both a start location and at least one destination.
    """
    waypoints = payload.get("waypoints") or []
    if len(waypoints) < 2:
        raise ValueError(
            "need at least 2 waypoints: the first marks the takeoff/"
            f"approx-start location, the rest are destinations to fly to "
            f"(got {len(waypoints)})")

    if not payload.get("approx_start"):
        lat, lon = _lat_lon(waypoints[0])
        payload["approx_start"] = [lat, lon]

    payload["waypoints"] = waypoints[1:]
    return payload
