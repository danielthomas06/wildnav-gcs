"""
nav_modes.py — GPS-usage policy matrix for the four testing modes.
=================================================================

Two INDEPENDENT axes control GPS usage:

  (a) initial position SEED  — how waypoint_1 (the start location, captured
      at altitude) is obtained.
  (b) course-correction BACKSTOP — whether the GPS_RESET / sustained-bias
      snap / GPS-arrival backstop logic from navigate_north.py is active.

              seed_from_gps   gps_correction
  safety           True            True         <- current navigate_north.py behaviour
  cold_start       False           True         <- start from first VISUAL fix @ alt, GPS still backstops
  safe_start       True            False        <- GPS seed once, then pure vision
  full_gps_denied  False           False        <- pure vision, start-and-correction

When seed_from_gps is False, the drone climbs VERTICALLY to takeoff altitude
(directly above its physical start), then the first valid visual localisation
(above min_valid_alt_m) becomes waypoint_1. An optional approx-start pin from
the GUI narrows that first-fix search; if absent, a full-map tile search runs.
"""

from dataclasses import dataclass


VALID_MODES = ("safety", "cold_start", "safe_start", "full_gps_denied")


@dataclass(frozen=True)
class ModePolicy:
    name: str
    seed_from_gps: bool       # (a) use GPS to seed initial position?
    gps_correction: bool      # (b) allow GPS course-correction backstop?
    description: str

    @property
    def needs_visual_first_fix(self) -> bool:
        """True when waypoint_1 must come from vision (vertical climb + localise)."""
        return not self.seed_from_gps


_POLICIES = {
    "safety": ModePolicy(
        name="safety",
        seed_from_gps=True,
        gps_correction=True,
        description="GPS for both start seed and course correction. "
                    "Matches the original navigate_north.py. Safest for testing.",
    ),
    "cold_start": ModePolicy(
        name="cold_start",
        seed_from_gps=False,
        gps_correction=True,
        description="Start location comes from the first visual fix at altitude "
                    "(no GPS seed). GPS still allowed as course-correction backstop.",
    ),
    "safe_start": ModePolicy(
        name="safe_start",
        seed_from_gps=True,
        gps_correction=False,
        description="GPS gives the start location once, then GPS is dropped "
                    "entirely — pure vision for the rest of the flight.",
    ),
    "full_gps_denied": ModePolicy(
        name="full_gps_denied",
        seed_from_gps=False,
        gps_correction=False,
        description="No GPS at any point. Start from first visual fix at altitude, "
                    "vision-only course correction.",
    ),
}


def get_policy(mode: str) -> ModePolicy:
    if mode not in _POLICIES:
        raise ValueError(f"unknown mode {mode!r}; valid: {VALID_MODES}")
    return _POLICIES[mode]


def all_policies():
    """For the GUI to render mode cards with descriptions."""
    return [
        {
            "name": p.name,
            "seed_from_gps": p.seed_from_gps,
            "gps_correction": p.gps_correction,
            "needs_visual_first_fix": p.needs_visual_first_fix,
            "description": p.description,
        }
        for p in _POLICIES.values()
    ]
