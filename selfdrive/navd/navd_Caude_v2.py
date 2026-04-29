#!/usr/bin/env python3
"""
navd.py — FrogPilot navigation daemon.

Owns the active route (Mapbox Directions API), publishes per-tick
``navInstruction`` messages, and translates the active maneuver into
FrogPilot lane-change *requests*.

Control authority
-----------------
The only control authority navd has over the car is the
``NavigationTestTurnCommand`` param, which the FrogPilot lane-change
subsystem can honor as a request for a LEFT or RIGHT lane change.
That single primitive is used in two distinct modes:

* **Lane positioning** — early, anticipatory left/right command to put
  the car into the correct edge lane well before a highway exit / fork.

* **Junction nudge** — a single, late left/right command issued near a
  surface-street turn or U-turn so the lane-change planner biases the
  car onto the correct branch at the junction.

navd never tries to steer through a turn — that is FrogPilot's vision
stack. We only place the car into the correct lane and (optionally)
nudge it through the junction.

The whole lane-command flow is gated by ``NavigationTestControl``. When
disabled, only ``navInstruction`` is published — no lane-change requests
are issued.
"""

import csv
import hashlib
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import requests

import cereal.messaging as messaging
from cereal import log
from openpilot.common.api import Api
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.navd.helpers import (Coordinate, coordinate_from_param,
                                              distance_along_geometry, maxspeed_to_ms,
                                              minimum_distance,
                                              parse_banner_instructions)
from openpilot.common.swaglog import cloudlog

from openpilot.frogpilot.common.frogpilot_variables import get_frogpilot_toggles


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# --- Route reroute / recompute ---------------------------------------------
REROUTE_DISTANCE = 25                 # m off-route → start counting toward reroute
REROUTE_COUNTER_MIN = 3               # consecutive ticks off-route to trigger reroute
MANEUVER_TRANSITION_THRESHOLD = 10    # m past maneuver to advance to next step

# --- Lane policy: command timing -------------------------------------------
# "Approaching the maneuver" window. Used for the basic display/voice
# 'maneuver imminent' state and as the surface-turn upcoming threshold.
LANE_COMMAND_DISTANCE_MIN = 35        # m
LANE_COMMAND_DISTANCE_SECONDS = 8     # s of speed-distance

# Lane-positioning prep window for surface (non-highway) exits/forks.
LANE_EXIT_PREP_DISTANCE_MIN = 250     # m
LANE_EXIT_PREP_SECONDS = 30           # s of speed-distance

# Highway-specific prep window. Active above HIGHWAY_PREP_SPEED.
HIGHWAY_PREP_SPEED = 22.0             # m/s (~80 km/h)
HIGHWAY_PREP_SECONDS = 180            # s of speed-distance
HIGHWAY_PREP_DISTANCE_MIN = 1500      # m
HIGHWAY_PREP_DISTANCE_MAX = 5000      # m

# --- Lane policy: junction nudge -------------------------------------------
# When True, navd will issue a single late lane-change request at
# surface-street turns / U-turns to bias the lane-change planner onto the
# correct branch at the junction. Disable to revert to guidance-only.
LANE_JUNCTION_NUDGE_ENABLED = True
LANE_JUNCTION_NUDGE_DISTANCE_MIN = 25.0   # m — absolute minimum window
LANE_JUNCTION_NUDGE_SECONDS = 4.0         # s of speed-distance

# --- Lane policy: safety / hysteresis --------------------------------------
# Late-lane-change lockout: once this close to the maneuver, do not request
# new lane changes — the lane-change planner needs runway to complete safely.
LANE_LATE_LOCKOUT_SECONDS = 3.0
LANE_LATE_LOCKOUT_DISTANCE_MIN = 80.0     # m

# Cooldown between successive lane-change commands.
LANE_COMMAND_COOLDOWN_SECONDS = 10.0

# Two consecutive opposite turns this close together → don't migrate to the
# wrong side for the first one, you'll just have to migrate back.
LANE_CONSECUTIVE_CONFLICT_DISTANCE = 400  # m

# Probability threshold on modelV2 outer lane-line to call an adjacent lane
# "available". Best-effort; lane-change module still vetoes if wrong.
LANE_ADJACENT_PROBABILITY = 0.35

# --- Post-exit recenter ----------------------------------------------------
# After a highway exit, gently move back toward the interior so we're not
# camped in the rightmost lane of the new road. Held for this duration unless
# a same-side maneuver appears within CONFLICT_DISTANCE.
POST_EXIT_RECENTER_SECONDS = 20.0
POST_EXIT_RECENTER_CONFLICT_DISTANCE = 500.0  # m

# --- Route confidence ------------------------------------------------------
# If we are this far off the planned route, the strategy stops trying to act.
LANE_MAX_CROSS_TRACK_ERROR = 35.0     # m
LANE_MAX_BEARING_ERROR = 55.0         # deg, only checked above 5 m/s

# --- Reroute behavior in test/lane mode ------------------------------------
LANE_REROUTE_COUNTER_MIN = 2
LANE_REROUTE_COUNTDOWN_MIN = 5

# --- Destination tracking (test mode) --------------------------------------
NAV_TEST_DESTINATIONS = {
    "home":   ("Navigation test - Home",   Coordinate(24.675764, 46.581478)),
    "work":   ("Navigation test - Work",   Coordinate(24.714778, 46.683775)),
    "school": ("Navigation test - School", Coordinate(24.781423, 46.622246)),
}
NAV_TEST_SHARED_DEST_URL = "https://frihtcjnhcayqvcphczr.supabase.co/rest/v1/shared_destination?id=eq.1&select=lat,lng"
NAV_TEST_SHARED_DEST_API_KEY = "sb_publishable_1Lh9fwsQOJppOm82Rk7uyA_nm2qWGdh"
NAV_TEST_SHARED_DEST_RETRY_SECONDS = 15.0

NAV_TEST_DEST_APPROACH_DISTANCE = 50  # m, "we got here" closeness
NAV_TEST_DEST_MISSED_DISTANCE = 80    # m past closest before counting as missed
NAV_TEST_DEST_MISSED_DRIFT = 30       # m drift past closest before counting
NAV_TEST_DEST_MISSED_COUNTER_MIN = 2

# --- Debug logging ---------------------------------------------------------
DEBUG_LOG_DIR = "/data/media/0/navigation_test_logs"
DEBUG_LOG_INTERVAL = 0.5              # s between rows
DEBUG_LOG_VERSION = 3
DEBUG_LOG_FIELDS = [
    "log_version", "time", "gps_ok", "localizer_valid", "lat", "lon", "bearing",
    "v_ego", "destination", "step_idx", "step_count",
    "maneuver_type", "maneuver_modifier", "maneuver_text", "maneuver_class",
    "road_context", "maneuver_lat", "maneuver_lon",
    "distance_to_maneuver_along_route", "distance_to_maneuver_straight",
    "command_threshold", "strategy_phase", "strategy_threshold",
    "strategy_constraint", "command_block_reason",
    "target_lane_zone", "lane_belief", "lane_left_available", "lane_right_available",
    "route_confident", "route_confidence", "route_bearing_error",
    "next_maneuver_direction", "next_maneuver_distance_after_current",
    "migration_active", "migration_age_seconds", "migration_start_distance",
    "post_exit_recenter_active", "junction_nudge_active",
    "action", "direction", "urgency", "cross_track_error",
]


# ---------------------------------------------------------------------------
# Data classes — strategy inputs and outputs
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LaneAvailability:
    """Best-effort estimate of whether the adjacent lanes exist."""
    left: Optional[bool] = None
    right: Optional[bool] = None
    confidence: float = 0.0

    @property
    def belief(self) -> str:
        if self.left is None or self.right is None:
            return "unknown"
        if self.left and self.right:
            return "interior"
        if self.left and not self.right:
            return "right_edge"
        if not self.left and self.right:
            return "left_edge"
        return "single_or_unknown"

    def is_available(self, direction: str) -> bool:
        """A direction is available unless we're confident it isn't."""
        if direction == "left":
            return self.left is not False
        if direction == "right":
            return self.right is not False
        return False


@dataclass
class ManeuverContext:
    """Everything the lane policy needs to decide what to do this tick."""
    instruction: Optional[dict]
    geometry: List
    distance: float                           # m along route to the maneuver
    direction: str                            # "left", "right", "none"
    display_direction: str                    # "slight_left", "uturn", ...
    effective_direction: str                  # like direction, but uturn → "left"
    maneuver_class: str                       # "highway_exit", "normal_turn", ...
    road_context: str                         # "highway" | "surface"
    lane: LaneAvailability
    next_direction: str                       # next directional maneuver after this
    next_distance_after_current: Optional[float]
    route_confident: bool
    route_confidence: float
    route_block_reason: str
    is_uturn: bool

    @property
    def is_directional(self) -> bool:
        return self.direction in ("left", "right") or self.is_uturn


@dataclass(frozen=True)
class LaneDecision:
    """Output of the lane policy. Drives the published command."""
    action: str = "none"
    direction: str = "none"
    display_direction: str = "none"
    strategy_phase: str = "none"
    strategy_threshold: float = 0.0
    strategy_constraint: str = "none"
    target_lane_zone: str = "none"
    block_reason: str = "none"
    urgency: float = 0.0
    is_junction_nudge: bool = False


# ---------------------------------------------------------------------------
# Module-level helpers — pure functions over instructions and geometry
# ---------------------------------------------------------------------------

def maneuver_direction(instruction) -> str:
    """Return 'left', 'right', or 'none' from a banner instruction."""
    if instruction is None:
        return "none"
    text = f"{instruction.get('maneuverType', '').lower()} {instruction.get('maneuverModifier', '').lower()}"
    if "left" in text:
        return "left"
    if "right" in text:
        return "right"
    return "none"


def maneuver_display_direction(instruction) -> str:
    """Return the more specific direction for UI (e.g. 'sharp_left', 'uturn')."""
    if instruction is None:
        return "none"
    modifier = instruction.get("maneuverModifier", "").lower().replace(" ", "_")
    maneuver_type = instruction.get("maneuverType", "").lower().replace(" ", "_")
    if "uturn" in (modifier, maneuver_type):
        return "uturn"
    if modifier in ("slight_left", "sharp_left", "left",
                    "slight_right", "sharp_right", "right"):
        return modifier
    return maneuver_direction(instruction)


def classify_maneuver(instruction) -> str:
    """Bucket a maneuver into a class the policy can dispatch on.
    Returns one of: 'highway_exit', 'highway_fork', 'highway_merge',
    'roundabout', 'uturn', 'normal_turn', 'arrive', 'continue', 'unknown', 'none'.
    """
    if instruction is None:
        return "none"
    direction = maneuver_direction(instruction)
    display = maneuver_display_direction(instruction)
    maneuver_type = instruction.get("maneuverType", "").lower()
    primary = instruction.get("maneuverPrimaryText", "").lower()
    text = f"{maneuver_type} {instruction.get('maneuverModifier', '').lower()} {primary}"

    if display == "uturn" or "uturn" in text or "u-turn" in text:
        return "uturn"
    if "arrive" in text or "destination" in text:
        return "arrive"
    if "roundabout" in text or "rotary" in text:
        return "roundabout"
    if "ramp" in maneuver_type or "exit" in primary or "take exit" in text:
        return "highway_exit"
    if "fork" in text or ("keep" in maneuver_type and direction in ("left", "right")):
        return "highway_fork"
    if "merge" in text:
        return "highway_merge"
    if direction in ("left", "right"):
        return "normal_turn"
    if "straight" in text or "continue" in text:
        return "continue"
    return "unknown"


def effective_direction_for(instruction, drive_side: str = "right") -> str:
    """Like maneuver_direction, but maps U-turns onto the lane direction
    appropriate for the country's drive side. Right-side driving (the default)
    means U-turns happen from the leftmost lane → 'left'.
    """
    direction = maneuver_direction(instruction)
    if direction != "none":
        return direction
    if maneuver_display_direction(instruction) == "uturn":
        return "left" if drive_side == "right" else "right"
    return "none"


def bearing_between(a, b) -> float:
    lat1 = math.radians(a.latitude)
    lat2 = math.radians(b.latitude)
    d_lon = math.radians(b.longitude - a.longitude)
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def angle_diff(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def target_zone_for(direction: str) -> str:
    if direction == "left":
        return "left_edge"
    if direction == "right":
        return "right_edge"
    return "none"


def is_lane_positioning_class(maneuver_class: str) -> bool:
    """Maneuver classes that get the *early* highway-style migration."""
    return maneuver_class in ("highway_exit", "highway_fork")


# ---------------------------------------------------------------------------
# RouteEngine
# ---------------------------------------------------------------------------

class RouteEngine:
    def __init__(self, sm, pm):
        self.sm = sm
        self.pm = pm

        self.params = Params()

        # --- Localization ---------------------------------------------------
        self.last_position = coordinate_from_param("LastGPSPosition", self.params)
        self.last_bearing = None
        self.gps_ok = False
        self.localizer_valid = False

        # --- Active route ---------------------------------------------------
        self.nav_destination = None
        self.step_idx = None
        self.route = None
        self.route_geometry = None

        self.recompute_backoff = 0
        self.recompute_countdown = 0
        self.reroute_counter = 0

        self.ui_pid = None

        # --- FrogPilot integration -----------------------------------------
        self.frogpilot_toggles = get_frogpilot_toggles()
        self.approaching_intersection = False
        self.approaching_turn = False
        self.nav_speed_limit = 0
        self.stop_coord = []
        self.stop_signal = []

        # --- Lane policy state ---------------------------------------------
        self.lane_reroute_counter = 0

        # Last published JSON command (for change-detection)
        self._last_published_command: Optional[str] = None

        # Highway-style exit migration tracking
        self.exit_migration_key = None
        self.exit_migration_direction = "none"
        self.exit_migration_started_at = 0.0
        self.exit_migration_start_distance = 0.0

        # Post-exit recenter
        self.post_exit_recenter_direction = "none"
        self.post_exit_recenter_exit_direction = "none"
        self.post_exit_recenter_started_at = 0.0
        self.post_exit_recenter_expires_at = 0.0
        self.post_exit_recenter_done = False

        # Lane-command cooldown
        self.last_lane_command_at = 0.0
        self.last_lane_command_direction = "none"
        self.last_lane_command_distance = 0.0

        # Junction-nudge tracking — which maneuvers we've already nudged for
        self.junction_nudge_done_keys: set = set()
        self.junction_nudge_just_issued = False

        # --- Test-destination handling -------------------------------------
        self.shared_dest = None
        self.shared_dest_retry_at = 0.0
        self.last_share_token = ""
        self.dest_missed_counter = 0
        self.closest_dest_distance = None

        # --- Debug logging --------------------------------------------------
        self.debug_last_log_time = 0.0
        self.debug_log_override_path = os.environ.get("NAVIGATION_TEST_DEBUG_LOG_PATH", "")
        self.debug_log_dir = os.environ.get("NAVIGATION_TEST_DEBUG_LOG_DIR", DEBUG_LOG_DIR)

        # --- Background route fetcher --------------------------------------
        self._route_thread = None
        self._route_lock = threading.Lock()
        self._pending_route_result = None
        self._pending_route_error = None
        # External-app side files (consumed by the FrogPilot UI / nav app)
        self.r2 = {}
        self.r3 = {}

        # --- Mapbox API ----------------------------------------------------
        self.api = Api(self.params.get("DongleId", encoding='utf8'))
        self.mapbox_host = "https://api.mapbox.com"
        self.mapbox_token = self._load_mapbox_token()

    # -----------------------------------------------------------------------
    # Setup helpers
    # -----------------------------------------------------------------------

    def _load_mapbox_token(self) -> Optional[str]:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        token_file_path = os.path.join(current_dir, "mapbox_token")
        try:
            with open(token_file_path, "r") as f:
                return f.read().strip()
        except FileNotFoundError:
            cloudlog.warning(f"Mapbox token file not found at {token_file_path}!")
            return None

    @staticmethod
    def _async_write_json(filepath, data):
        """Write JSON in a daemon thread so we never block the 1 Hz update loop."""
        def write_task():
            try:
                with open(filepath, 'w') as f:
                    json.dump(data, f, indent=4)
            except Exception as e:
                cloudlog.warning(f"Failed to async write {filepath}: {e}")
        threading.Thread(target=write_task, daemon=True).start()

    # -----------------------------------------------------------------------
    # Main tick
    # -----------------------------------------------------------------------

    def update(self):
        self.sm.update(0)

        # UI restart: re-send route so the map repopulates.
        if self.sm.updated["managerState"]:
            ui_pid = [p.pid for p in self.sm["managerState"].processes
                      if p.name == "ui" and p.running]
            if ui_pid:
                if self.ui_pid and self.ui_pid != ui_pid[0]:
                    cloudlog.warning("UI restarting, sending route")
                    threading.Timer(5.0, self.send_route).start()
                self.ui_pid = ui_pid[0]

        self.update_location()
        try:
            self.update_test_destination()
            self.recompute_route()
            self._check_and_apply_route_thread()
            self.send_instruction()
        except Exception as err:
            if self.is_lane_control_enabled():
                self.publish_command("routeError", error=err.__class__.__name__)
            cloudlog.exception("navd.failed_to_compute")

        if self.sm['frogpilotPlan'].togglesUpdated:
            self.frogpilot_toggles = get_frogpilot_toggles()

    def update_location(self):
        location = self.sm['liveLocationKalman']
        self.gps_ok = location.gpsOK
        self.localizer_valid = (
            location.status == log.LiveLocationKalman.Status.valid
            and location.positionGeodetic.valid
        )
        if self.localizer_valid:
            self.last_bearing = math.degrees(location.calibratedOrientationNED.value[2])
            self.last_position = Coordinate(location.positionGeodetic.value[0],
                                            location.positionGeodetic.value[1])

    def is_lane_control_enabled(self) -> bool:
        return self.params.get_bool("NavigationTestControl")

    # =======================================================================
    # Test destination handling (home/work/school/share)
    # =======================================================================

    def update_test_destination(self):
        if not self.is_lane_control_enabled():
            self.publish_command("none")
            return

        if self.last_position is None:
            self.publish_command("waitingGps")
            return

        destination_id = self.params.get("NavigationTestSelectedDestination", encoding="utf8") or "home"

        if destination_id == "share":
            share_token = self.params.get("NavigationTestShareSelectionToken", encoding="utf8")
            force_refresh = share_token != self.last_share_token
            current_dest = coordinate_from_param("NavDestination", self.params)
            shared = self.get_shared_destination(force_refresh or current_dest is None)
            if shared is None:
                return
            self.last_share_token = share_token
            destination_name, target = shared
        else:
            destination_name, target = NAV_TEST_DESTINATIONS.get(
                destination_id, NAV_TEST_DESTINATIONS["home"])
            current_dest = coordinate_from_param("NavDestination", self.params)

        if current_dest == target:
            return

        self.publish_command("routing")
        self.params.put("NavDestination", json.dumps({
            "latitude": target.latitude,
            "longitude": target.longitude,
            "place_name": destination_name,
        }))

    def get_shared_destination(self, force_refresh: bool = False):
        now = time.monotonic()
        if not force_refresh and self.shared_dest is not None:
            return self.shared_dest
        if not force_refresh and now < self.shared_dest_retry_at:
            return self.shared_dest

        try:
            response = requests.get(
                NAV_TEST_SHARED_DEST_URL,
                timeout=5,
                headers={
                    "apikey": NAV_TEST_SHARED_DEST_API_KEY,
                    "Authorization": f"Bearer {NAV_TEST_SHARED_DEST_API_KEY}",
                },
            )
            response.raise_for_status()
            payload = response.json()
        except requests.RequestException as err:
            cloudlog.warning(f"Shared destination fetch failed: {err}")
            self.shared_dest_retry_at = now + NAV_TEST_SHARED_DEST_RETRY_SECONDS
            self.publish_command("routeError", error="sharedFetchFailed")
            return None
        except ValueError as err:
            cloudlog.warning(f"Shared destination JSON parse failed: {err}")
            self.shared_dest_retry_at = now + NAV_TEST_SHARED_DEST_RETRY_SECONDS
            self.publish_command("routeError", error="sharedInvalidJson")
            return None

        record = payload[0] if isinstance(payload, list) and payload else payload
        if not isinstance(record, dict):
            cloudlog.warning(f"Shared destination has invalid payload: {payload}")
            self.shared_dest_retry_at = now + NAV_TEST_SHARED_DEST_RETRY_SECONDS
            self.publish_command("routeError", error="sharedInvalidPayload")
            return None

        try:
            latitude = float(record["lat"])
            longitude = float(record["lng"])
        except (KeyError, TypeError, ValueError) as err:
            cloudlog.warning(f"Shared destination missing coordinates: {err}")
            self.shared_dest_retry_at = now + NAV_TEST_SHARED_DEST_RETRY_SECONDS
            self.publish_command("routeError", error="sharedInvalidCoordinates")
            return None

        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            cloudlog.warning(f"Shared destination out of bounds: {(latitude, longitude)}")
            self.shared_dest_retry_at = now + NAV_TEST_SHARED_DEST_RETRY_SECONDS
            self.publish_command("routeError", error="sharedOutOfBounds")
            return None

        self.shared_dest = ("Navigation test - Share", Coordinate(latitude, longitude))
        self.shared_dest_retry_at = 0.0
        return self.shared_dest

    # =======================================================================
    # Lane-availability estimation (best-effort, from modelV2)
    # =======================================================================

    def estimate_lane_availability(self) -> LaneAvailability:
        """Read modelV2 lane-line probabilities to guess if adjacent lanes
        exist. The lane-change module still vetoes if we get this wrong.
        """
        try:
            model = self.sm['modelV2']
            probs = list(model.laneLineProbs)
        except Exception:
            return LaneAvailability()

        # openpilot models expose four lane-line probs: outer-left,
        # left-of-ego, right-of-ego, outer-right. The outer ones being
        # high suggests a lane exists beyond ego's lane on that side.
        if len(probs) < 4:
            return LaneAvailability()

        left_score = float(probs[0])
        right_score = float(probs[3])
        return LaneAvailability(
            left=left_score >= LANE_ADJACENT_PROBABILITY,
            right=right_score >= LANE_ADJACENT_PROBABILITY,
            confidence=max(left_score, right_score),
        )

    # =======================================================================
    # Geometry / cross-track helpers
    # =======================================================================

    def path_minimum_distance(self, path) -> Optional[float]:
        """Distance from current position to the closest point on the polyline.
        Vectorized equirectangular projection, segments < 1 m are ignored.
        """
        if self.last_position is None or len(path) < 2:
            return None

        coords = np.array([(c.latitude, c.longitude) for c in path])
        p_lat, p_lon = self.last_position.latitude, self.last_position.longitude

        R = 6371000.0
        deg_to_rad = np.pi / 180.0

        # Project relative to ego (origin = ego position)
        y = (coords[:, 0] - p_lat) * deg_to_rad * R
        x = (coords[:, 1] - p_lon) * np.cos(p_lat * deg_to_rad) * deg_to_rad * R

        A_x, A_y = x[:-1], y[:-1]
        B_x, B_y = x[1:], y[1:]
        AB_x, AB_y = B_x - A_x, B_y - A_y
        AP_x, AP_y = -A_x, -A_y

        AB_dot_AB = AB_x ** 2 + AB_y ** 2
        AP_dot_AB = AP_x * AB_x + AP_y * AB_y
        with np.errstate(invalid='ignore', divide='ignore'):
            t = np.clip(AP_dot_AB / AB_dot_AB, 0.0, 1.0)
        t = np.nan_to_num(t)

        C_x = A_x + t * AB_x
        C_y = A_y + t * AB_y
        distances = np.sqrt(C_x ** 2 + C_y ** 2)

        segment_lengths = np.sqrt(AB_dot_AB)
        valid = distances[segment_lengths >= 1.0]
        if len(valid) == 0:
            return None
        return float(np.min(valid))

    def cross_track_error(self) -> Optional[float]:
        if self.route_geometry is None or self.last_position is None:
            return None
        closest = None
        for geometry in self.route_geometry:
            d = self.path_minimum_distance(geometry)
            if d is None:
                continue
            closest = d if closest is None else min(closest, d)
        return closest

    def closest_segment_bearing(self, path) -> Optional[float]:
        if self.last_position is None or path is None or len(path) < 2:
            return None
        closest_distance = None
        closest_bearing = None
        for i in range(len(path) - 1):
            d = self.path_minimum_distance([path[i], path[i + 1]])
            if d is None:
                continue
            if closest_distance is None or d < closest_distance:
                closest_distance = d
                closest_bearing = bearing_between(path[i], path[i + 1])
        return closest_bearing

    # =======================================================================
    # Route confidence
    # =======================================================================

    def assess_route_confidence(self, geometry, cross_track) -> Tuple[bool, float, Optional[float], str]:
        """Returns (confident, score in [0,1], bearing_error_deg, block_reason)."""
        if not self.gps_ok:
            return False, 0.0, None, "gpsInvalid"
        if not self.localizer_valid:
            return False, 0.0, None, "localizerInvalid"

        score = 1.0
        if cross_track is not None:
            score = min(score, max(0.0, 1.0 - cross_track / max(LANE_MAX_CROSS_TRACK_ERROR, 1.0)))
            if cross_track > LANE_MAX_CROSS_TRACK_ERROR:
                return False, score, None, "crossTrackError"

        bearing_error = None
        route_bearing = self.closest_segment_bearing(geometry)
        if route_bearing is not None and self.last_bearing is not None:
            bearing_error = abs(angle_diff(self.last_bearing, route_bearing))
            score = min(score, max(0.0, 1.0 - bearing_error / 90.0))
            if self.sm['carState'].vEgo > 5.0 and bearing_error > LANE_MAX_BEARING_ERROR:
                return False, score, bearing_error, "bearingError"

        return True, score, bearing_error, "none"

    # =======================================================================
    # Distance windows
    # =======================================================================

    def _v_ego(self) -> float:
        return self.sm['carState'].vEgo

    def command_distance(self) -> float:
        return max(LANE_COMMAND_DISTANCE_MIN, self._v_ego() * LANE_COMMAND_DISTANCE_SECONDS)

    def exit_prep_distance(self) -> float:
        return max(LANE_EXIT_PREP_DISTANCE_MIN, self._v_ego() * LANE_EXIT_PREP_SECONDS)

    def highway_exit_prep_distance(self) -> float:
        v = self._v_ego()
        if v < HIGHWAY_PREP_SPEED:
            return self.exit_prep_distance()
        return min(HIGHWAY_PREP_DISTANCE_MAX,
                   max(HIGHWAY_PREP_DISTANCE_MIN, v * HIGHWAY_PREP_SECONDS))

    def late_lockout_distance(self) -> float:
        return max(LANE_LATE_LOCKOUT_DISTANCE_MIN, self._v_ego() * LANE_LATE_LOCKOUT_SECONDS)

    def junction_nudge_distance(self) -> float:
        return max(LANE_JUNCTION_NUDGE_DISTANCE_MIN, self._v_ego() * LANE_JUNCTION_NUDGE_SECONDS)

    def road_context_for(self, maneuver_class: str) -> str:
        if maneuver_class in ("highway_exit", "highway_fork", "highway_merge"):
            return "highway"
        if self._v_ego() >= HIGHWAY_PREP_SPEED:
            return "highway"
        if self.nav_speed_limit >= HIGHWAY_PREP_SPEED:
            return "highway"
        return "surface"

    # =======================================================================
    # Lane-command gating (cooldown)
    # =======================================================================

    def lane_command_allowed(self, direction: str) -> Tuple[bool, str]:
        if direction not in ("left", "right"):
            return False, "invalidDirection"
        if time.monotonic() - self.last_lane_command_at < LANE_COMMAND_COOLDOWN_SECONDS:
            return False, "laneCommandCooldown"
        return True, "none"

    def record_lane_command(self, direction: str, distance: float):
        self.last_lane_command_at = time.monotonic()
        self.last_lane_command_direction = direction
        self.last_lane_command_distance = distance

    # =======================================================================
    # Exit migration tracking (which highway-exit migration is in progress)
    # =======================================================================

    def maneuver_key(self, instruction, geometry):
        if instruction is None:
            return None
        coord = geometry[-1] if geometry else None
        m_lat = round(coord.latitude, 5) if coord is not None else None
        m_lon = round(coord.longitude, 5) if coord is not None else None
        return (
            instruction.get("maneuverType", ""),
            instruction.get("maneuverModifier", ""),
            instruction.get("maneuverPrimaryText", ""),
            m_lat,
            m_lon,
        )

    def reset_exit_migration(self):
        self.exit_migration_key = None
        self.exit_migration_direction = "none"
        self.exit_migration_started_at = 0.0
        self.exit_migration_start_distance = 0.0

    def update_exit_migration(self, instruction, geometry, direction, distance):
        key = self.maneuver_key(instruction, geometry)
        if key is None or direction == "none":
            self.reset_exit_migration()
            return
        if key != self.exit_migration_key or direction != self.exit_migration_direction:
            self.exit_migration_key = key
            self.exit_migration_direction = direction
            self.exit_migration_started_at = time.monotonic()
            self.exit_migration_start_distance = distance

    # =======================================================================
    # Post-exit recenter
    # =======================================================================

    def reset_post_exit_recenter(self):
        self.post_exit_recenter_direction = "none"
        self.post_exit_recenter_exit_direction = "none"
        self.post_exit_recenter_started_at = 0.0
        self.post_exit_recenter_expires_at = 0.0
        self.post_exit_recenter_done = False

    def post_exit_recenter_active(self) -> bool:
        return (
            self.post_exit_recenter_direction in ("left", "right")
            and not self.post_exit_recenter_done
            and time.monotonic() <= self.post_exit_recenter_expires_at
        )

    def start_post_exit_recenter(self, exit_direction: str):
        if exit_direction == "right":
            recenter = "left"
        elif exit_direction == "left":
            recenter = "right"
        else:
            self.reset_post_exit_recenter()
            return
        now = time.monotonic()
        self.post_exit_recenter_direction = recenter
        self.post_exit_recenter_exit_direction = exit_direction
        self.post_exit_recenter_started_at = now
        self.post_exit_recenter_expires_at = now + POST_EXIT_RECENTER_SECONDS
        self.post_exit_recenter_done = False

    def is_exit_maneuver(self, instruction) -> bool:
        if instruction is None:
            return False
        maneuver_type = instruction.get("maneuverType", "").lower()
        primary = instruction.get("maneuverPrimaryText", "").lower()
        return "ramp" in maneuver_type or "exit" in primary

    def decide_post_exit_recenter(self, ctx: ManeuverContext) -> Optional[LaneDecision]:
        """If a post-exit recenter is active, possibly emit a recenter command.
        Returns None if recenter doesn't apply this tick.
        """
        if not self.post_exit_recenter_active():
            if (self.post_exit_recenter_direction != "none"
                    and time.monotonic() > self.post_exit_recenter_expires_at):
                self.reset_post_exit_recenter()
            return None

        # Don't escape away from the exit side if a same-side maneuver is imminent.
        if (ctx.next_direction == self.post_exit_recenter_exit_direction
                and ctx.next_distance_after_current is not None
                and 0.0 < ctx.next_distance_after_current <= POST_EXIT_RECENTER_CONFLICT_DISTANCE):
            return None

        direction = self.post_exit_recenter_direction
        if not ctx.lane.is_available(direction):
            self.post_exit_recenter_done = True
            return None

        allowed, reason = self.lane_command_allowed(direction)
        if not allowed:
            return LaneDecision(
                action="upcoming",
                direction=direction,
                display_direction=direction,
                strategy_phase="postExitRecenterCooldown",
                strategy_constraint=reason,
                target_lane_zone="interior",
                block_reason=reason,
                urgency=0.25,
            )

        self.record_lane_command(direction, 0.0)
        self.post_exit_recenter_done = True
        return LaneDecision(
            action="laneChange",
            direction=direction,
            display_direction=direction,
            strategy_phase="postExitRecenter",
            target_lane_zone="interior",
            urgency=0.65,
        )

    # =======================================================================
    # Junction-nudge bookkeeping
    # =======================================================================

    def junction_nudge_done_for(self, key) -> bool:
        return key in self.junction_nudge_done_keys

    def record_junction_nudge(self, key):
        if key is not None:
            self.junction_nudge_done_keys.add(key)

    def reset_junction_nudge(self):
        self.junction_nudge_done_keys.clear()

    # =======================================================================
    # Maneuver lookahead — find the next directional maneuver after the current
    # =======================================================================

    def next_directional_maneuver(self, distance_to_current: float):
        if self.route is None or self.step_idx is None:
            return "none", None, None
        cumulative = distance_to_current
        for i in range(self.step_idx + 1, len(self.route)):
            cumulative += self.route[i]['distance']
            instruction = parse_banner_instructions(self.route[i]['bannerInstructions'], cumulative)
            if instruction is None:
                continue
            direction = maneuver_direction(instruction)
            display = maneuver_display_direction(instruction)
            if direction != "none" or display == "uturn":
                return direction, cumulative, instruction
        return "none", None, None

    # =======================================================================
    # Strategy — build context, decide action, dispatch policy
    # =======================================================================

    def build_maneuver_context(self, instruction, geometry, distance: float) -> ManeuverContext:
        direction = maneuver_direction(instruction)
        display = maneuver_display_direction(instruction)
        eff_dir = effective_direction_for(instruction)
        m_class = classify_maneuver(instruction)
        road_ctx = self.road_context_for(m_class)
        lane = self.estimate_lane_availability()

        next_dir, next_dist, _ = self.next_directional_maneuver(distance)
        next_dist_after = (
            max(next_dist - distance, 0.0) if next_dist is not None else None
        )

        cross_track = self.cross_track_error()
        confident, conf_score, _, block = self.assess_route_confidence(geometry, cross_track)

        return ManeuverContext(
            instruction=instruction,
            geometry=geometry,
            distance=distance,
            direction=direction,
            display_direction=display,
            effective_direction=eff_dir,
            maneuver_class=m_class,
            road_context=road_ctx,
            lane=lane,
            next_direction=next_dir,
            next_distance_after_current=next_dist_after,
            route_confident=confident,
            route_confidence=conf_score,
            route_block_reason=block,
            is_uturn=(display == "uturn"),
        )

    def decide_lane_action(self, ctx: ManeuverContext) -> LaneDecision:
        """Top-level strategy: dispatches by maneuver class."""

        # 1. Bail out if we don't trust the route
        if not ctx.route_confident:
            self.reset_exit_migration()
            return LaneDecision(
                action="routeMismatch",
                strategy_phase="routeMismatch",
                strategy_constraint="routeMismatch",
                block_reason=ctx.route_block_reason,
            )

        # 2. Post-exit recenter has priority over the next maneuver.
        recenter = self.decide_post_exit_recenter(ctx)
        if recenter is not None:
            return recenter

        # 3. No directional maneuver ahead — idle.
        if not ctx.is_directional:
            self.reset_exit_migration()
            return LaneDecision()

        # 4. Dispatch to the appropriate policy.
        if ctx.maneuver_class in ("highway_exit", "highway_fork"):
            return self._policy_highway_exit(ctx)
        if ctx.maneuver_class in ("normal_turn", "uturn"):
            return self._policy_junction_turn(ctx)

        # roundabout / highway_merge / arrive / continue / unknown:
        # We don't know which lane is correct, so fall back to guidance only.
        return self._policy_guidance_only(ctx)

    # ---- Policy: guidance-only (no command) -------------------------------

    def _policy_guidance_only(self, ctx: ManeuverContext) -> LaneDecision:
        self.reset_exit_migration()
        cmd_distance = self.command_distance()
        if ctx.distance <= cmd_distance:
            urgency = min(1.0, cmd_distance / max(ctx.distance, 1.0))
            return LaneDecision(
                action="guidanceOnly",
                direction=ctx.direction,
                display_direction=ctx.display_direction,
                strategy_phase="guidanceOnly",
                strategy_threshold=cmd_distance,
                target_lane_zone=target_zone_for(ctx.direction),
                urgency=urgency,
            )
        return LaneDecision(
            action="upcoming",
            direction=ctx.direction,
            display_direction=ctx.display_direction,
            strategy_phase="upcoming",
            strategy_threshold=cmd_distance,
            target_lane_zone=target_zone_for(ctx.direction),
        )

    # ---- Policy: highway exit / fork --------------------------------------

    def _policy_highway_exit(self, ctx: ManeuverContext) -> LaneDecision:
        """Early lane positioning: migrate to the exit-side edge well in
        advance, hold there until the maneuver, then let FrogPilot drive."""
        direction = ctx.direction or ctx.effective_direction
        target = target_zone_for(direction)

        standard_prep = self.exit_prep_distance()
        highway_prep = self.highway_exit_prep_distance()
        active_prep = highway_prep if ctx.road_context == "highway" else standard_prep
        late_lockout = self.late_lockout_distance()
        urgency = max(0.0, min(1.0, 1.0 - ctx.distance / max(active_prep, 1.0)))

        conflict_soon = (
            ctx.next_direction in ("left", "right")
            and ctx.next_direction != direction
            and ctx.next_distance_after_current is not None
            and 0.0 < ctx.next_distance_after_current <= LANE_CONSECUTIVE_CONFLICT_DISTANCE
        )

        base = dict(
            direction=direction,
            display_direction=ctx.display_direction,
            target_lane_zone=target,
            strategy_threshold=active_prep,
        )

        # Outside the prep window: just announce upcoming.
        if ctx.distance > active_prep:
            self.reset_exit_migration()
            return LaneDecision(action="upcoming", strategy_phase="upcoming",
                                urgency=urgency, **base)

        # Inside the prep window — track the migration target.
        self.update_exit_migration(ctx.instruction, ctx.geometry, direction, ctx.distance)

        # Conflicting opposite turn very soon after this one — hold off if we
        # haven't even entered the standard prep window yet.
        if conflict_soon and ctx.distance > standard_prep:
            self.reset_exit_migration()
            return LaneDecision(
                action="upcoming",
                strategy_phase="consecutiveConflictHold",
                strategy_constraint="conflictingNextManeuver",
                block_reason="conflictingNextManeuver",
                urgency=urgency,
                **base,
            )

        # Too late to safely change lanes — guidance only.
        if ctx.distance <= late_lockout:
            return LaneDecision(
                action="guidanceOnly",
                strategy_phase="maneuverLockout",
                block_reason="lateLaneChangeLockout",
                urgency=1.0,
                **{**base, "strategy_threshold": late_lockout},
            )

        # Already in the target edge lane — hold.
        if target != "none" and ctx.lane.belief == target:
            return LaneDecision(
                action="upcoming",
                strategy_phase="targetEdgeHold",
                block_reason="targetEdgeReached",
                urgency=urgency,
                **base,
            )

        # Adjacent lane (the one we'd change into) doesn't exist — hold.
        if not ctx.lane.is_available(direction):
            return LaneDecision(
                action="upcoming",
                strategy_phase="targetEdgeHold",
                block_reason="adjacentLaneUnavailable",
                urgency=urgency,
                **base,
            )

        # Cooldown after a recent lane-change command — hold.
        allowed, reason = self.lane_command_allowed(direction)
        if not allowed:
            constraint = "conflictingNextManeuver" if conflict_soon else "none"
            return LaneDecision(
                action="upcoming",
                strategy_phase="laneCommandCooldown",
                strategy_constraint=constraint,
                block_reason=reason,
                urgency=urgency,
                **base,
            )

        # All clear — issue lane change toward the exit side.
        phase = "exitMigration" if ctx.distance <= standard_prep else "highwayExitMigration"
        constraint = "conflictingNextManeuver" if conflict_soon else "none"
        self.record_lane_command(direction, ctx.distance)
        return LaneDecision(
            action="laneChange",
            strategy_phase=phase,
            strategy_constraint=constraint,
            urgency=urgency,
            **base,
        )

    # ---- Policy: surface-street junction turn / U-turn --------------------

    def _policy_junction_turn(self, ctx: ManeuverContext) -> LaneDecision:
        """Issue a single late lane-change command at the junction so the
        lane-change planner biases the car onto the correct branch.

        This is the only way for navd to influence a non-highway turn —
        FrogPilot's vision normally just follows lane lines, which can pick
        the wrong branch at a fork-shaped intersection.
        """
        if not LANE_JUNCTION_NUDGE_ENABLED:
            return self._policy_guidance_only(ctx)

        direction = ctx.effective_direction
        if direction == "none":
            return self._policy_guidance_only(ctx)

        # Junction nudges and exit migrations are mutually exclusive — clear
        # any leftover migration state from a previous step.
        self.reset_exit_migration()

        nudge_distance = self.junction_nudge_distance()
        cmd_distance = self.command_distance()
        target = target_zone_for(direction)
        urgency = max(0.0, min(1.0, 1.0 - ctx.distance / max(nudge_distance, 1.0)))

        base = dict(
            direction=direction,
            display_direction=ctx.display_direction,
            target_lane_zone=target,
            strategy_threshold=nudge_distance,
        )

        # Outside the nudge window — monitor only.
        if ctx.distance > nudge_distance:
            return LaneDecision(
                action="upcoming",
                strategy_phase="junctionMonitor",
                urgency=urgency,
                **{**base, "strategy_threshold": cmd_distance},
            )

        # Already nudged for this maneuver — done.
        key = self.maneuver_key(ctx.instruction, ctx.geometry)
        if key is not None and self.junction_nudge_done_for(key):
            return LaneDecision(
                action="upcoming",
                strategy_phase="junctionNudgeDone",
                block_reason="junctionNudgeAlreadyIssued",
                urgency=urgency,
                **base,
            )

        # Adjacent lane unavailable — fall back to guidance.
        if not ctx.lane.is_available(direction):
            return LaneDecision(
                action="guidanceOnly",
                strategy_phase="junctionGuidance",
                block_reason="adjacentLaneUnavailable",
                urgency=urgency,
                **base,
            )

        # Cooldown — hold.
        allowed, reason = self.lane_command_allowed(direction)
        if not allowed:
            return LaneDecision(
                action="upcoming",
                strategy_phase="laneCommandCooldown",
                block_reason=reason,
                urgency=urgency,
                **base,
            )

        # Issue exactly one nudge for this maneuver.
        self.record_lane_command(direction, ctx.distance)
        self.record_junction_nudge(key)
        return LaneDecision(
            action="laneChange",            # use 'laneChange' for UI compat
            strategy_phase="junctionNudge",
            urgency=urgency,
            is_junction_nudge=True,
            **base,
        )

    # =======================================================================
    # Command publishing
    # =======================================================================

    def publish_command(self, action: str, *, direction: str = "none",
                        distance: float = 0.0, eta_seconds: float = 0.0,
                        display_direction: Optional[str] = None,
                        error: str = "",
                        strategy_phase: str = "none",
                        strategy_constraint: str = "none",
                        target_lane_zone: str = "none",
                        lane_belief: str = "unknown",
                        maneuver_class: str = "unknown",
                        road_context: str = "unknown",
                        route_confidence: float = 0.0,
                        block_reason: str = "none",
                        urgency: float = 0.0):
        """Serialize current state to NavigationTestTurnCommand. Only writes
        the param when the payload actually changes.
        """
        migration_age = (
            time.monotonic() - self.exit_migration_started_at
            if self.exit_migration_key is not None else 0.0
        )
        payload = {
            "action": action,
            "direction": direction,
            "displayDirection": display_direction or direction,
            "distance": max(distance, 0.0),
            "etaSeconds": max(eta_seconds, 0.0),
            "error": error,
            "strategyPhase": strategy_phase,
            "strategyConstraint": strategy_constraint,
            "targetLaneZone": target_lane_zone,
            "laneBelief": lane_belief,
            "maneuverClass": maneuver_class,
            "roadContext": road_context,
            "routeConfidence": max(0.0, min(1.0, route_confidence)),
            "commandBlockReason": block_reason,
            "urgency": max(0.0, min(1.0, urgency)),
            "migrationActive": self.exit_migration_key is not None,
            "migrationAgeSeconds": max(migration_age, 0.0),
            "migrationStartDistance": max(self.exit_migration_start_distance, 0.0),
            "postExitRecenterActive": self.post_exit_recenter_active(),
        }
        serialized = json.dumps(payload)
        if serialized != self._last_published_command:
            self.params.put("NavigationTestTurnCommand", serialized)
            self._last_published_command = serialized

    def publish_decision(self, decision: LaneDecision, ctx: Optional[ManeuverContext],
                         distance: float, eta_seconds: float):
        """Helper that translates a LaneDecision + context into publish_command kwargs."""
        action = decision.action
        # Force direction/display to "none" when there's no actionable decision.
        direction = decision.direction if action != "none" else "none"
        display = decision.display_direction if action != "none" else "none"

        self.publish_command(
            action,
            direction=direction,
            distance=distance,
            eta_seconds=eta_seconds,
            display_direction=display,
            strategy_phase=decision.strategy_phase,
            strategy_constraint=decision.strategy_constraint,
            target_lane_zone=decision.target_lane_zone,
            lane_belief=ctx.lane.belief if ctx else "unknown",
            maneuver_class=ctx.maneuver_class if ctx else "unknown",
            road_context=ctx.road_context if ctx else "unknown",
            route_confidence=ctx.route_confidence if ctx else 0.0,
            block_reason=decision.block_reason,
            urgency=decision.urgency,
        )

    # =======================================================================
    # Debug logging (CSV)
    # =======================================================================

    def debug_log_path(self) -> Optional[str]:
        if self.debug_log_override_path:
            override_dir = os.path.dirname(self.debug_log_override_path)
            if override_dir:
                os.makedirs(override_dir, exist_ok=True)
            return self.debug_log_override_path

        existing = self.params.get("NavigationTestCurrentLog", encoding="utf8")
        if existing:
            existing_dir = os.path.dirname(existing)
            if existing_dir:
                os.makedirs(existing_dir, exist_ok=True)
            return existing

        os.makedirs(self.debug_log_dir, exist_ok=True)
        destination_id = self.params.get("NavigationTestSelectedDestination", encoding="utf8") or "home"
        safe = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in destination_id)
        filename = f"navigation_test_{safe}_{time.strftime('%Y%m%d_%H%M%S', time.localtime())}.csv"
        path = os.path.join(self.debug_log_dir, filename)
        self.params.put("NavigationTestCurrentLog", path)
        self.params.put("NavigationTestLastDriveLog", path)
        return path

    def log_debug_row(self, decision: LaneDecision, ctx: Optional[ManeuverContext],
                      instruction, geometry, distance: float, command_distance: float,
                      cross_track: Optional[float], route_bearing_error: Optional[float]):
        if not self.is_lane_control_enabled():
            return
        now = time.monotonic()
        if now - self.debug_last_log_time < DEBUG_LOG_INTERVAL:
            return
        self.debug_last_log_time = now

        coord = geometry[-1] if geometry else None
        straight = (
            self.last_position.distance_to(coord)
            if self.last_position is not None and coord is not None else None
        )
        destination_id = self.params.get("NavigationTestSelectedDestination", encoding="utf8") or "home"
        migration_age = (
            now - self.exit_migration_started_at
            if self.exit_migration_key is not None else 0.0
        )

        row = {
            "log_version": DEBUG_LOG_VERSION,
            "time": f"{time.time():.3f}",
            "gps_ok": self.gps_ok,
            "localizer_valid": self.localizer_valid,
            "lat": f"{self.last_position.latitude:.7f}" if self.last_position else "",
            "lon": f"{self.last_position.longitude:.7f}" if self.last_position else "",
            "bearing": f"{self.last_bearing:.2f}" if self.last_bearing is not None else "",
            "v_ego": f"{self._v_ego():.2f}",
            "destination": destination_id,
            "step_idx": self.step_idx if self.step_idx is not None else "",
            "step_count": len(self.route) if self.route is not None else "",
            "maneuver_type": instruction.get("maneuverType", "") if instruction else "",
            "maneuver_modifier": instruction.get("maneuverModifier", "") if instruction else "",
            "maneuver_text": instruction.get("maneuverPrimaryText", "") if instruction else "",
            "maneuver_class": ctx.maneuver_class if ctx else "unknown",
            "road_context": ctx.road_context if ctx else "unknown",
            "maneuver_lat": f"{coord.latitude:.7f}" if coord else "",
            "maneuver_lon": f"{coord.longitude:.7f}" if coord else "",
            "distance_to_maneuver_along_route": f"{distance:.2f}",
            "distance_to_maneuver_straight": f"{straight:.2f}" if straight is not None else "",
            "command_threshold": f"{command_distance:.2f}",
            "strategy_phase": decision.strategy_phase,
            "strategy_threshold": f"{decision.strategy_threshold:.2f}",
            "strategy_constraint": decision.strategy_constraint,
            "command_block_reason": decision.block_reason,
            "target_lane_zone": decision.target_lane_zone,
            "lane_belief": ctx.lane.belief if ctx else "unknown",
            "lane_left_available": ctx.lane.left if ctx and ctx.lane.left is not None else "unknown",
            "lane_right_available": ctx.lane.right if ctx and ctx.lane.right is not None else "unknown",
            "route_confident": ctx.route_confident if ctx else False,
            "route_confidence": f"{ctx.route_confidence:.2f}" if ctx else "0.00",
            "route_bearing_error": f"{route_bearing_error:.2f}" if route_bearing_error is not None else "",
            "next_maneuver_direction": ctx.next_direction if ctx else "none",
            "next_maneuver_distance_after_current": (
                f"{ctx.next_distance_after_current:.2f}"
                if ctx and ctx.next_distance_after_current is not None else ""
            ),
            "migration_active": self.exit_migration_key is not None,
            "migration_age_seconds": f"{migration_age:.2f}",
            "migration_start_distance": (
                f"{self.exit_migration_start_distance:.2f}"
                if self.exit_migration_key is not None else ""
            ),
            "post_exit_recenter_active": self.post_exit_recenter_active(),
            "junction_nudge_active": decision.is_junction_nudge,
            "action": decision.action,
            "direction": decision.direction,
            "urgency": f"{decision.urgency:.2f}",
            "cross_track_error": f"{cross_track:.2f}" if cross_track is not None else "",
        }

        try:
            path = self.debug_log_path()
            if not path:
                return

            write_header = not os.path.exists(path) or os.path.getsize(path) == 0
            if not write_header:
                try:
                    with open(path, "r", newline="") as f:
                        first_line = f.readline().strip()
                    if first_line != ",".join(DEBUG_LOG_FIELDS):
                        rotated = path.replace(".csv", f".v{DEBUG_LOG_VERSION - 1}_{int(time.time())}.csv")
                        os.replace(path, rotated)
                        write_header = True
                except OSError:
                    write_header = True

            with open(path, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=DEBUG_LOG_FIELDS)
                if write_header:
                    writer.writeheader()
                writer.writerow(row)
        except OSError:
            cloudlog.exception("navigation_test_debug.failed_to_write")

    # =======================================================================
    # Routing (Mapbox) — recompute logic, threaded fetch, application
    # =======================================================================

    def recompute_route(self):
        if self.last_position is None:
            return

        new_destination = coordinate_from_param("NavDestination", self.params)
        if new_destination is None:
            self.clear_route()
            self.reset_recompute_limits()
            return

        should_recompute = self.should_recompute()
        if new_destination != self.nav_destination:
            cloudlog.warning(f"Got new destination from NavDestination param {new_destination}")
            should_recompute = True

        if not self.gps_ok and self.step_idx is not None:
            return

        if self.recompute_countdown == 0 and should_recompute:
            self.recompute_countdown = self.recompute_route_countdown()
            self.recompute_backoff = min(6, self.recompute_backoff + 1)
            self.calculate_route(new_destination)
            self.reroute_counter = 0
            self.lane_reroute_counter = 0
        else:
            self.recompute_countdown = max(0, self.recompute_countdown - 1)

    def calculate_route(self, destination):
        if self._route_thread is not None and self._route_thread.is_alive():
            cloudlog.warning("Route calculation already in progress. Skipping new request.")
            return

        cloudlog.warning(f"Calculating route {self.last_position} -> {destination}")
        self.nav_destination = destination
        self.reset_destination_tracking(destination)

        if self.is_lane_control_enabled():
            self.publish_command("routing")

        self._route_thread = threading.Thread(
            target=self._fetch_route_worker,
            args=(destination, self.last_position, self.last_bearing),
            daemon=True,
        )
        self._route_thread.start()

    def _fetch_route_worker(self, destination, last_position, last_bearing):
        try:
            waypoints = self.params.get('NavDestinationWaypoints', encoding='utf8')
            waypoint_coords = json.loads(waypoints) if waypoints and len(waypoints) > 0 else []

            coords = [
                (last_position.longitude, last_position.latitude),
                *waypoint_coords,
                (destination.longitude, destination.latitude),
            ]
            coords_str = ';'.join([f'{lon},{lat}' for lon, lat in coords])
            url = self.mapbox_host + '/directions/v5/mapbox/driving-traffic/' + coords_str

            lang = self.params.get('LanguageSetting', encoding='utf8')
            lang = lang.replace('main_', '') if lang else None
            token = self.mapbox_token or self.api.get_token()

            params = {
                'access_token': token,
                'annotations': 'maxspeed',
                'geometries': 'geojson',
                'overview': 'full',
                'steps': 'true',
                'banner_instructions': 'true',
                'alternatives': 'true',
                'language': lang,
                'waypoints': f'0;{len(coords)-1}',
            }
            if last_bearing is not None:
                params['bearings'] = f"{(last_bearing + 360) % 360:.0f},90" + (';' * (len(coords) - 1))

            resp = requests.get(url, params=params, timeout=10)
            if resp.status_code != 200:
                cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)
            resp.raise_for_status()

            r = resp.json()
            r1 = resp.json()

            if not r.get('routes'):
                with self._route_lock:
                    self._pending_route_error = "noRoute"
                return

            chosen_route = r['routes'][0]

            def remove_keys(obj, keys_to_remove):
                if isinstance(obj, list):
                    return [remove_keys(item, keys_to_remove) for item in obj]
                if isinstance(obj, dict):
                    return {key: remove_keys(value, keys_to_remove)
                            for key, value in obj.items() if key not in keys_to_remove}
                return obj

            r2 = remove_keys(r1, ['geometry', 'annotation', 'incidents',
                                  'intersections', 'components', 'sub', 'waypoints'])
            r3 = {}

            if 'routes' in r2 and len(r2['routes']) > 0:
                first_route = r2['routes'][0]
                try:
                    nav_destination_json = self.params.get('NavDestination', encoding='utf8')
                    nav_destination_data = json.loads(nav_destination_json) if nav_destination_json else {}
                    route_hash = nav_destination_data.get('routeHash')

                    if route_hash:
                        for cand in r['routes']:
                            flat = ','.join(
                                str(coordinate)
                                for pair in cand['geometry']['coordinates']
                                for coordinate in pair
                            )
                            if hashlib.sha1(flat.encode()).hexdigest() == route_hash:
                                chosen_route = cand
                                break

                    first_route['Destination'] = nav_destination_data.get('place_name', 'Default Place Name')
                    first_route['Metric'] = self.params.get_bool("IsMetric")
                    r3['CurrentStep'] = 0
                    r3['uuid'] = r2.get('uuid', 'osrm-navigation-test')
                except Exception as e:
                    cloudlog.warning(f"Error parsing destination data in thread: {e}")

            # Side files for the FrogPilot UI / external nav app.
            with open('navdirections.json', 'w') as f:
                json.dump(r2, f, indent=4)
            with open('CurrentStep.json', 'w') as f:
                json.dump(r3, f, indent=4)

            with self._route_lock:
                self._pending_route_result = (r, chosen_route, r2, r3)

        except requests.exceptions.RequestException as e:
            cloudlog.exception("failed to get route in thread")
            with self._route_lock:
                self._pending_route_error = e.__class__.__name__
        except Exception as e:
            cloudlog.exception("unexpected route worker failure")
            with self._route_lock:
                self._pending_route_error = e.__class__.__name__

    def _check_and_apply_route_thread(self):
        with self._route_lock:
            err = self._pending_route_error
            res = self._pending_route_result
            self._pending_route_error = None
            self._pending_route_result = None

        if err:
            if self.is_lane_control_enabled():
                self.publish_command("routeError", error=err)
            self.clear_route()
            self.send_route()
            return

        if res is None:
            return

        r, chosen_route, r2, r3 = res
        self.r2 = r2
        self.r3 = r3

        if not r.get('routes'):
            cloudlog.warning("Got empty route response in applied data")
            self.clear_route()
            self.params.remove('NavDestinationWaypoints')
            self.send_route()
            return

        self.route = chosen_route['legs'][0]['steps']
        self.route_geometry = []

        if self.frogpilot_toggles.conditional_navigation_intersections:
            self.stop_signal = []
            self.stop_coord = []
            for step in self.route:
                for intersection in step.get("intersections", []):
                    if "stop_sign" in intersection or "traffic_signal" in intersection:
                        self.stop_signal.append(intersection["geometry_index"])
                        self.stop_coord.append(Coordinate.from_mapbox_tuple(intersection["location"]))

        maxspeed_idx = 0
        maxspeeds = chosen_route['legs'][0].get('annotation', {}).get('maxspeed', [])

        for step in self.route:
            coords = []
            for c in step['geometry']['coordinates']:
                coord = Coordinate.from_mapbox_tuple(c)
                if maxspeed_idx < len(maxspeeds):
                    maxspeed = maxspeeds[maxspeed_idx]
                    if 'unknown' not in maxspeed and 'none' not in maxspeed:
                        coord.annotations['maxspeed'] = maxspeed_to_ms(maxspeed)
                coords.append(coord)
                maxspeed_idx += 1
            self.route_geometry.append(coords)
            maxspeed_idx -= 1

        self.step_idx = 0

        # Fresh route → forget any junction nudges from the previous one.
        self.reset_junction_nudge()

        self.params.remove('NavDestinationWaypoints')
        self.send_route()

    # =======================================================================
    # Reroute / destination-tracking helpers
    # =======================================================================

    def send_route(self):
        coords = []
        if self.route is not None:
            for path in self.route_geometry:
                coords += [c.as_dict() for c in path]
        msg = messaging.new_message('navRoute', valid=True)
        msg.navRoute.coordinates = coords
        self.pm.send('navRoute', msg)

    def clear_route(self):
        self.route = None
        self.route_geometry = None
        self.step_idx = None
        self.nav_destination = None
        self.lane_reroute_counter = 0
        self.dest_missed_counter = 0
        self.closest_dest_distance = None
        self.reset_exit_migration()
        self.reset_post_exit_recenter()
        self.reset_junction_nudge()

    def reset_recompute_limits(self):
        self.recompute_backoff = 0
        self.recompute_countdown = 0

    def reset_destination_tracking(self, destination=None):
        self.dest_missed_counter = 0
        self.closest_dest_distance = (
            self.last_position.distance_to(destination)
            if self.last_position is not None and destination is not None else None
        )

    def recompute_route_countdown(self) -> int:
        countdown = 2 ** self.recompute_backoff
        if self.is_lane_control_enabled():
            return max(LANE_REROUTE_COUNTDOWN_MIN, countdown)
        return countdown

    def missed_destination(self) -> bool:
        if self.nav_destination is None or self.last_position is None:
            return False

        distance = self.last_position.distance_to(self.nav_destination)
        if self.closest_dest_distance is None or distance < self.closest_dest_distance:
            self.closest_dest_distance = distance
            self.dest_missed_counter = 0
            return False

        if self.closest_dest_distance > NAV_TEST_DEST_APPROACH_DISTANCE:
            return False

        missed = (
            distance > NAV_TEST_DEST_MISSED_DISTANCE
            and distance > self.closest_dest_distance + NAV_TEST_DEST_MISSED_DRIFT
        )
        if missed:
            self.dest_missed_counter += 1
        else:
            self.dest_missed_counter = 0

        if self.dest_missed_counter > NAV_TEST_DEST_MISSED_COUNTER_MIN:
            cloudlog.warning(f"Navigation test missed destination: distance={distance:.1f}m")
            return True
        return False

    def should_recompute(self) -> bool:
        if self.step_idx is None or self.route is None:
            return True

        if self.is_lane_control_enabled():
            cross_track = self.cross_track_error()
            if cross_track is not None and cross_track > self.command_distance():
                self.lane_reroute_counter += 1
            else:
                self.lane_reroute_counter = 0
                if cross_track is not None and cross_track <= LANE_MAX_CROSS_TRACK_ERROR:
                    self.recompute_backoff = 0

            if self.lane_reroute_counter > LANE_REROUTE_COUNTER_MIN:
                cloudlog.warning(f"Navigation test route mismatch: cross_track={cross_track:.1f}m")
                return True

        if self.step_idx == len(self.route) - 1:
            if self.is_lane_control_enabled() and self.missed_destination():
                return True
            return False

        min_d = self.path_minimum_distance(self.route_geometry[self.step_idx])
        if min_d is not None and min_d > REROUTE_DISTANCE:
            self.reroute_counter += 1
        else:
            self.reroute_counter = 0
        return self.reroute_counter > REROUTE_COUNTER_MIN

    # =======================================================================
    # Step-transition logic
    # =======================================================================

    def should_transition_to_next_step(self, distance_to_maneuver: float) -> bool:
        if self.step_idx + 1 >= len(self.route):
            return distance_to_maneuver < -MANEUVER_TRANSITION_THRESHOLD
        if distance_to_maneuver < -MANEUVER_TRANSITION_THRESHOLD:
            return True
        if distance_to_maneuver > MANEUVER_TRANSITION_THRESHOLD:
            return False
        current = self.path_minimum_distance(self.route_geometry[self.step_idx])
        nxt = self.path_minimum_distance(self.route_geometry[self.step_idx + 1])
        return current is not None and nxt is not None and nxt < current

    def advance_step(self, current_instruction):
        if self.step_idx + 1 < len(self.route):
            if self.is_exit_maneuver(current_instruction):
                self.start_post_exit_recenter(maneuver_direction(current_instruction))
            self.step_idx += 1
            self.reset_recompute_limits()

            if 'routes' in self.r2 and len(self.r2['routes']) > 0:
                self.r3['CurrentStep'] = self.step_idx
            self._async_write_json('CurrentStep.json', self.r3)
        else:
            cloudlog.warning("Destination reached")
            dist = self.nav_destination.distance_to(self.last_position)
            if dist > REROUTE_DISTANCE:
                self.params.remove("NavDestination")
                self.clear_route()
            else:
                self.publish_command("none")

    # =======================================================================
    # Per-tick instruction publishing
    # =======================================================================

    def send_instruction(self):
        msg = messaging.new_message('navInstruction', valid=True)
        fp_msg = messaging.new_message('frogpilotNavigation', valid=True)

        # ---- Idle: no active route ---------------------------------------
        if self.step_idx is None:
            msg.valid = False
            self.pm.send('navInstruction', msg)
            fp_msg.frogpilotNavigation.navigationSpeedLimit = 0
            self.pm.send('frogpilotNavigation', fp_msg)
            # In test mode the lane-control state is owned by update_test_destination()
            # / recompute_route() (e.g. "routing", "routeError"). Don't clobber it.
            if not self.is_lane_control_enabled():
                self.publish_command("none")
            return

        # ---- Resolve current step + banner instruction -------------------
        step = self.route[self.step_idx]
        geometry = self.route_geometry[self.step_idx]
        along_geometry = distance_along_geometry(geometry, self.last_position)
        distance_to_maneuver = step['distance'] - along_geometry

        # Final-step fallback: if no banner on the last step, use the previous
        # step's banner (Mapbox sometimes omits the arrival banner).
        banner_step = step
        if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
            banner_step = self.route[max(self.step_idx - 1, 0)]

        instruction = parse_banner_instructions(banner_step['bannerInstructions'], distance_to_maneuver)
        msg.navInstruction.maneuverDistance = distance_to_maneuver
        if instruction is not None:
            for k, v in instruction.items():
                setattr(msg.navInstruction, k, v)

        # ---- All-maneuvers list for the UI -------------------------------
        msg.navInstruction.allManeuvers = self._collect_all_maneuvers(distance_to_maneuver, along_geometry)

        # ---- Time / distance remaining -----------------------------------
        remaining = 1.0 - along_geometry / max(step['distance'], 1)
        total_distance = step['distance'] * remaining
        total_time = step['duration'] * remaining
        total_time_typical = (
            total_time if step['duration_typical'] is None
            else step['duration_typical'] * remaining
        )
        for i in range(self.step_idx + 1, len(self.route)):
            total_distance += self.route[i]['distance']
            total_time += self.route[i]['duration']
            total_time_typical += (
                self.route[i]['duration']
                if self.route[i]['duration_typical'] is None
                else self.route[i]['duration_typical']
            )
        msg.navInstruction.distanceRemaining = total_distance
        msg.navInstruction.timeRemaining = total_time
        msg.navInstruction.timeRemainingTypical = total_time_typical

        # ---- Lane-policy decision (gated by NavigationTestControl) -------
        decision = LaneDecision()
        ctx: Optional[ManeuverContext] = None
        cross_track: Optional[float] = None
        route_bearing_error: Optional[float] = None
        cmd_distance = 0.0

        if self.is_lane_control_enabled():
            cmd_distance = self.command_distance()
            cross_track = self.cross_track_error()
            ctx = self.build_maneuver_context(instruction, geometry, distance_to_maneuver)
            decision = self.decide_lane_action(ctx)
            self.publish_decision(decision, ctx, distance_to_maneuver, total_time)
            # Pull the bearing error for logging (we already computed it inside
            # assess_route_confidence, but a fresh call is cheap and keeps the
            # log column populated even if route_confident was True).
            _, _, route_bearing_error, _ = self.assess_route_confidence(geometry, cross_track)
            self.log_debug_row(decision, ctx, instruction, geometry,
                               distance_to_maneuver, cmd_distance,
                               cross_track, route_bearing_error)

        # ---- Closest geometry point + speed-limit annotations ------------
        closest_idx, closest = min(enumerate(geometry),
                                   key=lambda p: p[1].distance_to(self.last_position))
        if closest_idx > 0:
            if along_geometry < distance_along_geometry(geometry, geometry[closest_idx]):
                closest = geometry[closest_idx - 1]

        if 'maxspeed' in closest.annotations and self.localizer_valid:
            msg.navInstruction.speedLimit = closest.annotations['maxspeed']
            self.nav_speed_limit = closest.annotations['maxspeed']
        if not self.localizer_valid or 'maxspeed' not in closest.annotations:
            self.nav_speed_limit = 0

        if 'speedLimitSign' in step:
            if step['speedLimitSign'] == 'mutcd':
                msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.mutcd
            elif step['speedLimitSign'] == 'vienna':
                msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.vienna

        self.pm.send('navInstruction', msg)

        # ---- Step transition ---------------------------------------------
        if self.should_transition_to_next_step(distance_to_maneuver):
            self.advance_step(instruction)

        # ---- FrogPilot conditional navigation flags ----------------------
        self._populate_frogpilot_nav(fp_msg, distance_to_maneuver, closest_idx)
        self.pm.send('frogpilotNavigation', fp_msg)

    def _collect_all_maneuvers(self, distance_to_maneuver: float, along_geometry: float):
        maneuvers = []
        for i, step_i in enumerate(self.route):
            if i < self.step_idx:
                d = -sum(self.route[j]['distance']
                         for j in range(i + 1, self.step_idx)) - along_geometry
            elif i == self.step_idx:
                d = distance_to_maneuver
            else:
                d = distance_to_maneuver + sum(
                    self.route[j]['distance']
                    for j in range(self.step_idx + 1, i + 1))
            instruction_i = parse_banner_instructions(step_i['bannerInstructions'], d)
            if instruction_i is None:
                continue
            m = {'distance': d}
            if 'maneuverType' in instruction_i:
                m['type'] = instruction_i['maneuverType']
            if 'maneuverModifier' in instruction_i:
                m['modifier'] = instruction_i['maneuverModifier']
            maneuvers.append(m)
        return maneuvers

    def _populate_frogpilot_nav(self, fp_msg, distance_to_maneuver: float, closest_idx: int):
        if self.frogpilot_toggles.conditional_navigation:
            v_ego = self._v_ego()
            seconds_to_stop = interp(v_ego, [0, 22.5, 45], [5, 10, 10])

            closest_condition_indices = [idx for idx in self.stop_signal if idx >= closest_idx]
            if closest_condition_indices:
                closest_condition_index = min(
                    closest_condition_indices,
                    key=lambda idx: abs(closest_idx - idx),
                )
                index = self.stop_signal.index(closest_condition_index)
                distance_to_condition = self.last_position.distance_to(self.stop_coord[index])
                self.approaching_intersection = (
                    self.frogpilot_toggles.conditional_navigation_intersections
                    and distance_to_condition < max(seconds_to_stop * v_ego, 25)
                )
            else:
                self.approaching_intersection = False

            self.approaching_turn = (
                self.frogpilot_toggles.conditional_navigation_turns
                and distance_to_maneuver < max(seconds_to_stop * v_ego, 25)
            )
        else:
            self.approaching_intersection = False
            self.approaching_turn = False

        fp_msg.frogpilotNavigation.approachingIntersection = self.approaching_intersection
        fp_msg.frogpilotNavigation.approachingTurn = self.approaching_turn
        fp_msg.frogpilotNavigation.navigationSpeedLimit = self.nav_speed_limit


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    pm = messaging.PubMaster(['navInstruction', 'navRoute', 'frogpilotNavigation'])
    sm = messaging.SubMaster(['carState', 'liveLocationKalman', 'managerState',
                              'frogpilotPlan', 'modelV2'])

    rk = Ratekeeper(1.0)
    route_engine = RouteEngine(sm, pm)
    while True:
        route_engine.update()
        rk.keep_time()


if __name__ == "__main__":
    main()
