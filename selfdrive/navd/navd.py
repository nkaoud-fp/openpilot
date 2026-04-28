#!/usr/bin/env python3
import csv
import hashlib
import json
import math
import os
import time
import threading

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

REROUTE_DISTANCE = 25
MANEUVER_TRANSITION_THRESHOLD = 10
REROUTE_COUNTER_MIN = 3
NAVIGATION_TEST_DESTINATIONS = {
  "home": ("Navigation test - Home", Coordinate(24.675764, 46.581478)),
  "work": ("Navigation test - Work", Coordinate(24.714778, 46.683775)),
  "school": ("Navigation test - School", Coordinate(24.781423, 46.622246)),
}
NAVIGATION_TEST_SHARED_DESTINATION_URL = "https://frihtcjnhcayqvcphczr.supabase.co/rest/v1/shared_destination?id=eq.1&select=lat,lng"
NAVIGATION_TEST_SHARED_DESTINATION_API_KEY = "sb_publishable_1Lh9fwsQOJppOm82Rk7uyA_nm2qWGdh"
NAVIGATION_TEST_SHARED_DESTINATION_RETRY_SECONDS = 15.0
NAVIGATION_TEST_COMMAND_DISTANCE = 35
NAVIGATION_TEST_COMMAND_SECONDS = 8
NAVIGATION_TEST_EXIT_PREP_SECONDS = 30
NAVIGATION_TEST_EXIT_PREP_DISTANCE_MIN = 250
NAVIGATION_TEST_HIGHWAY_EXIT_PREP_SPEED = 22.0
NAVIGATION_TEST_HIGHWAY_EXIT_PREP_SECONDS = 180
NAVIGATION_TEST_HIGHWAY_EXIT_PREP_DISTANCE_MIN = 1500
NAVIGATION_TEST_HIGHWAY_EXIT_PREP_DISTANCE_MAX = 5000
NAVIGATION_TEST_CONSECUTIVE_CONFLICT_DISTANCE = 400
NAVIGATION_TEST_MAX_COMMAND_CROSS_TRACK_ERROR = 35 #15
NAVIGATION_TEST_REROUTE_COUNTER_MIN = 2
NAVIGATION_TEST_REROUTE_COUNTDOWN_MIN = 5
NAVIGATION_TEST_DESTINATION_APPROACH_DISTANCE = 50
NAVIGATION_TEST_DESTINATION_MISSED_DISTANCE = 80
NAVIGATION_TEST_DESTINATION_MISSED_DRIFT = 30
NAVIGATION_TEST_DESTINATION_MISSED_COUNTER_MIN = 2
NAVIGATION_TEST_DEBUG_LOG_PATH = "/data/media/0/navigation_test_debug.csv"
NAVIGATION_TEST_DEBUG_LOG_INTERVAL = 0.5
# Navigation test control is intentionally conservative: navd is a lane-preparation
# planner, not a steering/turn controller. It should only ask FrogPilot for lane
# changes when a route maneuver needs side-lane positioning.
NAVIGATION_TEST_DEBUG_LOG_VERSION = 2
NAVIGATION_TEST_MAX_BEARING_ERROR = 55.0
NAVIGATION_TEST_LATE_LANE_CHANGE_LOCKOUT_SECONDS = 3.0
NAVIGATION_TEST_LATE_LANE_CHANGE_LOCKOUT_DISTANCE_MIN = 80.0
NAVIGATION_TEST_LANE_CHANGE_COOLDOWN_SECONDS = 10.0
NAVIGATION_TEST_ADJACENT_LANE_PROBABILITY = 0.35
NAVIGATION_TEST_POST_EXIT_RECENTER_SECONDS = 20.0
NAVIGATION_TEST_POST_EXIT_RECENTER_CONFLICT_DISTANCE = 500.0
NAVIGATION_TEST_DEBUG_LOG_FIELDS = [
  "log_version",
  "time",
  "gps_ok",
  "localizer_valid",
  "lat",
  "lon",
  "bearing",
  "v_ego",
  "destination",
  "step_idx",
  "step_count",
  "maneuver_type",
  "maneuver_modifier",
  "maneuver_text",
  "maneuver_class",
  "road_context",
  "maneuver_lat",
  "maneuver_lon",
  "distance_to_maneuver_along_route",
  "distance_to_maneuver_straight",
  "command_threshold",
  "strategy_phase",
  "strategy_threshold",
  "strategy_constraint",
  "command_block_reason",
  "target_lane_zone",
  "lane_belief",
  "lane_left_available",
  "lane_right_available",
  "route_confident",
  "route_confidence",
  "route_bearing_error",
  "next_maneuver_direction",
  "next_maneuver_distance_after_current",
  "migration_active",
  "migration_age_seconds",
  "migration_start_distance",
  "post_exit_recenter_active",
  "action",
  "direction",
  "urgency",
  "cross_track_error",
]


class RouteEngine:
  def __init__(self, sm, pm):
    self.sm = sm
    self.pm = pm

    self.params = Params()

    # Get last gps position from params
    self.last_position = coordinate_from_param("LastGPSPosition", self.params)
    self.last_bearing = None

    self.gps_ok = False
    self.localizer_valid = False

    self.nav_destination = None
    self.step_idx = None
    self.route = None
    self.route_geometry = None

    self.recompute_backoff = 0
    self.recompute_countdown = 0

    self.ui_pid = None

    self.reroute_counter = 0
    self.navigation_test_reroute_counter = 0
    self.navigation_test_destination_missed_counter = 0
    self.navigation_test_closest_destination_distance = None
    self.navigation_test_command = None
    self.navigation_test_shared_destination = None
    self.navigation_test_shared_destination_retry_at = 0.0
    self.navigation_test_last_handled_share_selection_token = ""
    self.navigation_test_exit_migration_key = None
    self.navigation_test_exit_migration_direction = "none"
    self.navigation_test_exit_migration_started_at = 0.0
    self.navigation_test_exit_migration_start_distance = 0.0
    self.navigation_test_debug_last_log_time = 0.0
    self.navigation_test_debug_log_path = os.environ.get("NAVIGATION_TEST_DEBUG_LOG_PATH", NAVIGATION_TEST_DEBUG_LOG_PATH)
    self.navigation_test_last_lane_change_command_at = 0.0
    self.navigation_test_last_lane_change_command_direction = "none"
    self.navigation_test_last_lane_change_command_distance = 0.0
    self.navigation_test_post_exit_recenter_direction = "none"
    self.navigation_test_post_exit_recenter_exit_direction = "none"
    self.navigation_test_post_exit_recenter_started_at = 0.0
    self.navigation_test_post_exit_recenter_expires_at = 0.0
    self.navigation_test_post_exit_recenter_done = False

    # Threading variables
    self.route_thread = None
    self._pending_route_result = None
    self._pending_route_error = None
    self.r2 = {}
    self.r3 = {}

    self.api = Api(self.params.get("DongleId", encoding='utf8'))
    self.mapbox_host = "https://api.mapbox.com"
    #self.mapbox_token = None
    
    # Get the directory where navd.py is located and point to the token file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    token_file_path = os.path.join(current_dir, "mapbox_token")
    
    try:
      with open(token_file_path, "r") as f:
        self.mapbox_token = f.read().strip()
    except FileNotFoundError:
      self.mapbox_token = None
      cloudlog.warning(f"Mapbox token file not found at {token_file_path}!")
    
    #if "MAPBOX_TOKEN" in os.environ:
      #self.mapbox_token = os.environ["MAPBOX_TOKEN"]
      #self.mapbox_host = "https://api.mapbox.com"
    #else:
      #self.mapbox_token = self.params.get("MapboxSecretKey", encoding='utf8')
      #self.mapbox_host = "https://api.mapbox.com"

    # FrogPilot variables
    self.approaching_intersection = False
    self.approaching_turn = False

    self.nav_speed_limit = 0

    self.stop_coord = []
    self.stop_signal = []

    self.frogpilot_toggles = get_frogpilot_toggles()

  def _async_write_json(self, filepath, data):
    """Writes JSON files in a background thread to prevent loop blocking."""
    def write_task():
      try:
        with open(filepath, 'w') as f:
          json.dump(data, f, indent=4)
      except Exception as e:
        cloudlog.warning(f"Failed to async write {filepath}: {e}")
    threading.Thread(target=write_task, daemon=True).start()

  def update(self):
    self.sm.update(0)

    if self.sm.updated["managerState"]:
      ui_pid = [p.pid for p in self.sm["managerState"].processes if p.name == "ui" and p.running]
      if ui_pid:
        if self.ui_pid and self.ui_pid != ui_pid[0]:
          cloudlog.warning("UI restarting, sending route")
          threading.Timer(5.0, self.send_route).start()
        self.ui_pid = ui_pid[0]

    self.update_location()
    try:
      self.update_navigation_test_destination()
      self.recompute_route()
      
      # Check if background thread has a route ready
      self._check_and_apply_route_thread()
      
      self.send_instruction()
    except Exception as err:
      if self.params.get_bool("NavigationTestControl"):
        self.update_navigation_test_command("routeError", error=err.__class__.__name__)
      cloudlog.exception("navd.failed_to_compute")

    # Update FrogPilot variables
    if self.sm['frogpilotPlan'].togglesUpdated:
      self.frogpilot_toggles = get_frogpilot_toggles()

  def update_location(self):
    location = self.sm['liveLocationKalman']
    self.gps_ok = location.gpsOK

    self.localizer_valid = (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid

    if self.localizer_valid:
      self.last_bearing = math.degrees(location.calibratedOrientationNED.value[2])
      self.last_position = Coordinate(location.positionGeodetic.value[0], location.positionGeodetic.value[1])

  def update_navigation_test_destination(self):
    if not self.params.get_bool("NavigationTestControl"):
      self.update_navigation_test_command("none")
      return

    if self.last_position is None:
      self.update_navigation_test_command("waitingGps")
      return

    destination_id = self.params.get("NavigationTestSelectedDestination", encoding="utf8") or "home"
    if destination_id == "share":
      share_selection_token = self.params.get("NavigationTestShareSelectionToken", encoding="utf8")
      force_refresh = share_selection_token != self.navigation_test_last_handled_share_selection_token
      destination = coordinate_from_param("NavDestination", self.params)
      shared_destination = self.get_navigation_test_shared_destination(force_refresh or destination is None)
      if shared_destination is None:
        return
      self.navigation_test_last_handled_share_selection_token = share_selection_token
      destination_name, target_destination = shared_destination
    else:
      destination_name, target_destination = NAVIGATION_TEST_DESTINATIONS.get(destination_id, NAVIGATION_TEST_DESTINATIONS["home"])
      destination = coordinate_from_param("NavDestination", self.params)
    if destination == target_destination:
      return

    self.update_navigation_test_command("routing")
    self.params.put("NavDestination", json.dumps({
      "latitude": target_destination.latitude,
      "longitude": target_destination.longitude,
      "place_name": destination_name,
    }))

  def get_navigation_test_shared_destination(self, force_refresh=False):
    now = time.monotonic()
    if not force_refresh and self.navigation_test_shared_destination is not None:
      return self.navigation_test_shared_destination
    if not force_refresh and now < self.navigation_test_shared_destination_retry_at:
      return self.navigation_test_shared_destination

    try:
      response = requests.get(
        NAVIGATION_TEST_SHARED_DESTINATION_URL,
        timeout=5,
        headers={
          "apikey": NAVIGATION_TEST_SHARED_DESTINATION_API_KEY,
          "Authorization": f"Bearer {NAVIGATION_TEST_SHARED_DESTINATION_API_KEY}",
        },
      )
      response.raise_for_status()
      payload = response.json()
    except requests.RequestException as err:
      cloudlog.warning(f"Navigation test shared destination fetch failed: {err}")
      self.navigation_test_shared_destination_retry_at = now + NAVIGATION_TEST_SHARED_DESTINATION_RETRY_SECONDS
      self.update_navigation_test_command("routeError", error="sharedFetchFailed")
      return None
    except ValueError as err:
      cloudlog.warning(f"Navigation test shared destination JSON parse failed: {err}")
      self.navigation_test_shared_destination_retry_at = now + NAVIGATION_TEST_SHARED_DESTINATION_RETRY_SECONDS
      self.update_navigation_test_command("routeError", error="sharedInvalidJson")
      return None

    record = payload[0] if isinstance(payload, list) and payload else payload
    if not isinstance(record, dict):
      cloudlog.warning(f"Navigation test shared destination has invalid payload: {payload}")
      self.navigation_test_shared_destination_retry_at = now + NAVIGATION_TEST_SHARED_DESTINATION_RETRY_SECONDS
      self.update_navigation_test_command("routeError", error="sharedInvalidPayload")
      return None

    try:
      latitude = float(record["lat"])
      longitude = float(record["lng"])
    except (KeyError, TypeError, ValueError) as err:
      cloudlog.warning(f"Navigation test shared destination missing coordinates: {err}")
      self.navigation_test_shared_destination_retry_at = now + NAVIGATION_TEST_SHARED_DESTINATION_RETRY_SECONDS
      self.update_navigation_test_command("routeError", error="sharedInvalidCoordinates")
      return None

    if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
      cloudlog.warning(f"Navigation test shared destination out of bounds: {(latitude, longitude)}")
      self.navigation_test_shared_destination_retry_at = now + NAVIGATION_TEST_SHARED_DESTINATION_RETRY_SECONDS
      self.update_navigation_test_command("routeError", error="sharedOutOfBounds")
      return None

    self.navigation_test_shared_destination = ("Navigation test - Share", Coordinate(latitude, longitude))
    self.navigation_test_shared_destination_retry_at = 0.0
    return self.navigation_test_shared_destination

  def update_navigation_test_command(self, action, direction="none", distance=0.0, eta_seconds=0.0, display_direction=None, error="", strategy_phase="none", strategy_constraint="none", target_lane_zone="none", lane_belief="unknown", maneuver_class="unknown", road_context="unknown", route_confidence=0.0, command_block_reason="none", urgency=0.0):
    migration_age = time.monotonic() - self.navigation_test_exit_migration_started_at if self.navigation_test_exit_migration_key is not None else 0.0
    post_exit_recenter_active = self.navigation_test_post_exit_recenter_active()
    command = json.dumps({
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
      "routeConfidence": max(min(route_confidence, 1.0), 0.0),
      "commandBlockReason": command_block_reason,
      "urgency": max(min(urgency, 1.0), 0.0),
      "migrationActive": self.navigation_test_exit_migration_key is not None,
      "migrationAgeSeconds": max(migration_age, 0.0),
      "migrationStartDistance": max(self.navigation_test_exit_migration_start_distance, 0.0),
      "postExitRecenterActive": post_exit_recenter_active,
    })
    if command != self.navigation_test_command:
      self.params.put("NavigationTestTurnCommand", command)
      self.navigation_test_command = command

  def navigation_test_maneuver_direction(self, instruction):
    if instruction is None:
      return "none"

    modifier = instruction.get("maneuverModifier", "").lower()
    maneuver_type = instruction.get("maneuverType", "").lower()
    direction_text = f"{maneuver_type} {modifier}"
    if "left" in direction_text:
      return "left"
    if "right" in direction_text:
      return "right"
    return "none"

  def navigation_test_maneuver_display_direction(self, instruction):
    if instruction is None:
      return "none"

    modifier = instruction.get("maneuverModifier", "").lower().replace(" ", "_")
    maneuver_type = instruction.get("maneuverType", "").lower().replace(" ", "_")
    if "uturn" in (modifier, maneuver_type):
      return "uturn"
    if modifier in ("slight_left", "sharp_left", "left", "slight_right", "sharp_right", "right"):
      return modifier

    direction = self.navigation_test_maneuver_direction(instruction)
    return direction

  def navigation_test_maneuver_class(self, instruction):
    if instruction is None:
      return "none"

    maneuver_type = instruction.get("maneuverType", "").lower()
    modifier = instruction.get("maneuverModifier", "").lower()
    primary_text = instruction.get("maneuverPrimaryText", "").lower()
    direction = self.navigation_test_maneuver_direction(instruction)
    display_direction = self.navigation_test_maneuver_display_direction(instruction)
    text = f"{maneuver_type} {modifier} {primary_text}"

    if display_direction == "uturn" or "uturn" in text or "u-turn" in text:
      return "uturn"
    if "arrive" in text or "destination" in text:
      return "arrive"
    if "roundabout" in text or "rotary" in text:
      return "roundabout"
    if "ramp" in maneuver_type or "exit" in primary_text or "take exit" in text:
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

  def navigation_test_is_lane_positioning_maneuver(self, maneuver_class):
    return maneuver_class in ("highway_exit", "highway_fork")

  def navigation_test_road_context(self, maneuver_class):
    v_ego = self.sm['carState'].vEgo
    if maneuver_class in ("highway_exit", "highway_fork", "highway_merge"):
      return "highway"
    if v_ego >= NAVIGATION_TEST_HIGHWAY_EXIT_PREP_SPEED:
      return "highway"
    if self.nav_speed_limit >= NAVIGATION_TEST_HIGHWAY_EXIT_PREP_SPEED:
      return "highway"
    return "surface"

  def navigation_test_lane_availability(self):
    # Best-effort adjacent-lane estimate from modelV2 lane-line probabilities.
    # It is intentionally used as a soft gate only. The actual lane-change module
    # should still veto if the requested adjacent lane is not available.
    left_available = None
    right_available = None
    confidence = 0.0
    try:
      model = self.sm['modelV2']
      lane_line_probs = list(model.laneLineProbs)
      if len(lane_line_probs) >= 4:
        # openpilot models expose four lane-line probabilities. The outer lane
        # lines are a useful proxy for adjacent-lane availability.
        left_score = float(lane_line_probs[0])
        right_score = float(lane_line_probs[3])
        left_available = left_score >= NAVIGATION_TEST_ADJACENT_LANE_PROBABILITY
        right_available = right_score >= NAVIGATION_TEST_ADJACENT_LANE_PROBABILITY
        confidence = max(left_score, right_score)
    except Exception:
      pass
    return left_available, right_available, confidence

  def navigation_test_lane_belief(self, left_available=None, right_available=None):
    if left_available is None or right_available is None:
      return "unknown"
    if left_available and right_available:
      return "interior"
    if left_available and not right_available:
      return "right_edge"
    if not left_available and right_available:
      return "left_edge"
    return "single_or_unknown"

  def navigation_test_direction_available(self, direction, left_available=None, right_available=None):
    if direction == "left":
      return left_available is not False
    if direction == "right":
      return right_available is not False
    return False

  def navigation_test_target_zone_for_direction(self, direction):
    if direction == "left":
      return "left_edge"
    if direction == "right":
      return "right_edge"
    return "none"

  def navigation_test_target_edge_reached(self, target_lane_zone, lane_belief):
    return target_lane_zone != "none" and lane_belief == target_lane_zone

  def navigation_test_late_lane_change_lockout_distance(self):
    v_ego = self.sm['carState'].vEgo
    return max(NAVIGATION_TEST_LATE_LANE_CHANGE_LOCKOUT_DISTANCE_MIN, v_ego * NAVIGATION_TEST_LATE_LANE_CHANGE_LOCKOUT_SECONDS)

  def navigation_test_lane_command_allowed(self, direction):
    now = time.monotonic()
    if direction == "none":
      return False, "invalidDirection"
    if now - self.navigation_test_last_lane_change_command_at < NAVIGATION_TEST_LANE_CHANGE_COOLDOWN_SECONDS:
      return False, "laneCommandCooldown"
    return True, "none"

  def record_navigation_test_lane_command(self, direction, distance_to_maneuver_along_geometry):
    self.navigation_test_last_lane_change_command_at = time.monotonic()
    self.navigation_test_last_lane_change_command_direction = direction
    self.navigation_test_last_lane_change_command_distance = distance_to_maneuver_along_geometry

  def navigation_test_angle_diff(self, a, b):
    return (a - b + 180.0) % 360.0 - 180.0

  def navigation_test_bearing_between(self, a, b):
    lat1 = math.radians(a.latitude)
    lat2 = math.radians(b.latitude)
    d_lon = math.radians(b.longitude - a.longitude)
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

  def navigation_test_closest_segment_bearing(self, path):
    if self.last_position is None or path is None or len(path) < 2:
      return None

    closest_distance = None
    closest_bearing = None
    for i in range(len(path) - 1):
      segment = [path[i], path[i + 1]]
      distance = self.path_minimum_distance(segment)
      if distance is None:
        continue
      if closest_distance is None or distance < closest_distance:
        closest_distance = distance
        closest_bearing = self.navigation_test_bearing_between(path[i], path[i + 1])
    return closest_bearing

  def navigation_test_route_confidence(self, geometry, cross_track_error):
    route_bearing_error = None
    if not self.gps_ok:
      return False, 0.0, route_bearing_error, "gpsInvalid"
    if not self.localizer_valid:
      return False, 0.0, route_bearing_error, "localizerInvalid"

    score = 1.0
    if cross_track_error is not None:
      score = min(score, max(0.0, 1.0 - cross_track_error / max(NAVIGATION_TEST_MAX_COMMAND_CROSS_TRACK_ERROR, 1.0)))
      if cross_track_error > NAVIGATION_TEST_MAX_COMMAND_CROSS_TRACK_ERROR:
        return False, score, route_bearing_error, "crossTrackError"

    route_bearing = self.navigation_test_closest_segment_bearing(geometry)
    if route_bearing is not None and self.last_bearing is not None:
      route_bearing_error = abs(self.navigation_test_angle_diff(self.last_bearing, route_bearing))
      score = min(score, max(0.0, 1.0 - route_bearing_error / 90.0))
      if self.sm['carState'].vEgo > 5.0 and route_bearing_error > NAVIGATION_TEST_MAX_BEARING_ERROR:
        return False, score, route_bearing_error, "bearingError"

    return True, score, route_bearing_error, "none"

  def reset_navigation_test_post_exit_recenter(self):
    self.navigation_test_post_exit_recenter_direction = "none"
    self.navigation_test_post_exit_recenter_exit_direction = "none"
    self.navigation_test_post_exit_recenter_started_at = 0.0
    self.navigation_test_post_exit_recenter_expires_at = 0.0
    self.navigation_test_post_exit_recenter_done = False

  def navigation_test_post_exit_recenter_active(self):
    return (
      self.navigation_test_post_exit_recenter_direction in ("left", "right") and
      not self.navigation_test_post_exit_recenter_done and
      time.monotonic() <= self.navigation_test_post_exit_recenter_expires_at
    )

  def start_navigation_test_post_exit_recenter(self, exit_direction):
    if exit_direction == "right":
      recenter_direction = "left"
    elif exit_direction == "left":
      recenter_direction = "right"
    else:
      self.reset_navigation_test_post_exit_recenter()
      return

    now = time.monotonic()
    self.navigation_test_post_exit_recenter_direction = recenter_direction
    self.navigation_test_post_exit_recenter_exit_direction = exit_direction
    self.navigation_test_post_exit_recenter_started_at = now
    self.navigation_test_post_exit_recenter_expires_at = now + NAVIGATION_TEST_POST_EXIT_RECENTER_SECONDS
    self.navigation_test_post_exit_recenter_done = False

  def navigation_test_post_exit_recenter_decision(self, next_maneuver_direction="none", next_maneuver_distance_after_current=None):
    if not self.navigation_test_post_exit_recenter_active():
      if self.navigation_test_post_exit_recenter_direction != "none" and time.monotonic() > self.navigation_test_post_exit_recenter_expires_at:
        self.reset_navigation_test_post_exit_recenter()
      return None

    # Do not escape away from the exit side if another maneuver to the same side is immediate.
    if (
      next_maneuver_direction == self.navigation_test_post_exit_recenter_exit_direction and
      next_maneuver_distance_after_current is not None and
      0.0 < next_maneuver_distance_after_current <= NAVIGATION_TEST_POST_EXIT_RECENTER_CONFLICT_DISTANCE
    ):
      return None

    direction = self.navigation_test_post_exit_recenter_direction
    left_available, right_available, _ = self.navigation_test_lane_availability()
    if not self.navigation_test_direction_available(direction, left_available, right_available):
      self.navigation_test_post_exit_recenter_done = True
      return None

    allowed, block_reason = self.navigation_test_lane_command_allowed(direction)
    if not allowed:
      return ("upcoming", direction, direction, "postExitRecenterCooldown", 0.0, block_reason, "interior", block_reason, 0.25)

    self.record_navigation_test_lane_command(direction, 0.0)
    self.navigation_test_post_exit_recenter_done = True
    return ("laneChange", direction, direction, "postExitRecenter", 0.0, "none", "interior", "none", 0.65)

  def navigation_test_is_exit_maneuver(self, instruction):
    if instruction is None:
      return False

    maneuver_type = instruction.get("maneuverType", "").lower()
    primary_text = instruction.get("maneuverPrimaryText", "").lower()
    return "ramp" in maneuver_type or "exit" in primary_text

  def navigation_test_maneuver_key(self, instruction, geometry):
    if instruction is None:
      return None

    maneuver_coordinate = geometry[-1] if geometry else None
    maneuver_latitude = round(maneuver_coordinate.latitude, 5) if maneuver_coordinate is not None else None
    maneuver_longitude = round(maneuver_coordinate.longitude, 5) if maneuver_coordinate is not None else None
    return (
      instruction.get("maneuverType", ""),
      instruction.get("maneuverModifier", ""),
      instruction.get("maneuverPrimaryText", ""),
      maneuver_latitude,
      maneuver_longitude,
    )

  def reset_navigation_test_exit_migration(self):
    self.navigation_test_exit_migration_key = None
    self.navigation_test_exit_migration_direction = "none"
    self.navigation_test_exit_migration_started_at = 0.0
    self.navigation_test_exit_migration_start_distance = 0.0

  def update_navigation_test_exit_migration(self, instruction, geometry, direction, distance_to_maneuver_along_geometry):
    migration_key = self.navigation_test_maneuver_key(instruction, geometry)
    if migration_key is None or direction == "none":
      self.reset_navigation_test_exit_migration()
      return

    if migration_key != self.navigation_test_exit_migration_key or direction != self.navigation_test_exit_migration_direction:
      self.navigation_test_exit_migration_key = migration_key
      self.navigation_test_exit_migration_direction = direction
      self.navigation_test_exit_migration_started_at = time.monotonic()
      self.navigation_test_exit_migration_start_distance = distance_to_maneuver_along_geometry

  def navigation_test_command_distance(self):
    v_ego = self.sm['carState'].vEgo
    return max(NAVIGATION_TEST_COMMAND_DISTANCE, v_ego * NAVIGATION_TEST_COMMAND_SECONDS)

  def navigation_test_exit_prep_distance(self):
    v_ego = self.sm['carState'].vEgo
    return max(NAVIGATION_TEST_EXIT_PREP_DISTANCE_MIN, v_ego * NAVIGATION_TEST_EXIT_PREP_SECONDS)

  def navigation_test_highway_exit_prep_distance(self):
    v_ego = self.sm['carState'].vEgo
    if v_ego < NAVIGATION_TEST_HIGHWAY_EXIT_PREP_SPEED:
      return self.navigation_test_exit_prep_distance()
    return min(
      NAVIGATION_TEST_HIGHWAY_EXIT_PREP_DISTANCE_MAX,
      max(NAVIGATION_TEST_HIGHWAY_EXIT_PREP_DISTANCE_MIN, v_ego * NAVIGATION_TEST_HIGHWAY_EXIT_PREP_SECONDS),
    )

  def navigation_test_next_maneuver(self, distance_to_maneuver_along_geometry):
    if self.route is None or self.step_idx is None:
      return "none", None, None

    cumulative_distance = distance_to_maneuver_along_geometry
    for i in range(self.step_idx + 1, len(self.route)):
      cumulative_distance += self.route[i]['distance']
      instruction = parse_banner_instructions(self.route[i]['bannerInstructions'], cumulative_distance)
      if instruction is None:
        continue

      direction = self.navigation_test_maneuver_direction(instruction)
      display_direction = self.navigation_test_maneuver_display_direction(instruction)
      if direction != "none" or display_direction == "uturn":
        return direction, cumulative_distance, instruction

    return "none", None, None

  def navigation_test_strategy(self, instruction, geometry, distance_to_maneuver_along_geometry, command_distance, next_maneuver_direction="none", next_maneuver_distance_after_current=None):
    direction = self.navigation_test_maneuver_direction(instruction)
    display_direction = self.navigation_test_maneuver_display_direction(instruction)
    maneuver_class = self.navigation_test_maneuver_class(instruction)
    road_context = self.navigation_test_road_context(maneuver_class)
    target_lane_zone = self.navigation_test_target_zone_for_direction(direction)
    left_available, right_available, lane_confidence = self.navigation_test_lane_availability()
    lane_belief = self.navigation_test_lane_belief(left_available, right_available)
    strategy_phase = "none"
    strategy_threshold = 0.0
    strategy_constraint = "none"
    command_block_reason = "none"
    action = "none"
    urgency = 0.0

    post_exit_decision = self.navigation_test_post_exit_recenter_decision(next_maneuver_direction, next_maneuver_distance_after_current)
    if post_exit_decision is not None:
      action, direction, display_direction, strategy_phase, strategy_threshold, strategy_constraint, target_lane_zone, command_block_reason, urgency = post_exit_decision
      return action, direction, display_direction, strategy_phase, strategy_threshold, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, command_block_reason, urgency

    if direction == "none" and display_direction != "uturn":
      self.reset_navigation_test_exit_migration()
      return action, direction, display_direction, strategy_phase, strategy_threshold, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, command_block_reason, urgency

    # Normal surface-street turns, u-turns, roundabouts, and arrival instructions are
    # guidance only. FrogPilot will follow road curvature, but navd should not convert
    # them into lane-change requests.
    if not self.navigation_test_is_lane_positioning_maneuver(maneuver_class):
      self.reset_navigation_test_exit_migration()
      if distance_to_maneuver_along_geometry <= command_distance:
        return "guidanceOnly", direction, display_direction, "guidanceOnly", command_distance, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, command_block_reason, min(1.0, command_distance / max(distance_to_maneuver_along_geometry, 1.0))
      return "upcoming", direction, display_direction, "upcoming", command_distance, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, command_block_reason, 0.0

    standard_exit_prep_distance = self.navigation_test_exit_prep_distance()
    highway_exit_prep_distance = self.navigation_test_highway_exit_prep_distance()
    active_prep_distance = highway_exit_prep_distance if road_context == "highway" else standard_exit_prep_distance
    late_lockout_distance = self.navigation_test_late_lane_change_lockout_distance()
    strategy_threshold = active_prep_distance
    urgency = max(0.0, min(1.0, 1.0 - distance_to_maneuver_along_geometry / max(active_prep_distance, 1.0)))

    conflict_soon = (
      next_maneuver_direction in ("left", "right") and
      next_maneuver_direction != direction and
      next_maneuver_distance_after_current is not None and
      0.0 < next_maneuver_distance_after_current <= NAVIGATION_TEST_CONSECUTIVE_CONFLICT_DISTANCE
    )

    if distance_to_maneuver_along_geometry > active_prep_distance:
      self.reset_navigation_test_exit_migration()
      return "upcoming", direction, display_direction, "upcoming", active_prep_distance, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, command_block_reason, urgency

    self.update_navigation_test_exit_migration(instruction, geometry, direction, distance_to_maneuver_along_geometry)

    if conflict_soon and distance_to_maneuver_along_geometry > standard_exit_prep_distance:
      self.reset_navigation_test_exit_migration()
      return "upcoming", direction, display_direction, "consecutiveConflictHold", active_prep_distance, "conflictingNextManeuver", target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, "conflictingNextManeuver", urgency

    if distance_to_maneuver_along_geometry <= late_lockout_distance:
      return "guidanceOnly", direction, display_direction, "maneuverLockout", late_lockout_distance, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, "lateLaneChangeLockout", 1.0

    if self.navigation_test_target_edge_reached(target_lane_zone, lane_belief):
      return "upcoming", direction, display_direction, "targetEdgeHold", active_prep_distance, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, "targetEdgeReached", urgency

    if not self.navigation_test_direction_available(direction, left_available, right_available):
      return "upcoming", direction, display_direction, "targetEdgeHold", active_prep_distance, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, "adjacentLaneUnavailable", urgency

    allowed, block_reason = self.navigation_test_lane_command_allowed(direction)
    if not allowed:
      strategy_phase = "laneCommandCooldown"
      if conflict_soon:
        strategy_constraint = "conflictingNextManeuver"
      return "upcoming", direction, display_direction, strategy_phase, active_prep_distance, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, block_reason, urgency

    if distance_to_maneuver_along_geometry <= standard_exit_prep_distance:
      strategy_phase = "exitMigration"
    else:
      strategy_phase = "highwayExitMigration"

    if conflict_soon:
      strategy_constraint = "conflictingNextManeuver"

    self.record_navigation_test_lane_command(direction, distance_to_maneuver_along_geometry)
    return "laneChange", direction, display_direction, strategy_phase, active_prep_distance, strategy_constraint, target_lane_zone, lane_belief, left_available, right_available, maneuver_class, road_context, command_block_reason, urgency

  def navigation_test_cross_track_error(self):
    if self.route_geometry is None or self.last_position is None:
      return None

    closest_distance = None
    for geometry in self.route_geometry:
      distance = self.path_minimum_distance(geometry)
      if distance is None:
        continue
      closest_distance = distance if closest_distance is None else min(closest_distance, distance)
    return closest_distance

  def path_minimum_distance(self, path):
    if self.last_position is None or len(path) < 2:
      return None

    # 1. Extract coordinates into a fast numpy array
    coords = np.array([(c.latitude, c.longitude) for c in path])
    
    # Car position
    p_lat, p_lon = self.last_position.latitude, self.last_position.longitude
    
    # 2. Fast equirectangular projection (converting degrees to meters)
    # Earth radius in meters is approx 6371000
    R = 6371000.0
    deg_to_rad = np.pi / 180.0
    
    # Center projection directly on the car's position (0,0)
    y = (coords[:, 0] - p_lat) * deg_to_rad * R
    x = (coords[:, 1] - p_lon) * np.cos(p_lat * deg_to_rad) * deg_to_rad * R
    
    # 3. Vectorized point-to-segment distance calculations
    # A = segment starts, B = segment ends
    A_x, A_y = x[:-1], y[:-1]
    B_x, B_y = x[1:], y[1:]
    
    # Vector AB (The line segments)
    AB_x = B_x - A_x
    AB_y = B_y - A_y
    
    # Vector AP (Since car is at 0,0, P - A is just -A)
    AP_x = -A_x
    AP_y = -A_y
    
    # Project AP onto AB to find the closest point scalar 't'
    AB_dot_AB = AB_x**2 + AB_y**2
    AP_dot_AB = AP_x * AB_x + AP_y * AB_y
    
    # Ignore division by zero for zero-length segments, clamp t to [0, 1]
    with np.errstate(invalid='ignore', divide='ignore'):
      t = AP_dot_AB / AB_dot_AB
      t = np.clip(t, 0.0, 1.0)
    
    # Convert NaNs back to 0
    t = np.nan_to_num(t)
    
    # 4. Calculate distances from car (0,0) to closest points on the segments
    C_x = A_x + t * AB_x
    C_y = A_y + t * AB_y
    
    distances = np.sqrt(C_x**2 + C_y**2)
    
    # Ignore segments that are < 1.0 meter long (matching your original logic)
    segment_lengths = np.sqrt(AB_dot_AB)
    valid_distances = distances[segment_lengths >= 1.0]
    
    if len(valid_distances) == 0:
      return None
      
    return float(np.min(valid_distances))

  def should_transition_to_next_step(self, distance_to_maneuver_along_geometry):
    if self.step_idx + 1 >= len(self.route):
      return distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD

    if distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD:
      return True

    if distance_to_maneuver_along_geometry > MANEUVER_TRANSITION_THRESHOLD:
      return False

    current_distance = self.path_minimum_distance(self.route_geometry[self.step_idx])
    next_distance = self.path_minimum_distance(self.route_geometry[self.step_idx + 1])
    return current_distance is not None and next_distance is not None and next_distance < current_distance

  def log_navigation_test_debug(self, instruction, geometry, distance_to_maneuver_along_geometry, command_distance, action, direction, cross_track_error=None, strategy_phase="none", strategy_threshold=0.0, strategy_constraint="none", next_maneuver_direction="none", next_maneuver_distance_after_current=None, maneuver_class="unknown", road_context="unknown", target_lane_zone="none", lane_belief="unknown", lane_left_available=None, lane_right_available=None, route_confident=False, route_confidence=0.0, route_bearing_error=None, command_block_reason="none", urgency=0.0):
    if not self.params.get_bool("NavigationTestControl"):
      return

    now = time.monotonic()
    if now - self.navigation_test_debug_last_log_time < NAVIGATION_TEST_DEBUG_LOG_INTERVAL:
      return
    self.navigation_test_debug_last_log_time = now

    maneuver_coordinate = geometry[-1] if geometry else None
    distance_to_maneuver_straight = self.last_position.distance_to(maneuver_coordinate) if self.last_position is not None and maneuver_coordinate is not None else None
    if cross_track_error is None:
      cross_track_error = self.navigation_test_cross_track_error()

    destination_id = self.params.get("NavigationTestSelectedDestination", encoding="utf8") or "home"
    migration_age = time.monotonic() - self.navigation_test_exit_migration_started_at if self.navigation_test_exit_migration_key is not None else 0.0
    row = {
      "log_version": NAVIGATION_TEST_DEBUG_LOG_VERSION,
      "time": f"{time.time():.3f}",
      "gps_ok": self.gps_ok,
      "localizer_valid": self.localizer_valid,
      "lat": f"{self.last_position.latitude:.7f}" if self.last_position is not None else "",
      "lon": f"{self.last_position.longitude:.7f}" if self.last_position is not None else "",
      "bearing": f"{self.last_bearing:.2f}" if self.last_bearing is not None else "",
      "v_ego": f"{self.sm['carState'].vEgo:.2f}",
      "destination": destination_id,
      "step_idx": self.step_idx if self.step_idx is not None else "",
      "step_count": len(self.route) if self.route is not None else "",
      "maneuver_type": instruction.get("maneuverType", "") if instruction is not None else "",
      "maneuver_modifier": instruction.get("maneuverModifier", "") if instruction is not None else "",
      "maneuver_text": instruction.get("maneuverPrimaryText", "") if instruction is not None else "",
      "maneuver_class": maneuver_class,
      "road_context": road_context,
      "maneuver_lat": f"{maneuver_coordinate.latitude:.7f}" if maneuver_coordinate is not None else "",
      "maneuver_lon": f"{maneuver_coordinate.longitude:.7f}" if maneuver_coordinate is not None else "",
      "distance_to_maneuver_along_route": f"{distance_to_maneuver_along_geometry:.2f}",
      "distance_to_maneuver_straight": f"{distance_to_maneuver_straight:.2f}" if distance_to_maneuver_straight is not None else "",
      "command_threshold": f"{command_distance:.2f}",
      "strategy_phase": strategy_phase,
      "strategy_threshold": f"{strategy_threshold:.2f}",
      "strategy_constraint": strategy_constraint,
      "command_block_reason": command_block_reason,
      "target_lane_zone": target_lane_zone,
      "lane_belief": lane_belief,
      "lane_left_available": lane_left_available if lane_left_available is not None else "unknown",
      "lane_right_available": lane_right_available if lane_right_available is not None else "unknown",
      "route_confident": route_confident,
      "route_confidence": f"{route_confidence:.2f}",
      "route_bearing_error": f"{route_bearing_error:.2f}" if route_bearing_error is not None else "",
      "next_maneuver_direction": next_maneuver_direction,
      "next_maneuver_distance_after_current": f"{next_maneuver_distance_after_current:.2f}" if next_maneuver_distance_after_current is not None else "",
      "migration_active": self.navigation_test_exit_migration_key is not None,
      "migration_age_seconds": f"{migration_age:.2f}",
      "migration_start_distance": f"{self.navigation_test_exit_migration_start_distance:.2f}" if self.navigation_test_exit_migration_key is not None else "",
      "post_exit_recenter_active": self.navigation_test_post_exit_recenter_active(),
      "action": action,
      "direction": direction,
      "urgency": f"{urgency:.2f}",
      "cross_track_error": f"{cross_track_error:.2f}" if cross_track_error is not None else "",
    }

    try:
      write_header = not os.path.exists(self.navigation_test_debug_log_path) or os.path.getsize(self.navigation_test_debug_log_path) == 0
      if not write_header:
        try:
          with open(self.navigation_test_debug_log_path, "r", newline="") as debug_file:
            first_line = debug_file.readline().strip()
          if first_line != ",".join(NAVIGATION_TEST_DEBUG_LOG_FIELDS):
            rotated_path = self.navigation_test_debug_log_path.replace(".csv", f".v1_{int(time.time())}.csv")
            os.replace(self.navigation_test_debug_log_path, rotated_path)
            write_header = True
        except OSError:
          write_header = True

      with open(self.navigation_test_debug_log_path, "a", newline="") as debug_file:
        writer = csv.DictWriter(debug_file, fieldnames=NAVIGATION_TEST_DEBUG_LOG_FIELDS)
        if write_header:
          writer.writeheader()
        writer.writerow(row)
    except OSError:
      cloudlog.exception("navigation_test_debug.failed_to_write")

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
      self.navigation_test_reroute_counter = 0
    else:
      self.recompute_countdown = max(0, self.recompute_countdown - 1)

  def calculate_route(self, destination):
    if self.route_thread is not None and self.route_thread.is_alive():
      cloudlog.warning("Route calculation already in progress. Skipping new request.")
      return

    cloudlog.warning(f"Calculating route {self.last_position} -> {destination}")
    self.nav_destination = destination
    self.reset_navigation_test_destination_tracking(destination)
    
    if self.params.get_bool("NavigationTestControl"):
      self.update_navigation_test_command("routing")

    self.route_thread = threading.Thread(
        target=self._fetch_route_worker, 
        args=(destination, self.last_position, self.last_bearing)
    )
    self.route_thread.daemon = True
    self.route_thread.start()

  def _fetch_route_worker(self, destination, last_position, last_bearing):
    try:
      waypoints = self.params.get('NavDestinationWaypoints', encoding='utf8')
      waypoint_coords = json.loads(waypoints) if waypoints and len(waypoints) > 0 else []

      coords = [
        (last_position.longitude, last_position.latitude),
        *waypoint_coords,
        (destination.longitude, destination.latitude)
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
        'waypoints': f'0;{len(coords)-1}'
      }
      if last_bearing is not None:
        params['bearings'] = f"{(last_bearing + 360) % 360:.0f},90" + (';'*(len(coords)-1))

      resp = requests.get(url, params=params, timeout=10)
      if resp.status_code != 200:
        cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)
      resp.raise_for_status()

      r = resp.json()
      r1 = resp.json()
      
      if not r.get('routes'):
        if self.params.get_bool("NavigationTestControl"):
          self._pending_route_error = "noRoute"
        return

      chosen_route = r['routes'][0]

      def remove_keys(obj, keys_to_remove):
        if isinstance(obj, list):
          return [remove_keys(item, keys_to_remove) for item in obj]
        elif isinstance(obj, dict):
          return {key: remove_keys(value, keys_to_remove) for key, value in obj.items() if key not in keys_to_remove}
        return obj

      r2 = remove_keys(r1, ['geometry', 'annotation', 'incidents', 'intersections', 'components', 'sub', 'waypoints'])
      r3 = {}

      if 'routes' in r2 and len(r2['routes']) > 0:
        first_route = r2['routes'][0]
        try:
          nav_destination_json = self.params.get('NavDestination', encoding='utf8')
          nav_destination_data = json.loads(nav_destination_json) if nav_destination_json else {}
          route_hash = nav_destination_data.get('routeHash')

          if route_hash:
            for cand in r['routes']:
              flat = ','.join(str(coordinate) for pair in cand['geometry']['coordinates'] for coordinate in pair)
              if hashlib.sha1(flat.encode()).hexdigest() == route_hash:
                chosen_route = cand
                break

          first_route['Destination'] = nav_destination_data.get('place_name', 'Default Place Name')
          first_route['Metric'] = self.params.get_bool("IsMetric")
          r3['CurrentStep'] = 0
          r3['uuid'] = r2.get('uuid', 'osrm-navigation-test')
        except Exception as e:
          cloudlog.warning(f"Error parsing destination data in thread: {e}")

      # File writes safely moved to the background worker
      with open('navdirections.json', 'w') as json_file:
        json.dump(r2, json_file, indent=4)
      with open('CurrentStep.json', 'w') as json_file:
        json.dump(r3, json_file, indent=4)

      self._pending_route_result = (r, chosen_route, r2, r3)

    except requests.exceptions.RequestException as e:
      cloudlog.exception("failed to get route in thread")
      self._pending_route_error = e.__class__.__name__
    except Exception as e:
      cloudlog.exception("unexpected route worker failure")
      self._pending_route_error = e.__class__.__name__

  def _check_and_apply_route_thread(self):
    if self._pending_route_error:
      if self.params.get_bool("NavigationTestControl"):
        self.update_navigation_test_command("routeError", error=self._pending_route_error)
      self.clear_route()
      self.send_route()
      self._pending_route_error = None
      self._pending_route_result = None
      return

    if self._pending_route_result is None:
      return

    r, chosen_route, r2, r3 = self._pending_route_result
    self._pending_route_result = None

    self.r2 = r2
    self.r3 = r3

    if len(r.get('routes', [])):
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
          if (maxspeed_idx < len(maxspeeds)):
            maxspeed = maxspeeds[maxspeed_idx]
            if ('unknown' not in maxspeed) and ('none' not in maxspeed):
              coord.annotations['maxspeed'] = maxspeed_to_ms(maxspeed)
          coords.append(coord)
          maxspeed_idx += 1
        self.route_geometry.append(coords)
        maxspeed_idx -= 1 

      self.step_idx = 0
    else:
      cloudlog.warning("Got empty route response in applied data")
      self.clear_route()

    self.params.remove('NavDestinationWaypoints')
    self.send_route()

  def send_instruction(self):
    msg = messaging.new_message('navInstruction', valid=True)
    fp_msg = messaging.new_message('frogpilotNavigation', valid=True)

    if self.step_idx is None:
      msg.valid = False
      self.pm.send('navInstruction', msg)

      fp_msg.frogpilotNavigation.navigationSpeedLimit = 0
      self.pm.send('frogpilotNavigation', fp_msg)
      if not self.params.get_bool("NavigationTestControl"):
        self.update_navigation_test_command("none")
      return

    step = self.route[self.step_idx]
    geometry = self.route_geometry[self.step_idx]
    along_geometry = distance_along_geometry(geometry, self.last_position)
    distance_to_maneuver_along_geometry = step['distance'] - along_geometry

    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]

    msg.navInstruction.maneuverDistance = distance_to_maneuver_along_geometry
    instruction = parse_banner_instructions(banner_step['bannerInstructions'], distance_to_maneuver_along_geometry)
    if instruction is not None:
      for k,v in instruction.items():
        setattr(msg.navInstruction, k, v)

    navigation_test_action = "none"
    navigation_test_direction = "none"
    navigation_test_display_direction = "none"
    navigation_test_strategy_phase = "none"
    navigation_test_strategy_threshold = 0.0
    navigation_test_strategy_constraint = "none"
    navigation_test_target_lane_zone = "none"
    navigation_test_lane_belief = "unknown"
    navigation_test_lane_left_available = None
    navigation_test_lane_right_available = None
    navigation_test_maneuver_class = self.navigation_test_maneuver_class(instruction)
    navigation_test_road_context = self.navigation_test_road_context(navigation_test_maneuver_class)
    navigation_test_command_block_reason = "none"
    navigation_test_urgency = 0.0
    navigation_test_route_confident = False
    navigation_test_route_confidence = 0.0
    navigation_test_route_bearing_error = None
    next_maneuver_direction = "none"
    next_maneuver_distance_after_current = None
    command_distance = 0.0
    cross_track_error = None

    if self.params.get_bool("NavigationTestControl"):
      command_distance = self.navigation_test_command_distance()
      cross_track_error = self.navigation_test_cross_track_error()
      navigation_test_lane_left_available, navigation_test_lane_right_available, _ = self.navigation_test_lane_availability()
      navigation_test_lane_belief = self.navigation_test_lane_belief(navigation_test_lane_left_available, navigation_test_lane_right_available)
      navigation_test_route_confident, navigation_test_route_confidence, navigation_test_route_bearing_error, route_block_reason = self.navigation_test_route_confidence(geometry, cross_track_error)
      next_maneuver_direction, next_maneuver_distance, _ = self.navigation_test_next_maneuver(distance_to_maneuver_along_geometry)
      if next_maneuver_distance is not None:
        next_maneuver_distance_after_current = max(next_maneuver_distance - distance_to_maneuver_along_geometry, 0.0)

      if not navigation_test_route_confident:
        navigation_test_action = "routeMismatch"
        navigation_test_direction = "none"
        navigation_test_display_direction = "none"
        navigation_test_strategy_phase = "routeMismatch"
        navigation_test_strategy_constraint = "routeMismatch"
        navigation_test_command_block_reason = route_block_reason
        self.reset_navigation_test_exit_migration()
      else:
        (
          navigation_test_action,
          navigation_test_direction,
          navigation_test_display_direction,
          navigation_test_strategy_phase,
          navigation_test_strategy_threshold,
          navigation_test_strategy_constraint,
          navigation_test_target_lane_zone,
          navigation_test_lane_belief,
          navigation_test_lane_left_available,
          navigation_test_lane_right_available,
          navigation_test_maneuver_class,
          navigation_test_road_context,
          navigation_test_command_block_reason,
          navigation_test_urgency,
        ) = self.navigation_test_strategy(
          instruction,
          geometry,
          distance_to_maneuver_along_geometry,
          command_distance,
          next_maneuver_direction,
          next_maneuver_distance_after_current,
        )

      self.log_navigation_test_debug(
        instruction,
        geometry,
        distance_to_maneuver_along_geometry,
        command_distance,
        navigation_test_action,
        navigation_test_direction,
        cross_track_error,
        navigation_test_strategy_phase,
        navigation_test_strategy_threshold,
        navigation_test_strategy_constraint,
        next_maneuver_direction,
        next_maneuver_distance_after_current,
        navigation_test_maneuver_class,
        navigation_test_road_context,
        navigation_test_target_lane_zone,
        navigation_test_lane_belief,
        navigation_test_lane_left_available,
        navigation_test_lane_right_available,
        navigation_test_route_confident,
        navigation_test_route_confidence,
        navigation_test_route_bearing_error,
        navigation_test_command_block_reason,
        navigation_test_urgency,
      )

    maneuvers = []
    for i, step_i in enumerate(self.route):
      if i < self.step_idx:
        distance_to_maneuver = -sum(self.route[j]['distance'] for j in range(i+1, self.step_idx)) - along_geometry
      elif i == self.step_idx:
        distance_to_maneuver = distance_to_maneuver_along_geometry
      else:
        distance_to_maneuver = distance_to_maneuver_along_geometry + sum(self.route[j]['distance'] for j in range(self.step_idx+1, i+1))

      instruction_i = parse_banner_instructions(step_i['bannerInstructions'], distance_to_maneuver)
      if instruction_i is None:
        continue
      maneuver = {'distance': distance_to_maneuver}
      if 'maneuverType' in instruction_i:
        maneuver['type'] = instruction_i['maneuverType']
      if 'maneuverModifier' in instruction_i:
        maneuver['modifier'] = instruction_i['maneuverModifier']
      maneuvers.append(maneuver)

    msg.navInstruction.allManeuvers = maneuvers

    remaining = 1.0 - along_geometry / max(step['distance'], 1)
    total_distance = step['distance'] * remaining
    total_time = step['duration'] * remaining
    total_time_typical = total_time if step['duration_typical'] is None else step['duration_typical'] * remaining

    for i in range(self.step_idx + 1, len(self.route)):
      total_distance += self.route[i]['distance']
      total_time += self.route[i]['duration']
      total_time_typical += self.route[i]['duration'] if self.route[i]['duration_typical'] is None else self.route[i]['duration_typical']

    msg.navInstruction.distanceRemaining = total_distance
    msg.navInstruction.timeRemaining = total_time
    msg.navInstruction.timeRemainingTypical = total_time_typical

    if self.params.get_bool("NavigationTestControl"):
      self.update_navigation_test_command(
        navigation_test_action,
        navigation_test_direction if navigation_test_action != "none" else "none",
        distance_to_maneuver_along_geometry,
        total_time,
        navigation_test_display_direction if navigation_test_action != "none" else "none",
        strategy_phase=navigation_test_strategy_phase,
        strategy_constraint=navigation_test_strategy_constraint,
        target_lane_zone=navigation_test_target_lane_zone,
        lane_belief=navigation_test_lane_belief,
        maneuver_class=navigation_test_maneuver_class,
        road_context=navigation_test_road_context,
        route_confidence=navigation_test_route_confidence,
        command_block_reason=navigation_test_command_block_reason,
        urgency=navigation_test_urgency,
      )

    closest_idx, closest = min(enumerate(geometry), key=lambda p: p[1].distance_to(self.last_position))
    if closest_idx > 0:
      if along_geometry < distance_along_geometry(geometry, geometry[closest_idx]):
        closest = geometry[closest_idx - 1]

    if ('maxspeed' in closest.annotations) and self.localizer_valid:
      msg.navInstruction.speedLimit = closest.annotations['maxspeed']
      self.nav_speed_limit = closest.annotations['maxspeed']
    if not self.localizer_valid or ('maxspeed' not in closest.annotations):
      self.nav_speed_limit = 0

    if 'speedLimitSign' in step:
      if step['speedLimitSign'] == 'mutcd':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.mutcd
      elif step['speedLimitSign'] == 'vienna':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.vienna

    self.pm.send('navInstruction', msg)

    if self.should_transition_to_next_step(distance_to_maneuver_along_geometry):
      if self.step_idx + 1 < len(self.route):
        if self.navigation_test_is_exit_maneuver(instruction):
          self.start_navigation_test_post_exit_recenter(self.navigation_test_maneuver_direction(instruction))
        self.step_idx += 1
        self.reset_recompute_limits()

        if 'routes' in self.r2 and len(self.r2['routes']) > 0:
          self.r3['CurrentStep'] = self.step_idx
        # Safely offload to background thread so it doesn't stutter UI
        self._async_write_json('CurrentStep.json', self.r3)
      else:
        cloudlog.warning("Destination reached")

        dist = self.nav_destination.distance_to(self.last_position)
        if dist > REROUTE_DISTANCE:
          self.params.remove("NavDestination")
          self.clear_route()
        else:
          self.update_navigation_test_command("none")

    if self.frogpilot_toggles.conditional_navigation:
      v_ego = self.sm['carState'].vEgo
      seconds_to_stop = interp(v_ego, [0, 22.5, 45], [5, 10, 10])

      closest_condition_indices = [idx for idx in self.stop_signal if idx >= closest_idx]
      if closest_condition_indices:
        closest_condition_index = min(closest_condition_indices, key=lambda idx: abs(closest_idx - idx))
        index = self.stop_signal.index(closest_condition_index)

        distance_to_condition = self.last_position.distance_to(self.stop_coord[index])
        self.approaching_intersection = self.frogpilot_toggles.conditional_navigation_intersections and distance_to_condition < max((seconds_to_stop * v_ego), 25)
      else:
        self.approaching_intersection = False

      self.approaching_turn = self.frogpilot_toggles.conditional_navigation_turns and distance_to_maneuver_along_geometry < max((seconds_to_stop * v_ego), 25)
    else:
      self.approaching_intersection = False
      self.approaching_turn = False

    fp_msg.frogpilotNavigation.approachingIntersection = self.approaching_intersection
    fp_msg.frogpilotNavigation.approachingTurn = self.approaching_turn
    fp_msg.frogpilotNavigation.navigationSpeedLimit = self.nav_speed_limit

    self.pm.send('frogpilotNavigation', fp_msg)

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
    self.navigation_test_reroute_counter = 0
    self.navigation_test_destination_missed_counter = 0
    self.navigation_test_closest_destination_distance = None
    self.reset_navigation_test_exit_migration()
    self.reset_navigation_test_post_exit_recenter()

  def reset_recompute_limits(self):
    self.recompute_backoff = 0
    self.recompute_countdown = 0

  def reset_navigation_test_destination_tracking(self, destination=None):
    self.navigation_test_destination_missed_counter = 0
    self.navigation_test_closest_destination_distance = self.last_position.distance_to(destination) if self.last_position is not None and destination is not None else None

  def recompute_route_countdown(self):
    countdown = 2**self.recompute_backoff
    if self.params.get_bool("NavigationTestControl"):
      return max(NAVIGATION_TEST_REROUTE_COUNTDOWN_MIN, countdown)
    return countdown

  def navigation_test_missed_destination(self):
    if self.nav_destination is None or self.last_position is None:
      return False

    distance_to_destination = self.last_position.distance_to(self.nav_destination)
    if self.navigation_test_closest_destination_distance is None or distance_to_destination < self.navigation_test_closest_destination_distance:
      self.navigation_test_closest_destination_distance = distance_to_destination
      self.navigation_test_destination_missed_counter = 0
      return False

    if self.navigation_test_closest_destination_distance > NAVIGATION_TEST_DESTINATION_APPROACH_DISTANCE:
      return False

    missed_destination = (
      distance_to_destination > NAVIGATION_TEST_DESTINATION_MISSED_DISTANCE and
      distance_to_destination > self.navigation_test_closest_destination_distance + NAVIGATION_TEST_DESTINATION_MISSED_DRIFT
    )
    if missed_destination:
      self.navigation_test_destination_missed_counter += 1
    else:
      self.navigation_test_destination_missed_counter = 0

    if self.navigation_test_destination_missed_counter > NAVIGATION_TEST_DESTINATION_MISSED_COUNTER_MIN:
      cloudlog.warning(f"Navigation test missed destination: distance={distance_to_destination:.1f}m")
      return True
    return False

  def should_recompute(self):
    if self.step_idx is None or self.route is None:
      return True

    if self.params.get_bool("NavigationTestControl"):
      route_match_error = self.navigation_test_cross_track_error()
      if route_match_error is not None and route_match_error > self.navigation_test_command_distance():
        self.navigation_test_reroute_counter += 1
      else:
        self.navigation_test_reroute_counter = 0
        if route_match_error is not None and route_match_error <= NAVIGATION_TEST_MAX_COMMAND_CROSS_TRACK_ERROR:
          self.recompute_backoff = 0

      if self.navigation_test_reroute_counter > NAVIGATION_TEST_REROUTE_COUNTER_MIN:
        cloudlog.warning(f"Navigation test route mismatch: cross_track={route_match_error:.1f}m")
        return True

    if self.step_idx == len(self.route) - 1:
      if self.params.get_bool("NavigationTestControl") and self.navigation_test_missed_destination():
        return True
      return False

    min_d = self.path_minimum_distance(self.route_geometry[self.step_idx])

    if min_d is not None and min_d > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
    return self.reroute_counter > REROUTE_COUNTER_MIN

def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute', 'frogpilotNavigation'])
  sm = messaging.SubMaster(['carState', 'liveLocationKalman', 'managerState', 'frogpilotPlan', 'modelV2'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()

if __name__ == "__main__":
  main()
