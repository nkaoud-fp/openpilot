#!/usr/bin/env python3
import csv
from collections import Counter, deque
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
NAVIGATION_TEST_SHARED_DESTINATION_URL = os.environ.get("NAVIGATION_TEST_SHARED_DESTINATION_URL", "https://frogpilot-navigation.vercel.app/latest")
NAVIGATION_TEST_SHARED_DESTINATION_API_KEY = os.environ.get("NAVIGATION_TEST_SHARED_DESTINATION_API_KEY", "")
NAVIGATION_TEST_SHARED_DESTINATION_RETRY_SECONDS = 15.0
NAVIGATION_TEST_COMMAND_DISTANCE = 35
NAVIGATION_TEST_COMMAND_SECONDS = 8
NAVIGATION_TEST_EXIT_PREP_SECONDS = 30
NAVIGATION_TEST_EXIT_PREP_DISTANCE_MIN = 250
NAVIGATION_TEST_EXIT_PREP_MAX_LANE_CHANGES = 4
NAVIGATION_TEST_EXIT_PREP_LANE_CHANGE_SECONDS = 10
NAVIGATION_TEST_EXIT_PREP_LANE_CHANGE_COOLDOWN = 3
NAVIGATION_TEST_HIGHWAY_EXIT_PREP_SPEED = 22.0
NAVIGATION_TEST_HIGHWAY_EXIT_PREP_SECONDS = 180
NAVIGATION_TEST_HIGHWAY_EXIT_PREP_DISTANCE_MIN = 1500
NAVIGATION_TEST_HIGHWAY_EXIT_PREP_DISTANCE_MAX = 5000
NAVIGATION_TEST_CONSECUTIVE_CONFLICT_DISTANCE = 400
NAVIGATION_TEST_MAX_COMMAND_CROSS_TRACK_ERROR = 35
NAVIGATION_TEST_REROUTE_COUNTER_MIN = 2
NAVIGATION_TEST_REROUTE_COUNTDOWN_MIN = 5
NAVIGATION_TEST_DESTINATION_APPROACH_DISTANCE = 50
NAVIGATION_TEST_DESTINATION_MISSED_DISTANCE = 80
NAVIGATION_TEST_DESTINATION_MISSED_DRIFT = 30
NAVIGATION_TEST_DESTINATION_MISSED_COUNTER_MIN = 2
NAVIGATION_TEST_POST_EXIT_RECOVERY_MIN_SPEED = 13.0
NAVIGATION_TEST_POST_EXIT_RECOVERY_MIN_DISTANCE = 100.0
NAVIGATION_TEST_POST_EXIT_RECOVERY_MAX_DISTANCE = 600.0
NAVIGATION_TEST_POST_EXIT_RECOVERY_NEXT_SAME_DIRECTION_HOLD_DISTANCE = 500.0
NAVIGATION_TEST_POST_EXIT_RECOVERY_COMMAND_SECONDS = 10.0
NAVIGATION_TEST_TURN_COMMAND_DISTANCE_MIN = 35.0
NAVIGATION_TEST_TURN_COMMAND_SECONDS = 8.0
NAVIGATION_TEST_SURFACE_TURN_PREP_SECONDS = 18.0
NAVIGATION_TEST_SURFACE_TURN_PREP_DISTANCE_MIN = 120.0
NAVIGATION_TEST_SURFACE_TURN_PREP_DISTANCE_MAX = 650.0
NAVIGATION_TEST_LATE_LANE_CHANGE_LOCKOUT_SECONDS = 3.0
NAVIGATION_TEST_LATE_LANE_CHANGE_LOCKOUT_DISTANCE_MIN = 35.0
NAVIGATION_TEST_LANE_SAMPLE_SECONDS = 0.5
NAVIGATION_TEST_LANE_SAMPLE_RETENTION_SECONDS = 2.0
NAVIGATION_TEST_DEBUG_LOG_DIR = "/data/media/0/navigation_test_logs"
NAVIGATION_TEST_DEBUG_LOG_INTERVAL = 0.5
TURN_SLOWDOWN_MIN_SPEED_MS = 25.0 / 3.6

NAV_HIGHWAY_SPEED_MIN_MS = 18.0
NAV_ANGLE_CONTINUE_MAX_DEG = 5.0
NAV_ANGLE_FORK_MIN_DEG = 5.0
NAV_ANGLE_HIGHWAY_EXIT_MIN_DEG = 15.0
NAV_ANGLE_HIGHWAY_EXIT_MAX_DEG = 45.0
NAV_ANGLE_NORMAL_TURN_MIN_DEG = 15.0
NAV_ANGLE_UTURN_MIN_DEG = 135.0
NAV_ANGLE_ANCHOR_M = 8.0
NAVIGATION_TEST_DEBUG_LOG_FIELDS = [
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
  "maneuver_angle_deg",
  "maneuver_lat",
  "maneuver_lon",
  "distance_to_maneuver_along_route",
  "distance_to_maneuver_straight",
  "command_threshold",
  "strategy_phase",
  "strategy_threshold",
  "strategy_constraint",
  "lane_width_left",
  "lane_width_right",
  "current_lane",
  "total_lanes",
  "left_lane_available",
  "right_lane_available",
  "lane_belief",
  "target_lane_zone",
  "target_edge_reached",
  "next_maneuver_direction",
  "next_maneuver_distance_after_current",
  "migration_active",
  "migration_age_seconds",
  "migration_start_distance",
  "action",
  "direction",
  "command_actionable",
  "command_direction",
  "display_direction",
  "command_speed_active",
  "target_speed",
  "target_speed_source",
  "command_max_lane_changes",
  "prep_stage",
  "prep_reason",
  "prep_completed_lane_changes",
  "prep_max_lane_changes",
  "prep_cooldown_remaining",
  "prep_allowed",
  "prep_lane_available",
  "prep_lane_changes_enabled",
  "prep_below_lane_change_speed",
  "prep_blindspot_detected",
  "prep_adjacent_lead_status",
  "prep_adjacent_lead_distance",
  "prep_adjacent_lead_closing_speed",
  "prep_required_gap",
  "post_exit_recovery_active",
  "post_exit_recovery_exit_direction",
  "post_exit_recovery_direction",
  "post_exit_recovery_distance",
  "post_exit_recovery_done",
  "current_step_error",
  "global_route_error",
  "cross_track_error",
  "recompute_reason",
  "route_generation",
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
    self.navigation_test_debug_log_override_path = os.environ.get("NAVIGATION_TEST_DEBUG_LOG_PATH", "")
    self.navigation_test_debug_log_dir = os.environ.get("NAVIGATION_TEST_DEBUG_LOG_DIR", NAVIGATION_TEST_DEBUG_LOG_DIR)
    self.navigation_test_recompute_reason = "none"
    self.navigation_test_route_generation = 0
    self.navigation_test_post_exit_recovery_key = None
    self.navigation_test_post_exit_recovery_exit_direction = "none"
    self.navigation_test_post_exit_recovery_direction = "none"
    self.navigation_test_post_exit_recovery_exit_coordinate = None
    self.navigation_test_post_exit_recovery_started_at = 0.0
    self.navigation_test_post_exit_recovery_command_started_at = 0.0
    self.navigation_test_post_exit_recovery_done = True
    self.navigation_test_lane_samples = deque()

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
    self.update_navigation_test_lane_samples()

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
    except Exception:
      if self.params.get_bool("NavigationTestControl"):
        self.update_navigation_test_command("routeError")
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
      self.navigation_test_last_handled_share_selection_token = share_selection_token or ""
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
      headers = {}
      if NAVIGATION_TEST_SHARED_DESTINATION_API_KEY:
        headers["X-FrogPilot-Key"] = NAVIGATION_TEST_SHARED_DESTINATION_API_KEY
      response = requests.get(NAVIGATION_TEST_SHARED_DESTINATION_URL, timeout=5, headers=headers)
      response.raise_for_status()
      content_type = response.headers.get("content-type", "")
      if "application/json" in content_type:
        payload = response.json()
      else:
        payload = response.text.strip()
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
    if isinstance(record, str):
      coordinates = [value.strip() for value in record.split(",", 1)]
      if len(coordinates) != 2:
        cloudlog.warning(f"Navigation test shared destination has invalid plain payload: {payload}")
        self.navigation_test_shared_destination_retry_at = now + NAVIGATION_TEST_SHARED_DESTINATION_RETRY_SECONDS
        self.update_navigation_test_command("routeError", error="sharedInvalidPayload")
        return None
      record = {"latitude": coordinates[0], "longitude": coordinates[1]}

    if not isinstance(record, dict):
      cloudlog.warning(f"Navigation test shared destination has invalid payload: {payload}")
      self.navigation_test_shared_destination_retry_at = now + NAVIGATION_TEST_SHARED_DESTINATION_RETRY_SECONDS
      self.update_navigation_test_command("routeError", error="sharedInvalidPayload")
      return None

    try:
      latitude = float(record.get("latitude", record.get("lat")))
      longitude = float(record.get("longitude", record.get("lng", record.get("lon"))))
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

  def update_navigation_test_command(self, action, direction="none", distance=0.0, eta_seconds=0.0, display_direction=None, error="", strategy_phase="none", strategy_constraint="none", target_speed=0.0, target_speed_source="none", max_lane_changes=None, lane_change_cooldown=None):
    migration_age = time.monotonic() - self.navigation_test_exit_migration_started_at if self.navigation_test_exit_migration_key is not None else 0.0
    max_lane_changes = NAVIGATION_TEST_EXIT_PREP_MAX_LANE_CHANGES if max_lane_changes is None else max_lane_changes
    lane_change_cooldown = NAVIGATION_TEST_EXIT_PREP_LANE_CHANGE_COOLDOWN if lane_change_cooldown is None else lane_change_cooldown
    command = json.dumps({
      "action": action,
      "direction": direction,
      "displayDirection": display_direction or direction,
      "distance": max(distance, 0.0),
      "etaSeconds": max(eta_seconds, 0.0),
      "error": error,
      "strategyPhase": strategy_phase,
      "strategyConstraint": strategy_constraint,
      "migrationActive": self.navigation_test_exit_migration_key is not None,
      "migrationAgeSeconds": max(migration_age, 0.0),
      "migrationStartDistance": max(self.navigation_test_exit_migration_start_distance, 0.0),
      "maxLaneChanges": max_lane_changes,
      "laneChangeCooldown": lane_change_cooldown,
      "targetSpeed": max(target_speed, 0.0),
      "targetSpeedSource": target_speed_source,
    })
    if command != self.navigation_test_command:
      self.params.put("NavigationTestTurnCommand", command)
      self.navigation_test_command = command

  def navigation_test_angle_diff(self, a, b):
    return (a - b + 180.0) % 360.0 - 180.0

  def navigation_test_bearing_between(self, a, b):
    lat1 = math.radians(a.latitude)
    lat2 = math.radians(b.latitude)
    d_lon = math.radians(b.longitude - a.longitude)
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

  def navigation_test_compute_maneuver_angle(self, geometry, next_geometry=None):
    if geometry is None or len(geometry) < 2:
      return 0.0, False

    maneuver_point = geometry[-1]
    pre_anchor = geometry[0]
    accumulated = 0.0
    for i in range(len(geometry) - 1, 0, -1):
      accumulated += geometry[i].distance_to(geometry[i - 1])
      pre_anchor = geometry[i - 1]
      if accumulated >= NAV_ANGLE_ANCHOR_M:
        break
    if pre_anchor.distance_to(maneuver_point) < 1e-3:
      return 0.0, False
    bearing_in = self.navigation_test_bearing_between(pre_anchor, maneuver_point)

    if next_geometry is None and self.route_geometry is not None and self.step_idx is not None and self.step_idx + 1 < len(self.route_geometry):
      next_geometry = self.route_geometry[self.step_idx + 1]

    post_anchor = None
    if next_geometry and len(next_geometry) >= 2:
      accumulated = 0.0
      for i in range(len(next_geometry) - 1):
        accumulated += next_geometry[i].distance_to(next_geometry[i + 1])
        post_anchor = next_geometry[i + 1]
        if accumulated >= NAV_ANGLE_ANCHOR_M:
          break

    if post_anchor is None or post_anchor.distance_to(maneuver_point) < 1e-3:
      if len(geometry) < 3:
        return 0.0, False
      bearing_in = self.navigation_test_bearing_between(geometry[-3], geometry[-2])
      bearing_out = self.navigation_test_bearing_between(geometry[-2], geometry[-1])
    else:
      bearing_out = self.navigation_test_bearing_between(maneuver_point, post_anchor)

    return self.navigation_test_angle_diff(bearing_out, bearing_in), True

  def navigation_test_maneuver_direction(self, instruction, geometry=None, next_geometry=None):
    angle, valid = self.navigation_test_compute_maneuver_angle(geometry, next_geometry)
    if valid:
      if angle > NAV_ANGLE_CONTINUE_MAX_DEG:
        return "right"
      if angle < -NAV_ANGLE_CONTINUE_MAX_DEG:
        return "left"
      return "none"

    if instruction is None:
      return "none"

    text = (instruction.get("maneuverType", "") + " " + instruction.get("maneuverModifier", "")).lower()
    if "left" in text:
      return "left"
    if "right" in text:
      return "right"
    return "none"

  def navigation_test_maneuver_display_direction(self, instruction, geometry=None, next_geometry=None):
    angle, valid = self.navigation_test_compute_maneuver_angle(geometry, next_geometry)

    text_modifier = "" if instruction is None else instruction.get("maneuverModifier", "").lower().replace(" ", "_")
    text_type = "" if instruction is None else instruction.get("maneuverType", "").lower().replace(" ", "_")
    if "uturn" in (text_modifier, text_type):
      return "uturn"

    if not valid:
      return self.navigation_test_maneuver_direction(instruction, geometry, next_geometry)

    abs_angle = abs(angle)
    if abs_angle >= NAV_ANGLE_UTURN_MIN_DEG:
      return "uturn"
    side = "right" if angle > 0 else "left"
    if abs_angle >= NAV_ANGLE_NORMAL_TURN_MIN_DEG:
      return side
    if abs_angle >= NAV_ANGLE_FORK_MIN_DEG:
      return f"slight_{side}"
    return "none"

  def navigation_test_maneuver_class(self, instruction, geometry=None, next_geometry=None):
    if instruction is None and (geometry is None or len(geometry) < 2):
      return "none"

    text_type = "" if instruction is None else instruction.get("maneuverType", "").lower()
    text_primary = "" if instruction is None else instruction.get("maneuverPrimaryText", "").lower()
    text = f"{text_type} {text_primary}"

    if "arrive" in text or "destination" in text:
      return "arrive"
    if "roundabout" in text_type or "rotary" in text_type:
      return "roundabout"
    if "merge" in text_type or "on ramp" in text or "merge" in text_primary:
      return "highway_merge"

    angle, valid = self.navigation_test_compute_maneuver_angle(geometry, next_geometry)
    if not valid:
      if "fork" in text_type:
        return "highway_fork"
      if "ramp" in text_type or "exit" in text_primary:
        return "highway_exit"
      if "turn" in text_type or "end of road" in text_type:
        return "normal_turn"
      return "unknown"

    abs_angle = abs(angle)
    if abs_angle >= NAV_ANGLE_UTURN_MIN_DEG:
      return "uturn"
    if abs_angle < NAV_ANGLE_CONTINUE_MAX_DEG:
      return "continue"

    v_ego = float(self.sm['carState'].vEgo)
    if v_ego >= NAV_HIGHWAY_SPEED_MIN_MS:
      if abs_angle >= NAV_ANGLE_HIGHWAY_EXIT_MAX_DEG:
        return "normal_turn"
      if abs_angle >= NAV_ANGLE_HIGHWAY_EXIT_MIN_DEG:
        return "highway_exit"
      if abs_angle >= NAV_ANGLE_FORK_MIN_DEG:
        return "highway_fork"
      return "highway_merge"

    if abs_angle >= NAV_ANGLE_NORMAL_TURN_MIN_DEG:
      return "normal_turn"
    if abs_angle >= NAV_ANGLE_FORK_MIN_DEG:
      return "highway_fork"
    return "continue"

  def navigation_test_is_lane_positioning_maneuver(self, maneuver_class):
    return maneuver_class in ("highway_exit", "highway_fork", "highway_merge", "normal_turn", "uturn")

  def navigation_test_is_control_turn(self, instruction, geometry=None, next_geometry=None):
    if instruction is None:
      return False

    return self.navigation_test_maneuver_class(instruction, geometry, next_geometry) in ("normal_turn", "uturn")

  def navigation_test_is_exit_maneuver(self, instruction, geometry=None, next_geometry=None):
    return self.navigation_test_maneuver_class(instruction, geometry, next_geometry) == "highway_exit"

  def navigation_test_is_merge_maneuver(self, instruction, geometry=None, next_geometry=None):
    return self.navigation_test_maneuver_class(instruction, geometry, next_geometry) == "highway_merge"

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

  def reset_navigation_test_post_exit_recovery(self):
    self.navigation_test_post_exit_recovery_key = None
    self.navigation_test_post_exit_recovery_exit_direction = "none"
    self.navigation_test_post_exit_recovery_direction = "none"
    self.navigation_test_post_exit_recovery_exit_coordinate = None
    self.navigation_test_post_exit_recovery_started_at = 0.0
    self.navigation_test_post_exit_recovery_command_started_at = 0.0
    self.navigation_test_post_exit_recovery_done = True

  def start_navigation_test_post_exit_recovery(self, instruction, geometry, exit_direction):
    if exit_direction not in ("left", "right"):
      self.reset_navigation_test_post_exit_recovery()
      return

    # After a right-side exit, move one lane left away from the exit/on-ramp lane.
    # Mirror the behavior for left-side exits.
    recovery_direction = "left" if exit_direction == "right" else "right"
    exit_coordinate = geometry[-1] if geometry else self.last_position
    self.navigation_test_post_exit_recovery_key = self.navigation_test_maneuver_key(instruction, geometry)
    self.navigation_test_post_exit_recovery_exit_direction = exit_direction
    self.navigation_test_post_exit_recovery_direction = recovery_direction
    self.navigation_test_post_exit_recovery_exit_coordinate = exit_coordinate
    self.navigation_test_post_exit_recovery_started_at = time.monotonic()
    self.navigation_test_post_exit_recovery_command_started_at = 0.0
    self.navigation_test_post_exit_recovery_done = False

  def navigation_test_post_exit_recovery_distance(self):
    if self.navigation_test_post_exit_recovery_exit_coordinate is None or self.last_position is None:
      return None
    return self.last_position.distance_to(self.navigation_test_post_exit_recovery_exit_coordinate)

  def navigation_test_same_direction_maneuver_soon(self, direction, instruction, distance_to_maneuver_along_geometry, next_maneuver_direction, next_maneuver_distance_after_current):
    if direction not in ("left", "right"):
      return False

    current_direction = self.navigation_test_maneuver_direction(instruction)
    current_soon = (
      current_direction == direction and
      0.0 < distance_to_maneuver_along_geometry <= NAVIGATION_TEST_POST_EXIT_RECOVERY_NEXT_SAME_DIRECTION_HOLD_DISTANCE
    )
    next_soon = (
      next_maneuver_direction == direction and
      next_maneuver_distance_after_current is not None and
      0.0 <= next_maneuver_distance_after_current <= NAVIGATION_TEST_POST_EXIT_RECOVERY_NEXT_SAME_DIRECTION_HOLD_DISTANCE
    )
    return current_soon or next_soon

  def navigation_test_post_exit_recovery_strategy(self, instruction, distance_to_maneuver_along_geometry, next_maneuver_direction="none", next_maneuver_distance_after_current=None):
    if self.navigation_test_post_exit_recovery_done or self.navigation_test_post_exit_recovery_key is None:
      return None

    exit_direction = self.navigation_test_post_exit_recovery_exit_direction
    recovery_direction = self.navigation_test_post_exit_recovery_direction
    if exit_direction not in ("left", "right") or recovery_direction not in ("left", "right"):
      self.reset_navigation_test_post_exit_recovery()
      return None

    distance_since_exit = self.navigation_test_post_exit_recovery_distance()
    if distance_since_exit is None:
      return None

    if distance_since_exit > NAVIGATION_TEST_POST_EXIT_RECOVERY_MAX_DISTANCE:
      self.navigation_test_post_exit_recovery_done = True
      return None

    # If the next maneuver needs the same side as the exit very soon, stay in the exit-side lane.
    if self.navigation_test_same_direction_maneuver_soon(
      exit_direction,
      instruction,
      distance_to_maneuver_along_geometry,
      next_maneuver_direction,
      next_maneuver_distance_after_current,
    ):
      self.navigation_test_post_exit_recovery_done = True
      return None

    v_ego = self.sm['carState'].vEgo
    if v_ego < NAVIGATION_TEST_POST_EXIT_RECOVERY_MIN_SPEED:
      return None

    if distance_since_exit < NAVIGATION_TEST_POST_EXIT_RECOVERY_MIN_DISTANCE:
      return None

    now = time.monotonic()
    if self.navigation_test_post_exit_recovery_command_started_at == 0.0:
      self.navigation_test_post_exit_recovery_command_started_at = now

    if now - self.navigation_test_post_exit_recovery_command_started_at > NAVIGATION_TEST_POST_EXIT_RECOVERY_COMMAND_SECONDS:
      self.navigation_test_post_exit_recovery_done = True
      return None

    constraint = "recoverFromRightExitLane" if exit_direction == "right" else "recoverFromLeftExitLane"
    return (
      "laneChange",
      recovery_direction,
      recovery_direction,
      "postExitLaneRecovery",
      NAVIGATION_TEST_POST_EXIT_RECOVERY_MAX_DISTANCE,
      constraint,
    )

  def navigation_test_command_distance(self):
    v_ego = self.sm['carState'].vEgo
    return max(NAVIGATION_TEST_COMMAND_DISTANCE, v_ego * NAVIGATION_TEST_COMMAND_SECONDS)

  def navigation_test_turn_command_distance(self):
    v_ego = self.sm['carState'].vEgo
    configured_min_distance = max(
      0.0,
      float(getattr(self.frogpilot_toggles, "navigation_test_turn_command_distance_min", NAVIGATION_TEST_TURN_COMMAND_DISTANCE_MIN)),
    )
    configured_seconds = max(
      0.0,
      float(getattr(self.frogpilot_toggles, "navigation_test_turn_command_seconds", NAVIGATION_TEST_TURN_COMMAND_SECONDS)),
    )
    return max(configured_min_distance, v_ego * configured_seconds)

  def navigation_test_exit_prep_distance(self):
    v_ego = self.sm['carState'].vEgo
    lane_prep_seconds = NAVIGATION_TEST_EXIT_PREP_MAX_LANE_CHANGES * (
      NAVIGATION_TEST_EXIT_PREP_LANE_CHANGE_SECONDS + NAVIGATION_TEST_EXIT_PREP_LANE_CHANGE_COOLDOWN
    )
    return max(NAVIGATION_TEST_EXIT_PREP_DISTANCE_MIN, v_ego * max(NAVIGATION_TEST_EXIT_PREP_SECONDS, lane_prep_seconds))

  def navigation_test_highway_exit_prep_distance(self):
    v_ego = max(self.sm['carState'].vEgo, 0.1)
    if v_ego < NAVIGATION_TEST_HIGHWAY_EXIT_PREP_SPEED:
      return self.navigation_test_exit_prep_distance()

    seconds_per_lane = 12.0
    lane_sweep_distance = v_ego * seconds_per_lane

    target_exit_speed = 15.0
    comfortable_decel = 1.2
    decel_distance = (v_ego**2 - target_exit_speed**2) / (2 * comfortable_decel) if v_ego > target_exit_speed else 0.0
    total_prep_distance = lane_sweep_distance + decel_distance

    configured_min_distance = max(
      0.0,
      float(getattr(self.frogpilot_toggles, "navigation_test_highway_prep_distance_min", NAVIGATION_TEST_HIGHWAY_EXIT_PREP_DISTANCE_MIN)),
    )
    configured_max_distance = max(
      configured_min_distance,
      float(getattr(self.frogpilot_toggles, "navigation_test_highway_prep_distance_max", NAVIGATION_TEST_HIGHWAY_EXIT_PREP_DISTANCE_MAX)),
    )

    return min(
      configured_max_distance,
      max(configured_min_distance, total_prep_distance),
    )

  def navigation_test_surface_turn_prep_distance(self):
    v_ego = self.sm['carState'].vEgo
    configured_min_distance = max(
      0.0,
      float(getattr(self.frogpilot_toggles, "navigation_test_turn_prep_distance_min", NAVIGATION_TEST_SURFACE_TURN_PREP_DISTANCE_MIN)),
    )
    configured_max_distance = max(
      configured_min_distance,
      float(getattr(self.frogpilot_toggles, "navigation_test_turn_prep_distance_max", NAVIGATION_TEST_SURFACE_TURN_PREP_DISTANCE_MAX)),
    )
    return min(
      configured_max_distance,
      max(configured_min_distance, v_ego * NAVIGATION_TEST_SURFACE_TURN_PREP_SECONDS),
    )

  def navigation_test_prep_distance_for_maneuver(self, maneuver_class):
    if maneuver_class in ("normal_turn", "uturn"):
      return self.navigation_test_surface_turn_prep_distance()
    if maneuver_class in ("highway_exit", "highway_fork", "highway_merge"):
      return self.navigation_test_highway_exit_prep_distance()
    return self.navigation_test_exit_prep_distance()

  def navigation_test_lanes_to_target_edge(self, direction):
    current_lane, total_lanes = self.navigation_test_common_lane_position()
    if current_lane is None or total_lanes is None:
      return None
    if current_lane <= 0 or total_lanes <= 0 or current_lane > total_lanes:
      return None
    if direction == "left":
      return current_lane - 1
    if direction == "right":
      return total_lanes - current_lane
    return None

  def navigation_test_lane_positioning_prep_distance(self, maneuver_class, direction):
    per_lane_distance = self.navigation_test_prep_distance_for_maneuver(maneuver_class)
    lanes_to_target = self.navigation_test_lanes_to_target_edge(direction)
    if lanes_to_target is None:
      return per_lane_distance
    return per_lane_distance * max(1, min(NAVIGATION_TEST_EXIT_PREP_MAX_LANE_CHANGES, lanes_to_target))

  def navigation_test_max_lane_changes_for_direction(self, direction):
    lanes_to_target = self.navigation_test_lanes_to_target_edge(direction)
    if lanes_to_target is None:
      return NAVIGATION_TEST_EXIT_PREP_MAX_LANE_CHANGES
    return max(1, min(NAVIGATION_TEST_EXIT_PREP_MAX_LANE_CHANGES, lanes_to_target))

  def navigation_test_late_lane_change_lockout_distance(self):
    v_ego = self.sm['carState'].vEgo
    configured_min_distance = max(
      0.0,
      float(getattr(self.frogpilot_toggles, "navigation_test_turn_lockout_distance_min", NAVIGATION_TEST_LATE_LANE_CHANGE_LOCKOUT_DISTANCE_MIN)),
    )
    return max(configured_min_distance, v_ego * NAVIGATION_TEST_LATE_LANE_CHANGE_LOCKOUT_SECONDS)

  def navigation_test_late_lockout_distance_for_maneuver(self, maneuver_class):
    if maneuver_class in ("normal_turn", "uturn"):
      return self.navigation_test_late_lane_change_lockout_distance()
    return self.navigation_test_command_distance()

  def update_navigation_test_lane_samples(self):
    try:
      current_lane = int(self.sm['frogpilotPlan'].currentLane)
      total_lanes = int(self.sm['frogpilotPlan'].totalLanes)
    except Exception:
      current_lane = 0
      total_lanes = 0

    now = time.monotonic()
    if current_lane > 0 and total_lanes > 0 and current_lane <= total_lanes:
      self.navigation_test_lane_samples.append((now, current_lane, total_lanes))

    while self.navigation_test_lane_samples and now - self.navigation_test_lane_samples[0][0] > NAVIGATION_TEST_LANE_SAMPLE_RETENTION_SECONDS:
      self.navigation_test_lane_samples.popleft()

  def navigation_test_common_lane_position(self):
    now = time.monotonic()
    samples = [
      (sample_time, current_lane, total_lanes)
      for sample_time, current_lane, total_lanes in self.navigation_test_lane_samples
      if now - sample_time <= NAVIGATION_TEST_LANE_SAMPLE_RETENTION_SECONDS
    ]
    if len(samples) < 2 or samples[-1][0] - samples[0][0] < NAVIGATION_TEST_LANE_SAMPLE_SECONDS:
      return None, None

    counts = Counter((current_lane, total_lanes) for _, current_lane, total_lanes in samples)
    max_count = max(counts.values())
    for _, current_lane, total_lanes in reversed(samples):
      if counts[(current_lane, total_lanes)] == max_count:
        return current_lane, total_lanes

    return None, None

  def navigation_test_lane_availability(self):
    current_lane, total_lanes = self.navigation_test_common_lane_position()
    if current_lane is None or total_lanes is None:
      return None, None

    if current_lane <= 0 or total_lanes <= 0 or current_lane > total_lanes:
      return None, None

    left_available = current_lane > 1
    right_available = current_lane < total_lanes
    return left_available, right_available

  def navigation_test_lane_belief(self, left_available=None, right_available=None):
    if left_available is True and right_available is True:
      return "interior"
    if left_available is True and right_available is False:
      return "right_edge"
    if left_available is False and right_available is True:
      return "left_edge"
    if left_available is False and right_available is False:
      return "both_edges"
    return "unknown"

  def navigation_test_target_zone_for_direction(self, direction):
    if direction == "left":
      return "left_edge"
    if direction == "right":
      return "right_edge"
    return "none"

  def navigation_test_target_edge_reached(self, target_lane_zone, lane_belief):
    return target_lane_zone != "none" and lane_belief in (target_lane_zone, "both_edges")

  def navigation_test_next_maneuver(self, distance_to_maneuver_along_geometry):
    if self.route is None or self.step_idx is None:
      return "none", None, None

    cumulative_distance = distance_to_maneuver_along_geometry
    for i in range(self.step_idx + 1, len(self.route)):
      cumulative_distance += self.route[i]['distance']
      instruction = parse_banner_instructions(self.route[i]['bannerInstructions'], cumulative_distance)
      if instruction is None:
        continue

      step_geometry = self.route_geometry[i] if (self.route_geometry is not None and i < len(self.route_geometry)) else None
      step_next_geometry = self.route_geometry[i + 1] if (self.route_geometry is not None and i + 1 < len(self.route_geometry)) else None
      direction = self.navigation_test_maneuver_direction(instruction, step_geometry, step_next_geometry)
      display_direction = self.navigation_test_maneuver_display_direction(instruction, step_geometry, step_next_geometry)
      if direction != "none" or display_direction == "uturn":
        return direction, cumulative_distance, instruction

    return "none", None, None

  def navigation_test_strategy(self, instruction, geometry, distance_to_maneuver_along_geometry, command_distance, next_maneuver_direction="none", next_maneuver_distance_after_current=None):
    next_geometry = self.route_geometry[self.step_idx + 1] if (self.route_geometry is not None and self.step_idx is not None and self.step_idx + 1 < len(self.route_geometry)) else None
    direction = self.navigation_test_maneuver_direction(instruction, geometry, next_geometry)
    display_direction = self.navigation_test_maneuver_display_direction(instruction, geometry, next_geometry)
    maneuver_class = self.navigation_test_maneuver_class(instruction, geometry, next_geometry)
    strategy_phase = "none"
    strategy_threshold = 0.0
    strategy_constraint = "none"
    action = "none"
    left_available, right_available = self.navigation_test_lane_availability()
    lane_belief = self.navigation_test_lane_belief(left_available, right_available)
    target_lane_zone = self.navigation_test_target_zone_for_direction(direction)

    if direction == "none" and display_direction != "uturn":
      self.reset_navigation_test_exit_migration()
      return action, direction, display_direction, strategy_phase, strategy_threshold, strategy_constraint

    turn_command_distance = self.navigation_test_turn_command_distance() if maneuver_class in ("normal_turn", "uturn") else command_distance

    if distance_to_maneuver_along_geometry <= turn_command_distance and self.navigation_test_is_control_turn(instruction, geometry, next_geometry):
      self.reset_navigation_test_exit_migration()
      return "turn", direction, display_direction, "turn", turn_command_distance, strategy_constraint

    if distance_to_maneuver_along_geometry <= turn_command_distance and not self.navigation_test_is_lane_positioning_maneuver(maneuver_class):
      self.reset_navigation_test_exit_migration()
      return "upcoming", "none", display_direction, "upcoming", turn_command_distance, "displayOnly"

    if not self.navigation_test_is_lane_positioning_maneuver(maneuver_class):
      self.reset_navigation_test_exit_migration()
      return "upcoming", direction, display_direction, "upcoming", command_distance, strategy_constraint

    active_prep_distance = self.navigation_test_lane_positioning_prep_distance(maneuver_class, direction)
    standard_exit_prep_distance = self.navigation_test_exit_prep_distance()
    late_lockout_distance = self.navigation_test_late_lockout_distance_for_maneuver(maneuver_class)
    conflict_soon = (
      next_maneuver_direction in ("left", "right") and
      next_maneuver_direction != direction and
      next_maneuver_distance_after_current is not None and
      0.0 < next_maneuver_distance_after_current <= NAVIGATION_TEST_CONSECUTIVE_CONFLICT_DISTANCE
    )

    if distance_to_maneuver_along_geometry > active_prep_distance:
      self.reset_navigation_test_exit_migration()
      return "upcoming", direction, display_direction, "upcoming", active_prep_distance, strategy_constraint

    self.update_navigation_test_exit_migration(instruction, geometry, direction, distance_to_maneuver_along_geometry)

    if self.navigation_test_target_edge_reached(target_lane_zone, lane_belief):
      return "upcoming", direction, display_direction, "targetEdgeHold", active_prep_distance, "targetEdgeReached"

    if conflict_soon and maneuver_class in ("highway_exit", "highway_fork", "highway_merge") and distance_to_maneuver_along_geometry > standard_exit_prep_distance:
      self.reset_navigation_test_exit_migration()
      return "upcoming", direction, display_direction, "consecutiveConflictHold", active_prep_distance, "conflictingNextManeuver"

    if maneuver_class in ("normal_turn", "uturn") and distance_to_maneuver_along_geometry <= late_lockout_distance:
      return "upcoming", direction, display_direction, "maneuverLockout", late_lockout_distance, "lateLaneChangeLockout"

    if maneuver_class in ("normal_turn", "uturn"):
      return "laneChange", direction, display_direction, "turnLanePositioning", active_prep_distance, strategy_constraint

    if maneuver_class == "highway_fork":
      return "laneChange", direction, display_direction, "forkLanePositioning", active_prep_distance, strategy_constraint

    if maneuver_class == "highway_merge":
      return "laneChange", direction, display_direction, "mergeLanePositioning", active_prep_distance, strategy_constraint

    if maneuver_class == "highway_exit":
      phase = "exitMigration" if distance_to_maneuver_along_geometry <= standard_exit_prep_distance else "highwayExitMigration"
      return "laneChange", direction, display_direction, phase, active_prep_distance, strategy_constraint

    self.reset_navigation_test_exit_migration()
    return "upcoming", direction, display_direction, "upcoming", command_distance, strategy_constraint

  def navigation_test_maneuver_target_speed(self, instruction, current_geometry, maneuver_class=None):
    if instruction is None:
      return 0.0, "none"

    if maneuver_class is None:
      next_geometry = self.route_geometry[self.step_idx + 1] if (self.route_geometry is not None and self.step_idx is not None and self.step_idx + 1 < len(self.route_geometry)) else None
      maneuver_class = self.navigation_test_maneuver_class(instruction, current_geometry, next_geometry)

    if maneuver_class in ("normal_turn", "uturn"):
      configured_turn_speed = max(0.0, float(getattr(self.frogpilot_toggles, "navigation_test_turn_slowdown_speed", TURN_SLOWDOWN_MIN_SPEED_MS)))
      return configured_turn_speed, "configTurnSlowdown"

    if maneuver_class == "highway_exit":
      return 22.22, "hardcodedExit80kph"

    # Pull nearby route-annotation speeds around the maneuver: last points of current step plus first points of next step.
    speed_candidates = []
    if current_geometry is not None:
      for coordinate in current_geometry[-12:]:
        speed = coordinate.annotations.get('maxspeed', 0.0)
        if speed and speed > 0.0:
          speed_candidates.append(float(speed))

    next_idx = (self.step_idx + 1) if self.step_idx is not None else None
    if next_idx is not None and self.route_geometry is not None and next_idx < len(self.route_geometry):
      for coordinate in self.route_geometry[next_idx][:12]:
        speed = coordinate.annotations.get('maxspeed', 0.0)
        if speed and speed > 0.0:
          speed_candidates.append(float(speed))

    if not speed_candidates:
      return 0.0, "none"

    # Bias conservative around maneuver transitions by taking the minimum nearby annotated speed.
    return min(speed_candidates), "annotationNearbyMin"

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

  def log_navigation_test_debug(self, instruction, geometry, distance_to_maneuver_along_geometry, command_distance, action, direction, cross_track_error=None, strategy_phase="none", strategy_threshold=0.0, strategy_constraint="none", next_maneuver_direction="none", next_maneuver_distance_after_current=None, command_actionable=False, command_direction="none", display_direction="none", current_step_error=None, global_route_error=None, command_max_lane_changes=NAVIGATION_TEST_EXIT_PREP_MAX_LANE_CHANGES, target_speed=0.0, target_speed_source="none", command_speed_active=False):
    if not self.params.get_bool("NavigationTestControl") or not self.params.get_bool("NavigationTestDriveLogging"):
      return

    now = time.monotonic()
    if now - self.navigation_test_debug_last_log_time < NAVIGATION_TEST_DEBUG_LOG_INTERVAL:
      return
    self.navigation_test_debug_last_log_time = now

    maneuver_coordinate = geometry[-1] if geometry else None
    distance_to_maneuver_straight = self.last_position.distance_to(maneuver_coordinate) if self.last_position is not None and maneuver_coordinate is not None else None
    if current_step_error is None and geometry is not None:
      current_step_error = self.path_minimum_distance(geometry)
    if global_route_error is None:
      global_route_error = self.navigation_test_cross_track_error()
    if cross_track_error is None:
      cross_track_error = global_route_error

    next_geometry = self.route_geometry[self.step_idx + 1] if (self.route_geometry is not None and self.step_idx is not None and self.step_idx + 1 < len(self.route_geometry)) else None
    maneuver_class = self.navigation_test_maneuver_class(instruction, geometry, next_geometry)
    maneuver_angle, maneuver_angle_valid = self.navigation_test_compute_maneuver_angle(geometry, next_geometry)
    left_available, right_available = self.navigation_test_lane_availability()
    lane_belief = self.navigation_test_lane_belief(left_available, right_available)
    target_lane_zone = self.navigation_test_target_zone_for_direction(direction)
    target_edge_reached = self.navigation_test_target_edge_reached(target_lane_zone, lane_belief)

    lane_width_left = ""
    lane_width_right = ""
    current_lane, total_lanes = self.navigation_test_common_lane_position()
    try:
      frogpilot_plan = self.sm['frogpilotPlan']
      lane_width_left = f"{float(frogpilot_plan.laneWidthLeft):.2f}"
      lane_width_right = f"{float(frogpilot_plan.laneWidthRight):.2f}"
    except Exception:
      pass

    prep_status_raw = self.params.get("NavigationTestPrepStatus", encoding="utf8")
    prep_status = {}
    if prep_status_raw:
      try:
        prep_status = json.loads(prep_status_raw)
      except json.JSONDecodeError:
        prep_status = {"stage": "invalidJson", "reason": "jsonDecodeError"}

    destination_id = self.params.get("NavigationTestSelectedDestination", encoding="utf8") or "home"
    migration_age = time.monotonic() - self.navigation_test_exit_migration_started_at if self.navigation_test_exit_migration_key is not None else 0.0
    row = {
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
      "maneuver_angle_deg": f"{maneuver_angle:.2f}" if maneuver_angle_valid else "",
      "maneuver_lat": f"{maneuver_coordinate.latitude:.7f}" if maneuver_coordinate is not None else "",
      "maneuver_lon": f"{maneuver_coordinate.longitude:.7f}" if maneuver_coordinate is not None else "",
      "distance_to_maneuver_along_route": f"{distance_to_maneuver_along_geometry:.2f}",
      "distance_to_maneuver_straight": f"{distance_to_maneuver_straight:.2f}" if distance_to_maneuver_straight is not None else "",
      "command_threshold": f"{command_distance:.2f}",
      "strategy_phase": strategy_phase,
      "strategy_threshold": f"{strategy_threshold:.2f}",
      "strategy_constraint": strategy_constraint,
      "lane_width_left": lane_width_left,
      "lane_width_right": lane_width_right,
      "current_lane": current_lane if current_lane is not None else "",
      "total_lanes": total_lanes if total_lanes is not None else "",
      "left_lane_available": left_available if left_available is not None else "",
      "right_lane_available": right_available if right_available is not None else "",
      "lane_belief": lane_belief,
      "target_lane_zone": target_lane_zone,
      "target_edge_reached": target_edge_reached,
      "next_maneuver_direction": next_maneuver_direction,
      "next_maneuver_distance_after_current": f"{next_maneuver_distance_after_current:.2f}" if next_maneuver_distance_after_current is not None else "",
      "migration_active": self.navigation_test_exit_migration_key is not None,
      "migration_age_seconds": f"{migration_age:.2f}",
      "migration_start_distance": f"{self.navigation_test_exit_migration_start_distance:.2f}" if self.navigation_test_exit_migration_key is not None else "",
      "action": action,
      "direction": direction,
      "command_actionable": command_actionable,
      "command_direction": command_direction,
      "display_direction": display_direction,
      "command_speed_active": command_speed_active,
      "target_speed": f"{target_speed:.2f}" if target_speed > 0.0 else "",
      "target_speed_source": target_speed_source,
      "command_max_lane_changes": command_max_lane_changes,
      "prep_stage": prep_status.get("stage", ""),
      "prep_reason": prep_status.get("reason", ""),
      "prep_completed_lane_changes": prep_status.get("completedLaneChanges", ""),
      "prep_max_lane_changes": prep_status.get("maxLaneChanges", ""),
      "prep_cooldown_remaining": f"{float(prep_status['cooldownRemaining']):.2f}" if prep_status.get("cooldownRemaining") is not None else "",
      "prep_allowed": prep_status.get("allowed", ""),
      "prep_lane_available": prep_status.get("laneAvailable", ""),
      "prep_lane_changes_enabled": prep_status.get("laneChangesEnabled", ""),
      "prep_below_lane_change_speed": prep_status.get("belowLaneChangeSpeed", ""),
      "prep_blindspot_detected": prep_status.get("blindspotDetected", ""),
      "prep_adjacent_lead_status": prep_status.get("adjacentLeadStatus", ""),
      "prep_adjacent_lead_distance": f"{float(prep_status['adjacentLeadDistance']):.2f}" if prep_status.get("adjacentLeadDistance") is not None else "",
      "prep_adjacent_lead_closing_speed": f"{float(prep_status['adjacentLeadClosingSpeed']):.2f}" if prep_status.get("adjacentLeadClosingSpeed") is not None else "",
      "prep_required_gap": f"{float(prep_status['requiredGap']):.2f}" if prep_status.get("requiredGap") is not None else "",
      "post_exit_recovery_active": self.navigation_test_post_exit_recovery_key is not None and not self.navigation_test_post_exit_recovery_done,
      "post_exit_recovery_exit_direction": self.navigation_test_post_exit_recovery_exit_direction,
      "post_exit_recovery_direction": self.navigation_test_post_exit_recovery_direction,
      "post_exit_recovery_distance": f"{self.navigation_test_post_exit_recovery_distance():.2f}" if self.navigation_test_post_exit_recovery_distance() is not None else "",
      "post_exit_recovery_done": self.navigation_test_post_exit_recovery_done,
      "current_step_error": f"{current_step_error:.2f}" if current_step_error is not None else "",
      "global_route_error": f"{global_route_error:.2f}" if global_route_error is not None else "",
      "cross_track_error": f"{cross_track_error:.2f}" if cross_track_error is not None else "",
      "recompute_reason": self.navigation_test_recompute_reason,
      "route_generation": self.navigation_test_route_generation,
    }

    try:
      debug_log_path = self.navigation_test_debug_log_path()
      if not debug_log_path:
        return

      write_header = not os.path.exists(debug_log_path) or os.path.getsize(debug_log_path) == 0
      with open(debug_log_path, "a", newline="") as debug_file:
        writer = csv.DictWriter(debug_file, fieldnames=NAVIGATION_TEST_DEBUG_LOG_FIELDS)
        if write_header:
          writer.writeheader()
        writer.writerow(row)
    except OSError:
      cloudlog.exception("navigation_test_debug.failed_to_write")

  def navigation_test_debug_log_path(self):
    if self.navigation_test_debug_log_override_path:
      override_dir = os.path.dirname(self.navigation_test_debug_log_override_path)
      if override_dir:
        os.makedirs(override_dir, exist_ok=True)
      return self.navigation_test_debug_log_override_path

    existing_path = self.params.get("NavigationTestCurrentLog", encoding="utf8")
    if existing_path:
      existing_dir = os.path.dirname(existing_path)
      if existing_dir:
        os.makedirs(existing_dir, exist_ok=True)
      return existing_path

    os.makedirs(self.navigation_test_debug_log_dir, exist_ok=True)

    destination_id = self.params.get("NavigationTestSelectedDestination", encoding="utf8") or "home"
    safe_destination = "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in destination_id)
    filename = f"navigation_test_{safe_destination}_{time.strftime('%Y%m%d_%H%M%S', time.localtime())}.csv"
    debug_log_path = os.path.join(self.navigation_test_debug_log_dir, filename)

    self.params.put("NavigationTestCurrentLog", debug_log_path)
    self.params.put("NavigationTestLastDriveLog", debug_log_path)
    return debug_log_path

  def recompute_route(self):
    self.navigation_test_recompute_reason = "none"

    if self.last_position is None:
      self.navigation_test_recompute_reason = "waitingGps"
      return

    new_destination = coordinate_from_param("NavDestination", self.params)
    if new_destination is None:
      self.navigation_test_recompute_reason = "noDestination"
      self.clear_route()
      self.reset_recompute_limits()
      return

    should_recompute = self.should_recompute()
    if new_destination != self.nav_destination:
      cloudlog.warning(f"Got new destination from NavDestination param {new_destination}")
      self.navigation_test_recompute_reason = "newDestination"
      should_recompute = True

    if not self.gps_ok and self.step_idx is not None:
      if should_recompute and self.navigation_test_recompute_reason != "none":
        self.navigation_test_recompute_reason = f"{self.navigation_test_recompute_reason}:gpsHold"
      return

    if self.recompute_countdown == 0 and should_recompute:
      if self.navigation_test_recompute_reason == "none":
        self.navigation_test_recompute_reason = "scheduledRecompute"
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
      self.navigation_test_route_generation += 1
      self.reset_navigation_test_post_exit_recovery()
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
    navigation_test_target_speed = 0.0
    navigation_test_target_speed_source = "none"
    navigation_test_command_max_lane_changes = NAVIGATION_TEST_EXIT_PREP_MAX_LANE_CHANGES
    navigation_test_actionable = False
    navigation_test_command_direction = "none"
    navigation_test_command_display_direction = "none"
    current_step_error = None
    global_route_error = None
    next_maneuver_direction = "none"
    next_maneuver_distance_after_current = None
    command_distance = 0.0
    cross_track_error = None
    
    if self.params.get_bool("NavigationTestControl"):
      command_distance = self.navigation_test_command_distance()
      maneuver_class = self.navigation_test_maneuver_class(instruction, geometry, self.route_geometry[self.step_idx + 1] if (self.route_geometry is not None and self.step_idx + 1 < len(self.route_geometry)) else None)
      if maneuver_class in ("normal_turn", "uturn"):
        command_distance = self.navigation_test_turn_command_distance()
      cross_track_error = self.navigation_test_cross_track_error()
      next_maneuver_direction, next_maneuver_distance, _ = self.navigation_test_next_maneuver(distance_to_maneuver_along_geometry)
      if next_maneuver_distance is not None:
        next_maneuver_distance_after_current = max(next_maneuver_distance - distance_to_maneuver_along_geometry, 0.0)

      if cross_track_error is not None and cross_track_error > NAVIGATION_TEST_MAX_COMMAND_CROSS_TRACK_ERROR:
        navigation_test_action = "routeMismatch"
        navigation_test_direction = "none"
        navigation_test_display_direction = "none"
        navigation_test_strategy_phase = "routeMismatch"
        navigation_test_strategy_constraint = "routeMismatch"
        self.reset_navigation_test_exit_migration()
      else:
        navigation_test_action, navigation_test_direction, navigation_test_display_direction, navigation_test_strategy_phase, navigation_test_strategy_threshold, navigation_test_strategy_constraint = self.navigation_test_strategy(
          instruction,
          geometry,
          distance_to_maneuver_along_geometry,
          command_distance,
          next_maneuver_direction,
          next_maneuver_distance_after_current,
        )

        current_actionable = navigation_test_action in ("laneChange", "turn") and navigation_test_direction in ("left", "right")
        if not current_actionable:
          post_exit_recovery = self.navigation_test_post_exit_recovery_strategy(
            instruction,
            distance_to_maneuver_along_geometry,
            next_maneuver_direction,
            next_maneuver_distance_after_current,
          )
          if post_exit_recovery is not None:
            navigation_test_action, navigation_test_direction, navigation_test_display_direction, navigation_test_strategy_phase, navigation_test_strategy_threshold, navigation_test_strategy_constraint = post_exit_recovery
            navigation_test_command_max_lane_changes = 1
        elif navigation_test_action == "laneChange":
          navigation_test_command_max_lane_changes = self.navigation_test_max_lane_changes_for_direction(navigation_test_direction)

        next_geometry = self.route_geometry[self.step_idx + 1] if (self.route_geometry is not None and self.step_idx + 1 < len(self.route_geometry)) else None
        navigation_test_target_speed, navigation_test_target_speed_source = self.navigation_test_maneuver_target_speed(instruction, geometry, maneuver_class)
        if navigation_test_strategy_phase == "postExitLaneRecovery":
          navigation_test_target_speed = 0.0
          navigation_test_target_speed_source = "none"

      navigation_test_actionable = navigation_test_action in ("laneChange", "turn") and navigation_test_direction in ("left", "right")
      navigation_test_speed_active = navigation_test_target_speed > 0.0 and navigation_test_action not in ("none", "routeMismatch", "routeError", "routing", "waitingGps")
      navigation_test_command_direction = navigation_test_direction if navigation_test_actionable else "none"
      navigation_test_display_active = navigation_test_action not in ("none", "routeMismatch", "routeError", "routing", "waitingGps")
      navigation_test_command_display_direction = navigation_test_display_direction if navigation_test_display_active else "none"
      current_step_error = self.path_minimum_distance(geometry)
      global_route_error = cross_track_error

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
        navigation_test_actionable,
        navigation_test_command_direction,
        navigation_test_command_display_direction,
        current_step_error,
        global_route_error,
        navigation_test_command_max_lane_changes,
        navigation_test_target_speed,
        navigation_test_target_speed_source,
        navigation_test_speed_active,
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
        navigation_test_command_direction,
        distance_to_maneuver_along_geometry,
        total_time,
        navigation_test_command_display_direction,
        strategy_phase=navigation_test_strategy_phase,
        strategy_constraint=navigation_test_strategy_constraint,
        target_speed=navigation_test_target_speed if navigation_test_speed_active else 0.0,
        target_speed_source=navigation_test_target_speed_source if navigation_test_speed_active else "none",
        max_lane_changes=navigation_test_command_max_lane_changes,
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
      completed_instruction = instruction
      completed_next_geometry = self.route_geometry[self.step_idx + 1] if (self.route_geometry is not None and self.step_idx + 1 < len(self.route_geometry)) else None
      completed_direction = self.navigation_test_maneuver_direction(completed_instruction, geometry, completed_next_geometry)
      completed_maneuver_class = self.navigation_test_maneuver_class(completed_instruction, geometry, completed_next_geometry)

      if self.step_idx + 1 < len(self.route):
        if completed_maneuver_class in ("highway_exit", "highway_merge") and completed_direction in ("left", "right"):
          self.start_navigation_test_post_exit_recovery(completed_instruction, geometry, completed_direction)
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
    self.reset_navigation_test_post_exit_recovery()

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
      self.navigation_test_recompute_reason = "routeMissing"
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
        self.navigation_test_recompute_reason = "globalRouteMismatch"
        cloudlog.warning(f"Navigation test route mismatch: cross_track={route_match_error:.1f}m")
        return True

    if self.step_idx == len(self.route) - 1:
      if self.params.get_bool("NavigationTestControl") and self.navigation_test_missed_destination():
        self.navigation_test_recompute_reason = "missedDestination"
        return True
      return False

    min_d = self.path_minimum_distance(self.route_geometry[self.step_idx])

    if min_d is not None and min_d > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0

    if self.reroute_counter > REROUTE_COUNTER_MIN:
      self.navigation_test_recompute_reason = "currentStepMismatch"
      return True
    return False

def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute', 'frogpilotNavigation'])
  sm = messaging.SubMaster(['carState', 'liveLocationKalman', 'managerState', 'frogpilotPlan'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()

if __name__ == "__main__":
  main()
