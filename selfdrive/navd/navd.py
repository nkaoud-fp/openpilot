#!/usr/bin/env python3
import csv
import hashlib
import json
import math
import os
import time
import threading

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
from openpilot.selfdrive.navd.osrm import request_osrm_route
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
NAVIGATION_TEST_COMMAND_DISTANCE = 35
NAVIGATION_TEST_COMMAND_SECONDS = 8
NAVIGATION_TEST_EXIT_PREP_DISTANCE = 800
NAVIGATION_TEST_MAX_COMMAND_CROSS_TRACK_ERROR = 15
NAVIGATION_TEST_REROUTE_COUNTER_MIN = 2
NAVIGATION_TEST_DEBUG_LOG_PATH = "/data/media/0/navigation_test_debug.csv"
NAVIGATION_TEST_DEBUG_LOG_INTERVAL = 0.5
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
  "maneuver_lat",
  "maneuver_lon",
  "distance_to_maneuver_along_route",
  "distance_to_maneuver_straight",
  "command_threshold",
  "action",
  "direction",
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
    self.navigation_test_command = None
    self.navigation_test_debug_last_log_time = 0.0
    self.navigation_test_debug_log_path = os.environ.get("NAVIGATION_TEST_DEBUG_LOG_PATH", NAVIGATION_TEST_DEBUG_LOG_PATH)


    self.api = None
    self.mapbox_token = None
    if "MAPBOX_TOKEN" in os.environ:
      self.mapbox_token = os.environ["MAPBOX_TOKEN"]
      self.mapbox_host = "https://api.mapbox.com"
    else:
      self.mapbox_token = self.params.get("MapboxSecretKey", encoding='utf8')
      self.mapbox_host = "https://api.mapbox.com"

    # FrogPilot variables
    self.approaching_intersection = False
    self.approaching_turn = False

    self.nav_speed_limit = 0

    self.stop_coord = []
    self.stop_signal = []

    self.frogpilot_toggles = get_frogpilot_toggles()

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

  def update_navigation_test_command(self, action, direction="none", distance=0.0, eta_seconds=0.0):
    command = json.dumps({
      "action": action,
      "direction": direction,
      "distance": max(distance, 0.0),
      "etaSeconds": max(eta_seconds, 0.0),
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

  def navigation_test_is_exit_maneuver(self, instruction):
    if instruction is None:
      return False

    maneuver_type = instruction.get("maneuverType", "").lower()
    primary_text = instruction.get("maneuverPrimaryText", "").lower()
    return "ramp" in maneuver_type or "exit" in primary_text

  def navigation_test_command_distance(self):
    v_ego = self.sm['carState'].vEgo
    return max(NAVIGATION_TEST_COMMAND_DISTANCE, v_ego * NAVIGATION_TEST_COMMAND_SECONDS)

  def navigation_test_cross_track_error(self):
    if self.route_geometry is None or self.last_position is None:
      return None

    closest_distance = None
    for geometry in self.route_geometry:
      for i in range(len(geometry) - 1):
        distance = minimum_distance(geometry[i], geometry[i + 1], self.last_position)
        closest_distance = distance if closest_distance is None else min(closest_distance, distance)
    return closest_distance

  def log_navigation_test_debug(self, instruction, geometry, distance_to_maneuver_along_geometry, command_distance, action, direction, cross_track_error=None):
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
      "maneuver_lat": f"{maneuver_coordinate.latitude:.7f}" if maneuver_coordinate is not None else "",
      "maneuver_lon": f"{maneuver_coordinate.longitude:.7f}" if maneuver_coordinate is not None else "",
      "distance_to_maneuver_along_route": f"{distance_to_maneuver_along_geometry:.2f}",
      "distance_to_maneuver_straight": f"{distance_to_maneuver_straight:.2f}" if distance_to_maneuver_straight is not None else "",
      "command_threshold": f"{command_distance:.2f}",
      "action": action,
      "direction": direction,
      "cross_track_error": f"{cross_track_error:.2f}" if cross_track_error is not None else "",
    }

    try:
      write_header = not os.path.exists(self.navigation_test_debug_log_path) or os.path.getsize(self.navigation_test_debug_log_path) == 0
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

    # Don't recompute when GPS drifts in tunnels
    if not self.gps_ok and self.step_idx is not None:
      return

    if self.params.get_bool("NavigationTestControl") and should_recompute:
      self.recompute_countdown = 0
      self.recompute_backoff = 0

    if self.recompute_countdown == 0 and should_recompute:
      self.recompute_countdown = 2**self.recompute_backoff
      self.recompute_backoff = min(6, self.recompute_backoff + 1)
      self.calculate_route(new_destination)
      self.reroute_counter = 0
      self.navigation_test_reroute_counter = 0
    else:
      self.recompute_countdown = max(0, self.recompute_countdown - 1)

  def calculate_route(self, destination):
    cloudlog.warning(f"Calculating route {self.last_position} -> {destination}")
    self.nav_destination = destination
    if self.params.get_bool("NavigationTestControl"):
      self.update_navigation_test_command("routing")

    # TODO: move waypoints into NavDestination param?
    waypoints = self.params.get('NavDestinationWaypoints', encoding='utf8')
    waypoint_coords = []
    if waypoints is not None and len(waypoints) > 0:
      waypoint_coords = json.loads(waypoints)

    coords = [
      (self.last_position.longitude, self.last_position.latitude),
      *waypoint_coords,
      (destination.longitude, destination.latitude)
    ]

    coords_str = ';'.join([f'{lon},{lat}' for lon, lat in coords])
    url = self.mapbox_host + '/directions/v5/mapbox/driving-traffic/' + coords_str
    try:
      if self.params.get_bool("NavigationTestControl"):
        chosen_route, r = request_osrm_route(self.last_position, destination, self.last_bearing)
        if chosen_route is None:
          cloudlog.warning("Got empty OSRM route response")
          self.update_navigation_test_command("noRoute")
          self.clear_route()
          self.send_route()
          return
        r1 = r
      else:
        lang = self.params.get('LanguageSetting', encoding='utf8')
        if lang is not None:
          lang = lang.replace('main_', '')

        token = self.mapbox_token
        if token is None:
          token = self.api.get_token()

        params = {
          'access_token': token,
          'annotations': 'maxspeed',
          'geometries': 'geojson',
          'overview': 'full',
          'steps': 'true',
          'banner_instructions': 'true',
          'alternatives': 'true',
          'language': lang,
        }
        params['waypoints'] = f'0;{len(coords)-1}'
        if self.last_bearing is not None:
          params['bearings'] = f"{(self.last_bearing + 360) % 360:.0f},90" + (';'*(len(coords)-1))

        resp = requests.get(url, params=params, timeout=10)
        if resp.status_code != 200:
          cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)
        resp.raise_for_status()

        r = resp.json()
        r1 = resp.json()
        chosen_route = r['routes'][0]

      # Function to remove specified keys recursively unnessary for display
      def remove_keys(obj, keys_to_remove):
        if isinstance(obj, list):
          return [remove_keys(item, keys_to_remove) for item in obj]
        elif isinstance(obj, dict):
          return {key: remove_keys(value, keys_to_remove) for key, value in obj.items() if key not in keys_to_remove}
        else:
          return obj

      keys_to_remove = ['geometry', 'annotation', 'incidents', 'intersections', 'components', 'sub', 'waypoints']
      self.r2 = remove_keys(r1, keys_to_remove)
      self.r3 = {}

      # Add items for display under "routes"
      if 'routes' in self.r2 and len(self.r2['routes']) > 0:
        first_route = self.r2['routes'][0]
        nav_destination_json = self.params.get('NavDestination')

        try:
          route_hash = json.loads(nav_destination_json).get('routeHash')
        except Exception:
          route_hash = None

        if route_hash:
          for cand in r['routes']:
            flat = ','.join(str(coordinate) for pair in cand['geometry']['coordinates'] for coordinate in pair)
            if hashlib.sha1(flat.encode()).hexdigest() == route_hash:
              chosen_route = cand
              break

        try:
          nav_destination_data = json.loads(nav_destination_json)
          place_name = nav_destination_data.get('place_name', 'Default Place Name')
          first_route['Destination'] = place_name
          first_route['Metric'] = self.params.get_bool("IsMetric")
          self.r3['CurrentStep'] = 0
          self.r3['uuid'] = self.r2.get('uuid', 'osrm-navigation-test')
        except json.JSONDecodeError as e:
          print(f"Error decoding JSON: {e}")

      # Save slim json as file
      with open('navdirections.json', 'w') as json_file:
        json.dump(self.r2, json_file, indent=4)
      with open('CurrentStep.json', 'w') as json_file:
        json.dump(self.r3, json_file, indent=4)

      if len(r['routes']):
        self.route = chosen_route['legs'][0]['steps']
        self.route_geometry = []

        # Iterate through the steps in self.route to find "stop_sign" and "traffic_light"
        if self.frogpilot_toggles.conditional_navigation_intersections:
          self.stop_signal = []
          self.stop_coord = []

          for step in self.route:
            for intersection in step["intersections"]:
              if "stop_sign" in intersection or "traffic_signal" in intersection:
                self.stop_signal.append(intersection["geometry_index"])
                self.stop_coord.append(Coordinate.from_mapbox_tuple(intersection["location"]))

        maxspeed_idx = 0
        maxspeeds = chosen_route['legs'][0].get('annotation', {}).get('maxspeed', [])

        # Convert coordinates
        for step in self.route:
          coords = []

          for c in step['geometry']['coordinates']:
            coord = Coordinate.from_mapbox_tuple(c)

            # Last step does not have maxspeed
            if (maxspeed_idx < len(maxspeeds)):
              maxspeed = maxspeeds[maxspeed_idx]
              if ('unknown' not in maxspeed) and ('none' not in maxspeed):
                coord.annotations['maxspeed'] = maxspeed_to_ms(maxspeed)

            coords.append(coord)
            maxspeed_idx += 1

          self.route_geometry.append(coords)
          maxspeed_idx -= 1  # Every segment ends with the same coordinate as the start of the next

        self.step_idx = 0
      else:
        cloudlog.warning("Got empty route response")
        self.clear_route()

      # clear waypoints to avoid a re-route including past waypoints
      # TODO: only clear once we're past a waypoint
      self.params.remove('NavDestinationWaypoints')

    except requests.exceptions.RequestException:
      cloudlog.exception("failed to get route")
      if self.params.get_bool("NavigationTestControl"):
        self.update_navigation_test_command("routeError")
      self.clear_route()

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

    # Banner instructions are for the following maneuver step, don't use empty last step
    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]

    # Current instruction
    msg.navInstruction.maneuverDistance = distance_to_maneuver_along_geometry
    instruction = parse_banner_instructions(banner_step['bannerInstructions'], distance_to_maneuver_along_geometry)
    if instruction is not None:
      for k,v in instruction.items():
        setattr(msg.navInstruction, k, v)

    navigation_test_action = "none"
    navigation_test_direction = "none"
    command_distance = 0.0
    cross_track_error = None
    if self.params.get_bool("NavigationTestControl"):
      command_distance = self.navigation_test_command_distance()
      cross_track_error = self.navigation_test_cross_track_error()
      navigation_test_direction = self.navigation_test_maneuver_direction(instruction)

      if cross_track_error is not None and cross_track_error > NAVIGATION_TEST_MAX_COMMAND_CROSS_TRACK_ERROR:
        navigation_test_action = "routeMismatch"
        navigation_test_direction = "none"
      elif navigation_test_direction != "none":
        if self.navigation_test_is_exit_maneuver(instruction) and command_distance < distance_to_maneuver_along_geometry <= NAVIGATION_TEST_EXIT_PREP_DISTANCE:
          navigation_test_action = "laneChange"
        elif distance_to_maneuver_along_geometry <= command_distance:
          navigation_test_action = "turn"
        else:
          navigation_test_action = "upcoming"

      self.log_navigation_test_debug(instruction, geometry, distance_to_maneuver_along_geometry, command_distance, navigation_test_action, navigation_test_direction, cross_track_error)

    # All instructions
    maneuvers = []
    for i, step_i in enumerate(self.route):
      if i < self.step_idx:
        distance_to_maneuver = -sum(self.route[j]['distance'] for j in range(i+1, self.step_idx)) - along_geometry
      elif i == self.step_idx:
        distance_to_maneuver = distance_to_maneuver_along_geometry
      else:
        distance_to_maneuver = distance_to_maneuver_along_geometry + sum(self.route[j]['distance'] for j in range(self.step_idx+1, i+1))

      instruction = parse_banner_instructions(step_i['bannerInstructions'], distance_to_maneuver)
      if instruction is None:
        continue
      maneuver = {'distance': distance_to_maneuver}
      if 'maneuverType' in instruction:
        maneuver['type'] = instruction['maneuverType']
      if 'maneuverModifier' in instruction:
        maneuver['modifier'] = instruction['maneuverModifier']
      maneuvers.append(maneuver)

    msg.navInstruction.allManeuvers = maneuvers

    # Compute total remaining time and distance
    remaining = 1.0 - along_geometry / max(step['distance'], 1)
    total_distance = step['distance'] * remaining
    total_time = step['duration'] * remaining

    if step['duration_typical'] is None:
      total_time_typical = total_time
    else:
      total_time_typical = step['duration_typical'] * remaining

    # Add up totals for future steps
    for i in range(self.step_idx + 1, len(self.route)):
      total_distance += self.route[i]['distance']
      total_time += self.route[i]['duration']
      if self.route[i]['duration_typical'] is None:
        total_time_typical += self.route[i]['duration']
      else:
        total_time_typical += self.route[i]['duration_typical']

    msg.navInstruction.distanceRemaining = total_distance
    msg.navInstruction.timeRemaining = total_time
    msg.navInstruction.timeRemainingTypical = total_time_typical

    if self.params.get_bool("NavigationTestControl"):
      self.update_navigation_test_command(
        navigation_test_action,
        navigation_test_direction if navigation_test_action != "none" else "none",
        distance_to_maneuver_along_geometry,
        total_time,
      )

    # Speed limit
    closest_idx, closest = min(enumerate(geometry), key=lambda p: p[1].distance_to(self.last_position))
    if closest_idx > 0:
      # If we are not past the closest point, show previous
      if along_geometry < distance_along_geometry(geometry, geometry[closest_idx]):
        closest = geometry[closest_idx - 1]

    if ('maxspeed' in closest.annotations) and self.localizer_valid:
      msg.navInstruction.speedLimit = closest.annotations['maxspeed']
      self.nav_speed_limit = closest.annotations['maxspeed']
    if not self.localizer_valid or ('maxspeed' not in closest.annotations):
      self.nav_speed_limit = 0

    # Speed limit sign type
    if 'speedLimitSign' in step:
      if step['speedLimitSign'] == 'mutcd':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.mutcd
      elif step['speedLimitSign'] == 'vienna':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.vienna

    self.pm.send('navInstruction', msg)

    # Transition to next route segment
    if distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD:
      if self.step_idx + 1 < len(self.route):
        self.step_idx += 1
        self.reset_recompute_limits()

        # Update the 'CurrentStep' value in the JSON
        if 'routes' in self.r2 and len(self.r2['routes']) > 0:
          self.r3['CurrentStep'] = self.step_idx
        # Write the modified JSON data back to the file
        with open('CurrentStep.json', 'w') as json_file:
          json.dump(self.r3, json_file, indent=4)
      else:
        cloudlog.warning("Destination reached")

        # Clear route if driving away from destination
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

  def reset_recompute_limits(self):
    self.recompute_backoff = 0
    self.recompute_countdown = 0

  def should_recompute(self):
    if self.step_idx is None or self.route is None:
      return True

    if self.params.get_bool("NavigationTestControl"):
      route_match_error = self.navigation_test_cross_track_error()
      if route_match_error is not None and route_match_error > self.navigation_test_command_distance():
        self.navigation_test_reroute_counter += 1
      else:
        self.navigation_test_reroute_counter = 0

      if self.navigation_test_reroute_counter > NAVIGATION_TEST_REROUTE_COUNTER_MIN:
        cloudlog.warning(f"Navigation test route mismatch: cross_track={route_match_error:.1f}m")
        return True

    # Don't recompute in last segment, assume destination is reached
    if self.step_idx == len(self.route) - 1:
      return False

    # Compute closest distance to all line segments in the current path
    min_d = REROUTE_DISTANCE + 1
    path = self.route_geometry[self.step_idx]
    for i in range(len(path) - 1):
      a = path[i]
      b = path[i + 1]

      if a.distance_to(b) < 1.0:
        continue

      min_d = min(min_d, minimum_distance(a, b, self.last_position))

    if min_d > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
    return self.reroute_counter > REROUTE_COUNTER_MIN
    # TODO: Check for going wrong way in segment


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
