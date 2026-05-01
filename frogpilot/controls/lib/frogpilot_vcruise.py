#!/usr/bin/env python3
import json

from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, PLANNER_TIME, params
from openpilot.frogpilot.controls.lib.curve_speed_controller import CurveSpeedController
from openpilot.frogpilot.controls.lib.speed_limit_controller import SpeedLimitController

NAVIGATION_EXIT_TARGET_SPEED = 17.9  # 40 mph / 64 kph
NAVIGATION_TURN_TARGET_SPEED_SHARP = 9.0   # 20 mph / 32 kph
NAVIGATION_PREP_DECEL = 1.2

class FrogPilotVCruise:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.csc = CurveSpeedController(self)
    self.slc = SpeedLimitController()

    self.forcing_stop = False
    self.override_force_stop = False

    self.force_stop_timer = 0
    self.override_force_stop_timer = 0
    self.tracked_model_length = 0

    self.braking_target = 0
    self.csc_controlling_speed = False
    self.csc_target = 0
    self.navigation_prep_target = 0

  def update(self, gps_position, now, time_validated, v_cruise, v_ego, sm, frogpilot_toggles):
    force_stop = self.frogpilot_planner.cem.stop_light_detected and sm["controlsState"].enabled and frogpilot_toggles.force_stops
    force_stop &= self.frogpilot_planner.model_stopped
    force_stop &= self.override_force_stop_timer <= 0

    self.force_stop_timer = self.force_stop_timer + DT_MDL if force_stop else 0

    force_stop_enabled = self.force_stop_timer >= 1

    self.override_force_stop |= sm["carState"].gasPressed
    self.override_force_stop |= sm["frogpilotCarState"].accelPressed
    self.override_force_stop &= force_stop_enabled

    if self.override_force_stop:
      self.override_force_stop_timer = 10
    elif self.override_force_stop_timer > 0:
      self.override_force_stop_timer -= DT_MDL

    v_cruise_cluster = max(sm["controlsState"].vCruiseCluster * CV.KPH_TO_MS, v_cruise)
    v_cruise_diff = v_cruise_cluster - v_cruise

    v_ego_cluster = max(sm["carState"].vEgoCluster, v_ego)
    v_ego_diff = v_ego_cluster - v_ego

    # FrogsGoMoo's Curve Speed Controller
    if v_ego > CRUISING_SPEED and sm["controlsState"].enabled and self.frogpilot_planner.road_curvature_detected and frogpilot_toggles.curve_speed_controller:
      self.csc.update_target(v_ego)

      self.csc_controlling_speed = True

      self.csc_target = self.csc.target
    else:
      self.csc.log_data(v_ego, sm)

      self.csc_controlling_speed = False
      self.csc.target_set = False

      self.csc_target = v_cruise

    # Mike's extended lead linear braking
    if self.frogpilot_planner.lead_one.vLead < v_ego > CRUISING_SPEED and sm["controlsState"].enabled and self.frogpilot_planner.tracking_lead and frogpilot_toggles.human_following:
      if not self.frogpilot_planner.frogpilot_following.following_lead:
        decel_rate = (v_ego - self.frogpilot_planner.lead_one.vLead)**2 / self.frogpilot_planner.lead_one.dRel
        self.braking_target = max(v_ego - (decel_rate * DT_MDL), self.frogpilot_planner.lead_one.vLead + CRUISING_SPEED)
      else:
        self.braking_target = v_cruise
    else:
      self.braking_target = v_cruise

    # Pfeiferj's Speed Limit Controller
    self.slc.frogpilot_toggles = frogpilot_toggles

    if frogpilot_toggles.speed_limit_controller:
      self.slc.update_limits(sm["frogpilotCarState"].dashboardSpeedLimit, gps_position, sm["frogpilotNavigation"].navigationSpeedLimit, now, time_validated, v_cruise, v_ego, sm)
      self.slc.update_override(v_cruise, v_cruise_diff, v_ego, v_ego_diff, sm)

      self.slc_offset = self.slc.offset
      self.slc_target = self.slc.target
    elif frogpilot_toggles.show_speed_limits:
      self.slc.update_limits(sm["frogpilotCarState"].dashboardSpeedLimit, gps_position, sm["frogpilotNavigation"].navigationSpeedLimit, now, time_validated, v_cruise, v_ego, sm)

      self.slc_offset = 0
      self.slc_target = self.slc.target
    else:
      self.slc_offset = 0
      self.slc_target = 0

    self.navigation_prep_target = self.update_navigation_prep_target(v_cruise, frogpilot_toggles)

    if force_stop_enabled and not self.override_force_stop:
      self.forcing_stop |= not sm["carState"].standstill

      self.tracked_model_length = max(self.tracked_model_length - (v_ego * DT_MDL), 0)
      v_cruise = min((self.tracked_model_length // PLANNER_TIME), v_cruise)

    else:
      self.forcing_stop = False

      self.tracked_model_length = self.frogpilot_planner.model_length

      targets = [self.braking_target, self.csc_target, v_cruise]
      if self.navigation_prep_target > CRUISING_SPEED:
        targets.append(self.navigation_prep_target)
      if frogpilot_toggles.speed_limit_controller:
        targets.append(max(self.slc.overridden_speed, self.slc_target + self.slc_offset) - v_ego_diff)

      v_cruise = min([target if target > CRUISING_SPEED else v_cruise for target in targets])

    return v_cruise

  def update_navigation_prep_target(self, v_cruise, frogpilot_toggles):
    if not params.get_bool("NavigationTestControl"):
      return 0

    command = params.get("NavigationTestTurnCommand", encoding="utf-8")
    if command is None:
      return 0

    try:
      command_json = json.loads(command)
    except json.JSONDecodeError:
      return 0

    action = command_json.get("action", "none")
    target_speed_from_nav = float(command_json.get("targetSpeed", 0.0))
    if target_speed_from_nav <= 0.0:
      target_speed_from_nav = 0.0

    if action not in ("laneChange", "turn", "upcoming"):
      return 0

    distance = float(command_json.get("distance", 0.0))
    if distance <= 0:
      return 0

    display_direction = str(command_json.get("displayDirection", "none")).lower()
    is_sharp_turn = display_direction in ("sharp_left", "sharp_right", "uturn")

    strategy_phase = command_json.get("strategyPhase", "none")
    turn_related_phase = strategy_phase in ("turn", "turnLanePositioning", "targetEdgeHold", "maneuverLockout")
    turn_slowdown_start_distance = max(
      0.0,
      float(getattr(frogpilot_toggles, "navigation_test_turn_slowdown_start_distance", 0.0)),
    )
    if turn_related_phase and turn_slowdown_start_distance > 0.0 and distance > turn_slowdown_start_distance:
      return 0

    if action == "laneChange" or "Exit" in strategy_phase:
      target_speed = target_speed_from_nav if target_speed_from_nav > 0.0 else NAVIGATION_EXIT_TARGET_SPEED
    elif action in ("turn", "upcoming") and target_speed_from_nav > 0.0 and turn_related_phase:
      target_speed = target_speed_from_nav
    elif action == "turn" and is_sharp_turn:
      target_speed = target_speed_from_nav if target_speed_from_nav > 0.0 else NAVIGATION_TURN_TARGET_SPEED_SHARP
    else:
      return 0

    decel = min(NAVIGATION_PREP_DECEL, COMFORT_BRAKE)
    speed_target_now = (target_speed ** 2 + 2.0 * decel * distance) ** 0.5

    return float(min(v_cruise, max(speed_target_now, target_speed, CRUISING_SPEED)))
