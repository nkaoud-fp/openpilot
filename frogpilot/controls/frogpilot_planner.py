#!/usr/bin/env python3
import json
import math

import cereal.messaging as messaging

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import A_CHANGE_COST, DANGER_ZONE_COST, J_EGO_COST, STOP_DISTANCE

from openpilot.frogpilot.common.frogpilot_utilities import calculate_lane_width, calculate_road_curvature
from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, MINIMUM_LATERAL_ACCELERATION, PLANNER_TIME, THRESHOLD, params, params_memory
from openpilot.frogpilot.controls.lib.conditional_experimental_mode import ConditionalExperimentalMode
from openpilot.frogpilot.controls.lib.frogpilot_acceleration import FrogPilotAcceleration
from openpilot.frogpilot.controls.lib.frogpilot_events import FrogPilotEvents
from openpilot.frogpilot.controls.lib.frogpilot_following import FrogPilotFollowing
from openpilot.frogpilot.controls.lib.frogpilot_vcruise import FrogPilotVCruise

class FrogPilotPlanner:
  def __init__(self, error_log, ThemeManager):
    self.cem = ConditionalExperimentalMode(self)
    self.frogpilot_acceleration = FrogPilotAcceleration(self)
    self.frogpilot_events = FrogPilotEvents(self, error_log, ThemeManager)
    self.frogpilot_following = FrogPilotFollowing(self)
    self.frogpilot_vcruise = FrogPilotVCruise(self)

    # --- ADD THIS BLOCK for Transition Smoothing Logic ---
    self.prev_experimental_mode = False
    self.smoothing_timer = 0
    # ----------------------

    with car.CarParams.from_bytes(params.get("CarParams", block=True)) as msg:
      self.CP = msg

    self.tracking_lead_filter = FirstOrderFilter(0, 0.5, DT_MDL)

    self.driving_in_curve = False
    self.lateral_check = False
    self.model_stopped = False
    self.road_curvature_detected = False
    self.slower_lead = False
    self.tracking_lead = False

    self.lane_width_left = 0
    self.lane_width_right = 0
    self.lateral_acceleration = 0
    self.model_length = 0
    self.road_curvature = 0
    self.time_to_curve = 0
    self.v_cruise = 0

  def update(self, now, time_validated, sm, frogpilot_toggles):
    self.lead_one = sm["radarState"].leadOne

    v_cruise = min(sm["controlsState"].vCruise, V_CRUISE_MAX) * CV.KPH_TO_MS
    v_ego = max(sm["carState"].vEgo, 0)

    if sm["controlsState"].enabled:
      self.frogpilot_acceleration.update(v_ego, sm, frogpilot_toggles)
    else:
      self.frogpilot_acceleration.max_accel = 0
      self.frogpilot_acceleration.min_accel = 0

    if sm["controlsState"].enabled and frogpilot_toggles.conditional_experimental_mode:
      self.cem.update(v_ego, sm, frogpilot_toggles)
      
      # ============================================================
      # START: Custom "Conditional Chill Mode" Logic
      # ============================================================
      # 1. Determine Previous State
      currently_in_chill = not self.cem.experimental_mode

      # 2. Set Hysteresis Thresholds
      if currently_in_chill:
          # STICKY (help Stay in Chill for better ecceleration):
          # Allow curves up to 2.0 m/s^2
          lat_accel_threshold = 1.8  #Increasing will cause Riskier Cornering. The car will stay in Chill Mode even on sharper curves, potentially taking them too fast before realizing it needs to slow down.
          # Stay in chill until we catch up to 95% of lead speed
          lead_ratio_threshold = 0.95 #by Increasing The car will accelerate until it nearly matches the lead's speed. Then, it will suddenly switch to Experimental and slam on the brakes because the gap is too small.
          # Stay in chill even if right lane slows to 2 m/s (~4.5 mph) BELOW our target
          right_flow_offset = -2.0 #Increasing this offset means the right lane must be flying past you to keep you in Chill. You will drop out of Chill more often.
      else:
          # STRICT (Enter Chill because experemental in slow):
          # Road must be very straight (< 1.5 m/s^2)
          lat_accel_threshold = 1.5 # Reducing will make it Harder to activate. The road must be perfectly straight to enter Chill. The car will likely remain in Experimental Mode (slow) on even slight curves.
          # Must be significantly slower than lead (< 85%)
          lead_ratio_threshold = 0.85 #0.85 Reducing will make it Harder to activate chill mode. You must be driving significantly slower than the lead car to trigger Chill. You will likely feel "stuck" in Experimental Mode while following traffic.
          # Right lane must be at least 1 m/s (~2.2 mph) FASTER than our target
          right_flow_offset = 0.1 #1.0 Reducing the offset means the right lane only needs to be slightly faster than you to trigger Chill.

      # 3. Calculate Data
      # A. Lateral Acceleration
      current_lat_accel = v_ego**2 * abs(self.road_curvature)
      
      # B. Model "Safe Speed"
      curve_speed_limit = (2.5 / max(abs(self.road_curvature), 0.001))**0.5
      model_target_speed = min(v_cruise, curve_speed_limit)

      # C. Right Lane Lead Detection
      right_lead_speed = 0
      has_right_lead = False
      if sm.valid and "modelV2" in sm.data:
        for l in sm["modelV2"].leadsV3:
          if l.prob > 0.5 and -5.0 < l.y[0] < -2.0:
            has_right_lead = True
            right_lead_speed = l.v[0]
            break

      # 4. Evaluate Triggers using Dynamic Thresholds
      is_straight = current_lat_accel < lat_accel_threshold

      # Trigger A: Significant Speed Delta with Lead
      safe_lead_gap = False
      if self.lead_one.status:
         if v_ego < (self.lead_one.vLead * lead_ratio_threshold):
             safe_lead_gap = True
      
      # Trigger B: Right lane is flowing faster (with hysteresis)
      better_flow_right = False
      if not self.lead_one.status and has_right_lead:
          # If Experimental: Only switch if right lead is > (Target + 1.0)
          # If Chill: Stay in chill as long as right lead is > (Target - 2.0)
          if right_lead_speed > (model_target_speed + right_flow_offset):
              better_flow_right = True

      # 5. Final Decision
      if is_straight and (safe_lead_gap or better_flow_right):
          self.cem.experimental_mode = False
      else:
          self.cem.experimental_mode = True

      # ============================================================
      # END: Custom Logic
      # ============================================================

      # ============================================================
      # START: Transition Smoothing Logic
      # ============================================================
      # Detect rising edge of "Chill Mode" (Experimental switching True -> False)
      if self.prev_experimental_mode and not self.cem.experimental_mode:
          # Start a 3.0 second timer (DT_MDL is typically 0.05s, so 3.0 / 0.05 = 60 frames)
          self.smoothing_timer = 3.0 / DT_MDL

      # If the timer is active, dampen the acceleration
      if self.smoothing_timer > 0:
          self.smoothing_timer -= 1
          # Cap max acceleration to a gentle 1.0 m/s^2 (approx 2.2 mph/s)
          # We use min() so we don't accidentally raise the limit if the planner wants to go slower
          self.frogpilot_acceleration.max_accel = min(self.frogpilot_acceleration.max_accel, 1.0)

      # Update state for the next frame
      self.prev_experimental_mode = self.cem.experimental_mode
      # ============================================================
      # END: Transition Smoothing Logic
      # ============================================================


    
    else:
      self.cem.curve_detected = False
      self.cem.stop_sign_and_light(v_ego, sm, PLANNER_TIME - 2)

    self.driving_in_curve = abs(self.lateral_acceleration) >= MINIMUM_LATERAL_ACCELERATION

    self.frogpilot_events.update(v_cruise, sm, frogpilot_toggles)

    self.frogpilot_following.update(v_ego, sm, frogpilot_toggles)

    localizer_valid = (sm["liveLocationKalman"].status == log.LiveLocationKalman.Status.valid) and sm["liveLocationKalman"].positionGeodetic.valid
    if sm["liveLocationKalman"].gpsOK and localizer_valid:
      gps_position = {
        "latitude": sm["liveLocationKalman"].positionGeodetic.value[0],
        "longitude": sm["liveLocationKalman"].positionGeodetic.value[1],
        "bearing": math.degrees(sm["liveLocationKalman"].calibratedOrientationNED.value[2])
      }

      params_memory.put("LastGPSPosition", json.dumps(gps_position))
    else:
      gps_position = None

      params_memory.remove("LastGPSPosition")

    self.lateral_acceleration = v_ego**2 * (sm["carState"].steeringAngleDeg - sm["liveParameters"].angleOffsetDeg) * CV.DEG_TO_RAD / (self.CP.steerRatio * self.CP.wheelbase)

    check_lane_width = frogpilot_toggles.adjacent_paths or frogpilot_toggles.adjacent_path_metrics or frogpilot_toggles.blind_spot_path or frogpilot_toggles.lane_detection
    if check_lane_width and v_ego >= frogpilot_toggles.minimum_lane_change_speed:
      self.lane_width_left = calculate_lane_width(sm["modelV2"].laneLines[0], sm["modelV2"].laneLines[1], sm["modelV2"].roadEdges[0])
      self.lane_width_right = calculate_lane_width(sm["modelV2"].laneLines[3], sm["modelV2"].laneLines[2], sm["modelV2"].roadEdges[1])
    else:
      self.lane_width_left = 0
      self.lane_width_right = 0

    self.lateral_check = v_ego >= frogpilot_toggles.pause_lateral_below_speed
    self.lateral_check |= not (sm["carState"].leftBlinker or sm["carState"].rightBlinker) and frogpilot_toggles.pause_lateral_below_signal
    self.lateral_check |= sm["carState"].standstill

    self.model_length = sm["modelV2"].position.x[-1]

    self.model_stopped = self.model_length < CRUISING_SPEED * PLANNER_TIME
    self.model_stopped |= self.frogpilot_vcruise.forcing_stop

    self.road_curvature, self.time_to_curve = calculate_road_curvature(sm["modelV2"], v_ego)

    self.road_curvature_detected = (1 / abs(self.road_curvature))**0.5 < v_ego > CRUISING_SPEED and not (sm["carState"].leftBlinker or sm["carState"].rightBlinker)

    if not sm["carState"].standstill:
      self.tracking_lead = self.update_lead_status()

    self.v_cruise = self.frogpilot_vcruise.update(gps_position, now, time_validated, v_cruise, v_ego, sm, frogpilot_toggles)

  def update_lead_status(self):
    following_lead = self.lead_one.status
    following_lead &= self.lead_one.dRel < self.model_length + STOP_DISTANCE

    self.tracking_lead_filter.update(following_lead)
    return self.tracking_lead_filter.x >= THRESHOLD

  def publish(self, theme_updated, toggles_updated, sm, pm, frogpilot_toggles):
    frogpilot_plan_send = messaging.new_message("frogpilotPlan")
    frogpilot_plan_send.valid = sm.all_checks(service_list=["carState", "controlsState"])
    frogpilotPlan = frogpilot_plan_send.frogpilotPlan

    frogpilotPlan.accelerationJerk = A_CHANGE_COST * self.frogpilot_following.acceleration_jerk
    frogpilotPlan.accelerationJerkStock = A_CHANGE_COST * self.frogpilot_following.base_acceleration_jerk
    frogpilotPlan.dangerJerk = DANGER_ZONE_COST * self.frogpilot_following.danger_jerk
    frogpilotPlan.speedJerk = J_EGO_COST * self.frogpilot_following.speed_jerk
    frogpilotPlan.speedJerkStock = J_EGO_COST * self.frogpilot_following.base_speed_jerk
    frogpilotPlan.tFollow = self.frogpilot_following.t_follow

    frogpilotPlan.cscControllingSpeed = self.frogpilot_vcruise.csc_controlling_speed
    frogpilotPlan.cscSpeed = self.frogpilot_vcruise.csc_target
    frogpilotPlan.cscTraining = self.frogpilot_vcruise.csc.enable_training

    frogpilotPlan.desiredFollowDistance = self.frogpilot_following.desired_follow_distance

    frogpilotPlan.experimentalMode = self.cem.experimental_mode or self.frogpilot_vcruise.slc.experimental_mode

    frogpilotPlan.forcingStop = self.frogpilot_vcruise.forcing_stop
    frogpilotPlan.forcingStopLength = self.frogpilot_vcruise.tracked_model_length

    frogpilotPlan.frogpilotEvents = self.frogpilot_events.events.to_msg()

    frogpilotPlan.increasedStoppedDistance = frogpilot_toggles.increase_stopped_distance if not sm["frogpilotCarState"].trafficModeEnabled else 0

    frogpilotPlan.laneWidthLeft = self.lane_width_left
    frogpilotPlan.laneWidthRight = self.lane_width_right

    frogpilotPlan.lateralCheck = self.lateral_check

    frogpilotPlan.maxAcceleration = self.frogpilot_acceleration.max_accel
    frogpilotPlan.minAcceleration = self.frogpilot_acceleration.min_accel

    frogpilotPlan.redLight = self.cem.stop_light_detected

    frogpilotPlan.roadCurvature = self.road_curvature

    frogpilotPlan.slcMapSpeedLimit = self.frogpilot_vcruise.slc.map_speed_limit
    frogpilotPlan.slcMapboxSpeedLimit = self.frogpilot_vcruise.slc.mapbox_limit
    frogpilotPlan.slcNextSpeedLimit = self.frogpilot_vcruise.slc.next_speed_limit
    frogpilotPlan.slcOverriddenSpeed = self.frogpilot_vcruise.slc.overridden_speed
    frogpilotPlan.slcSpeedLimit = self.frogpilot_vcruise.slc_target
    frogpilotPlan.slcSpeedLimitOffset = self.frogpilot_vcruise.slc_offset
    frogpilotPlan.slcSpeedLimitSource = self.frogpilot_vcruise.slc.source
    frogpilotPlan.speedLimitChanged = self.frogpilot_vcruise.slc.speed_limit_changed_timer > DT_MDL
    frogpilotPlan.unconfirmedSlcSpeedLimit = self.frogpilot_vcruise.slc.unconfirmed_speed_limit

    frogpilotPlan.themeUpdated = theme_updated or params_memory.get_bool("UseActiveTheme")

    frogpilotPlan.togglesUpdated = toggles_updated

    frogpilotPlan.trackingLead = self.tracking_lead

    frogpilotPlan.vCruise = self.v_cruise

    pm.send("frogpilotPlan", frogpilot_plan_send)
