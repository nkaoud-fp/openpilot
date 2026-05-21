#!/usr/bin/env python3
import json
import math
import os ### only needed for greenlight hack

from collections import Counter, deque

import cereal.messaging as messaging

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import A_CHANGE_COST, DANGER_ZONE_COST, J_EGO_COST, STOP_DISTANCE

from openpilot.frogpilot.common.frogpilot_utilities import calculate_lane_width, calculate_road_curvature, compute_lane_positions, LANE_POSITION_METHODS
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
    
    # NEW: Timer for Green Light Auto-Resume
    self.green_light_timer = 0
    
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

    self.lane_positions = [(label, 0, 0, "unknown", 0.0, 0.0) for label, _ in LANE_POSITION_METHODS]
    self.lane_history = {label: deque(maxlen=10) for label, _ in LANE_POSITION_METHODS}  # 0.5 s mode window per method
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
      # Read the raw AI model's desire to stop
      model_wants_to_stop = False
      if sm.valid and "modelV2" in sm.data:
          model_wants_to_stop = sm["modelV2"].action.shouldStop
      is_stopping_for_light = self.cem.stop_light_detected # Don't override if stopping for a light!

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

      # ============================================================
      # NEW: Trigger C: Green Light Auto-Resume (State Transition)
      # ============================================================
      # 1. Initialize our memory variable (runs once)
      if not hasattr(self, 'was_stopped_for_light'):
          self.was_stopped_for_light = False

      # 2. Track if we are actively stopped for a red light
      if sm["carState"].standstill and self.cem.stop_light_detected:
          self.was_stopped_for_light = True
          self.green_light_timer = 0 # Keep timer at 0 while waiting
          
      # 3. Detect the Green Light (we were stopped, but the light is no longer red)
      elif self.was_stopped_for_light and not self.cem.stop_light_detected:
          self.green_light_timer = 4.0 / DT_MDL  # Start the 4-second timer
          self.was_stopped_for_light = False     # Reset memory so timer can count down

      force_green_light_chill = False
      if self.green_light_timer > 0:
          force_green_light_chill = True
          self.green_light_timer -= 1

      # --- NEW: Broadcast a "Force Resume" flag to the car's hardware ---
      #params_memory.put_bool("GreenLightAutoResume", force_green_light_chill)
      # --- NEW: Broadcast a "Force Resume" flag via RAM disk ---
      resume_flag_path = "/dev/shm/green_light_resume"
      if force_green_light_chill:
          try:
              # Only create the file if it isn't already there!
              if not os.path.exists(resume_flag_path):
                  open(resume_flag_path, 'w').close()
          except Exception:
              pass
      else:
          try:
              if os.path.exists(resume_flag_path):
                  os.remove(resume_flag_path)       # Delete the flag when timer ends
          except Exception:
              pass
      # ------------------------------------------------------------------
      # ------------------------------------------------------------------

      '''
      # 5. Final Decision
      if is_straight and not model_wants_to_stop and (safe_lead_gap or (better_flow_right and not is_stopping_for_light)):
          self.cem.experimental_mode = False
      else:
          self.cem.experimental_mode = True
      '''

      # 5. Final Decision
      
      safe_lead_gap = False # to disable Chill on lead
      better_flow_right = False # to disable Chill on adjacent
      
      if force_green_light_chill:
          # Force Chill Mode to automatically resume at the green light
          self.cem.experimental_mode = False
      elif is_straight and not model_wants_to_stop and (safe_lead_gap or (better_flow_right and not is_stopping_for_light)):
          self.cem.experimental_mode = False
      else:
          self.cem.experimental_mode = True

      # ============================================================
      # END: Custom Logic
      # ============================================================

      """
      # ============================================================
      # START: Gradual Transition Smoothing (Ramped)
      # ============================================================
      # 1. Detect switch from Experimental -> Chill
      #if self.prev_experimental_mode and not self.cem.experimental_mode:
      if self.prev_experimental_mode and not self.cem.experimental_mode and not force_green_light_chill:
          self.smoothing_timer = 5.0 / DT_MDL  # 5 seconds 
          self.initial_v_ego = v_ego           # Record speed at start of transition

      
      # 2. If timer is active, ramp the acceleration limit up
      if self.smoothing_timer > 0:
          # total_frames = 60
          # current_progress goes from 0.0 (start) to 1.0 (end)
          total_frames = 5.0 / DT_MDL # 5 seconds 
          current_progress = 1.0 - (self.smoothing_timer / total_frames)
          
          # Ramp max_accel from 0.2 m/s^2 up to 1.2 m/s^2 over 3 seconds
          # This forces a smooth "roll-on" of power
          ramped_accel = 0.2 + (1.0 * current_progress)
          
          self.frogpilot_acceleration.max_accel = min(self.frogpilot_acceleration.max_accel, ramped_accel)
          self.smoothing_timer -= 1

      self.prev_experimental_mode = self.cem.experimental_mode
      
      # ============================================================
      # END: Transition Smoothing Logic
      # ============================================================
      """

      # ============================================================
      # START: Gradual Transition Smoothing (S-Curve Dynamic)
      # ============================================================
      # 1. Detect switch from Experimental -> Chill
      if self.prev_experimental_mode and not self.cem.experimental_mode and not force_green_light_chill:
          self.smoothing_timer = 5.0 / DT_MDL  # 5 seconds (100 frames)
          self.initial_v_ego = v_ego           # Record speed at start of transition
          
          # NEW: Capture real-time acceleration, floored at 0.0 so we don't start from a braking state
          self.initial_a_ego = max(sm["carState"].aEgo, 0.0)

      # 2. Apply the S-Curve ramp
      if self.smoothing_timer > 0:
          total_frames = 5.0 / DT_MDL  # 5 seconds (100 frames)
          current_progress = 1.0 - (self.smoothing_timer / total_frames)
          
          # S-Curve using Sine: starts flat, gets steep, ends flat
          s_curve_factor = (math.sin((current_progress * math.pi) - (math.pi / 2)) + 1) / 2
          
          # The target cap we want to reach is 1.2 m/s^2
          target_max_accel = 1.2
          
          # Calculate the difference between where we are and where we are going
          accel_gap = target_max_accel - self.initial_a_ego
          
          # Start at current acceleration, and add the S-Curve percentage of the gap
          ramped_accel = self.initial_a_ego + (accel_gap * s_curve_factor)
          
          self.frogpilot_acceleration.max_accel = min(self.frogpilot_acceleration.max_accel, ramped_accel)
          self.smoothing_timer -= 1

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

    smoothed = []
    for label, raw_current, raw_total, raw_conf, dbg_l, dbg_r in compute_lane_positions(sm["modelV2"]):
      hist = self.lane_history[label]
      hist.append((raw_current, raw_total, raw_conf))
      (cur, tot), lane_count = Counter((c, t) for c, t, _ in hist).most_common(1)[0]
      conf_mode, _ = Counter(c for _, _, c in hist).most_common(1)[0]
      conf = conf_mode if lane_count >= 0.8 * len(hist) else "low"
      smoothed.append((label, cur, tot, conf, dbg_l, dbg_r))
    self.lane_positions = smoothed

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

    primary = self.lane_positions[0] if self.lane_positions else (None, 0, 0, "unknown", 0.0, 0.0)
    frogpilotPlan.currentLane = primary[1]
    frogpilotPlan.totalLanes = primary[2]
    frogpilotPlan.laneConfidence = primary[3]

    lane_positions_msg = frogpilotPlan.init("lanePositions", len(self.lane_positions))
    for i, (label, cur, tot, conf, dbg_l, dbg_r) in enumerate(self.lane_positions):
      lane_positions_msg[i].method = label
      lane_positions_msg[i].currentLane = cur
      lane_positions_msg[i].totalLanes = tot
      lane_positions_msg[i].confidence = conf
      lane_positions_msg[i].debugLeft = dbg_l
      lane_positions_msg[i].debugRight = dbg_r

    pm.send("frogpilotPlan", frogpilot_plan_send)
