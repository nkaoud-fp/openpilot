#!/usr/bin/env python3
import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE, STOP_DISTANCE, desired_follow_distance, get_jerk_factor, get_T_FOLLOW

from openpilot.frogpilot.common.frogpilot_variables import CITY_SPEED_LIMIT

TRAFFIC_MODE_BP = [0., CITY_SPEED_LIMIT]

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.following_lead = False
    self.slower_lead = False

    self.acceleration_jerk = 0
    self.danger_jerk = 0
    self.desired_follow_distance = 0
    self.speed_jerk = 0
    self.t_follow = 0

  def update(self, v_ego, sm, frogpilot_toggles):
    if sm["controlsState"].enabled and sm["frogpilotCarState"].trafficModeEnabled:
      if sm["carState"].aEgo >= 0:
        self.base_acceleration_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_acceleration))
        self.base_speed_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed))
      else:
        self.base_acceleration_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_deceleration))
        self.base_speed_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed_decrease))

      self.base_danger_jerk = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_danger))
      self.t_follow = float(np.interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_follow))
    elif sm["controlsState"].enabled:
      if sm["carState"].aEgo >= 0:
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_acceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed,
          frogpilot_toggles.standard_jerk_acceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed,
          frogpilot_toggles.relaxed_jerk_acceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed,
          frogpilot_toggles.custom_personalities, sm["controlsState"].personality
        )
      else:
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_deceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed_decrease,
          frogpilot_toggles.standard_jerk_deceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed_decrease,
          frogpilot_toggles.relaxed_jerk_deceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed_decrease,
          frogpilot_toggles.custom_personalities, sm["controlsState"].personality
        )

      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities, sm["controlsState"].personality
      )
    else:
      self.base_acceleration_jerk = 0
      self.base_danger_jerk = 0
      self.base_speed_jerk = 0
      self.t_follow = 0

    #### ========================================================================
    #### Stopped Car LOGIC - ADD THIS
    #### ========================================================================
    #### Increase follow distance when approaching a stopped car to encourage earlier braking ONLY when approaching
    # a stopped car from an approaching speed greater than 60 kph (16.5 m/s).
    if self.frogpilot_planner.tracking_lead and self.frogpilot_planner.lead_one.vLead < 1.0 and v_ego > 16.5:
    #if self.frogpilot_planner.tracking_lead and self.frogpilot_planner.lead_one.vLead < 1.0:
      # This multiplier INCREASES t_follow. A larger value means an earlier reaction.
      # 1.3 = 30% increase in desired follow time
      # 1.5 = 50% increase in desired follow time
      stopped_car_factor = 2 #1.5
      self.t_follow *= stopped_car_factor

      # Define the distance breakpoints and the corresponding follow time multipliers.
      # At 40m or less, the multiplier is 2.0x (maximum effect).
      # At 150m or more, the multiplier is 1.0x (no effect).
      # The effect scales linearly between these two points.
      ##DISTANCE_BP = [50, 150]  # meters
      ##MULTIPLIER_VP = [2.0, 1.0] # factor
      # Get the current distance to the lead car.
      ##d_rel = self.frogpilot_planner.lead_one.dRel
      # Calculate the proportional multiplier using interpolation.
      ##stopped_car_factor = np.interp(d_rel, DISTANCE_BP, MULTIPLIER_VP)
      # Apply the calculated multiplier to the follow time.
      ##self.t_follow *= stopped_car_factor

    #### ========================================================================
    #### END OF Stopped Car LOGIC
    #### ========================================================================    
    
    
    self.acceleration_jerk = self.base_acceleration_jerk
    self.danger_jerk = self.base_danger_jerk
    self.speed_jerk = self.base_speed_jerk

    self.following_lead = self.frogpilot_planner.tracking_lead and self.frogpilot_planner.lead_one.dRel < (self.t_follow + 1) * v_ego

    if sm["controlsState"].enabled and self.frogpilot_planner.tracking_lead:
      self.update_follow_values(self.frogpilot_planner.lead_one.dRel, v_ego, self.frogpilot_planner.lead_one.vLead, frogpilot_toggles)
      self.desired_follow_distance = int(desired_follow_distance(v_ego, self.frogpilot_planner.lead_one.vLead, self.t_follow))
    else:
      self.desired_follow_distance = 0

  def update_follow_values(self, lead_distance, v_ego, v_lead, frogpilot_toggles):
    # Offset by FrogAi for FrogPilot for a more natural approach to a faster lead
    if frogpilot_toggles.human_following and v_lead > v_ego:
      distance_factor = max(lead_distance - (v_ego * self.t_follow), 1)
      accelerating_offset = float(np.clip(STOP_DISTANCE - v_ego, 1, distance_factor))

      self.acceleration_jerk /= accelerating_offset
      self.speed_jerk /= accelerating_offset
      self.t_follow /= accelerating_offset

    # Offset by FrogAi for FrogPilot for a more natural approach to a slower lead
    if (frogpilot_toggles.conditional_slower_lead or frogpilot_toggles.human_following) and v_lead < v_ego:

      ################################   ADD softer breaking ###############################
      # 1. Define how close to get before braking, a buffer time proportional to our speed (in seconds)
      CoastingBufferTime = 0.3 # (0.35 is almost 8 meters @ 80KPH)
      # 2. Define how gradual the approach is (0.1=very broad, 1.0=original)
      RampBroadness = 0.5
      
      # 3. Calculate the dynamic buffer distance based on our speed
      dynamic_coasting_buffer = v_ego * CoastingBufferTime
      # 4. Calculate the modified distance_factor with the dynamic buffer
      base_distance = lead_distance - ((v_lead * self.t_follow) - dynamic_coasting_buffer)
      
      distance_factor = max(1, base_distance * RampBroadness)
      braking_offset = float(np.clip(min(v_ego - v_lead, v_lead) - COMFORT_BRAKE, 1, distance_factor))

      if frogpilot_toggles.human_following:
        if not self.following_lead and v_lead > CITY_SPEED_LIMIT:
          far_lead_offset = max(lead_distance - (v_ego * self.t_follow) - STOP_DISTANCE, 0)
        else:
          far_lead_offset = 0
        self.t_follow /= braking_offset + far_lead_offset
      self.slower_lead = braking_offset > 1

