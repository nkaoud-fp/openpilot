#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_UNSET, CONTROL_N, get_accel_from_plan
from openpilot.common.swaglog import cloudlog

from openpilot.frogpilot.common.frogpilot_variables import MINIMUM_LATERAL_ACCELERATION

# ==========================================================
# --- USER TUNING PARAMETERS FOR DYNAMIC SOFT BRAKING ---
# ==========================================================
USER_TUNING = {
    "distance_buffer": 2.5,        #5.0 [3.0 - 8.0] Meters left behind lead car bumper in physics calculation
    "base_step_slow": 0.038,       # [0.02 - 0.05] Decel ramp step (m/s^2) for slow/no-lead stops
    "base_step_fast": 0.10,         # 0.15 [0.10 - 0.20] Decel ramp step (m/s^2) for high closing speeds
    "boost_multiplier": 4.0,       # 4.0 [2.0 - 6.0] Proportional gain for emergency catch-up
    "boost_ceiling": 0.08,         # [0.05 - 0.12] Max extra ramp step allowed per frame in emergencies
    "baseline_cap": -1.7,          # [-1.2 to -2.0] Default decel limit before softening kicks in
    "hard_limit_cap": -4.0,        # [-3.5 to -4.5] Absolute max braking force allowed in a panic stop
    "recovery_step": 0.008         # [0.005 - 0.02] How quickly the cap resets to baseline
}
# ==========================================================


def get_soft_braking_tuning(frogpilot_toggles):
  if getattr(frogpilot_toggles, "soft_experimental_mode_braking", False):
    return {
      "distance_buffer": getattr(frogpilot_toggles, "soft_experimental_distance_buffer", USER_TUNING["distance_buffer"]),
      "base_step_slow": getattr(frogpilot_toggles, "soft_experimental_base_step_slow", USER_TUNING["base_step_slow"]),
      "base_step_fast": getattr(frogpilot_toggles, "soft_experimental_base_step_fast", USER_TUNING["base_step_fast"]),
      "boost_multiplier": USER_TUNING["boost_multiplier"],
      "boost_ceiling": USER_TUNING["boost_ceiling"],
      "baseline_cap": getattr(frogpilot_toggles, "soft_experimental_baseline_cap", USER_TUNING["baseline_cap"]),
      "hard_limit_cap": USER_TUNING["hard_limit_cap"],
      "recovery_step": USER_TUNING["recovery_step"],
    }
  return USER_TUNING

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
EXP_MODEL_DECEL_BLEND = 1.0 ### weight for model decel; remainder comes from MPC

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)

  if abs(a_y) > MINIMUM_LATERAL_ACCELERATION:
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
  else:
    a_x_allowed = a_target[1]

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.mpc.mode = 'acc'
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False
    
    ### Initialize Dynamic Experimental-mode decel softening V2 
    self.dynamic_model_decel_cap = USER_TUNING["baseline_cap"] 

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.params_memory = Params("/dev/shm/params")

  @staticmethod
  def parse_model(model_msg, v_ego, taco_tune):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))

    if taco_tune:
      max_lat_accel = np.interp(v_ego, [5, 10, 20], [1.5, 2.0, 3.0])
      curvatures = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z) / np.clip(v, 0.3, 100.0)
      max_v = np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0
      v = np.minimum(max_v, v)

    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm, classic_longitudinal, frogpilot_toggles):
    soft_braking_tuning = get_soft_braking_tuning(frogpilot_toggles)
    mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'
    if classic_longitudinal:
      self.mpc.mode = mode

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise = sm['frogpilotPlan'].vCruise
    v_cruise_initialized = sm['controlsState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled
    reset_state = reset_state or not v_cruise_initialized

    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if mode == 'acc':
      accel_clip = [sm['frogpilotPlan'].minAcceleration, sm['frogpilotPlan'].maxAcceleration]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      if not sm['frogpilotPlan'].cscControllingSpeed:
        accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])
      self.dynamic_model_decel_cap = soft_braking_tuning["baseline_cap"] ############# added to reset CAP

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'], v_ego, frogpilot_toggles.taco_tune)
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(sm['frogpilotPlan'].accelerationJerk, sm['frogpilotPlan'].dangerJerk, sm['frogpilotPlan'].speedJerk, prev_accel_constraint, personality=sm['controlsState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, x, v, a, j, sm['frogpilotPlan'].tFollow, frogpilot_toggles, personality=sm['controlsState'].personality)

    self.a_desired_trajectory_full = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t = frogpilot_toggles.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=frogpilot_toggles.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    required_decel = 0.0  # default; overwritten in experimental mode
    if mode == 'acc':
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else: 
      ### ----------- Dynamic Experimental-mode decel softening V2 ---------------####   
      
      has_lead = sm['radarState'].leadOne.status
      if has_lead:
        v_rel = sm['radarState'].leadOne.vRel
        d_rel = sm['radarState'].leadOne.dRel
      else:
        v_rel = 0.0
        d_rel = 100.0

      # 1. Base dynamic ramp step based on closing speed
      dynamic_ramp_step = float(np.interp(v_rel, [-10.0, 0.0], [soft_braking_tuning["base_step_fast"], soft_braking_tuning["base_step_slow"]]))

      # 2. Kinematic Safety Check
      required_decel = 0.0
      if v_rel < -0.5 and d_rel > soft_braking_tuning["distance_buffer"]:
        required_decel = -((v_rel ** 2) / (2 * max(d_rel - soft_braking_tuning["distance_buffer"], 1.0)))

      # 3. Ramp-up logic with dt-scaled accelerated stepping
      if output_a_target_e2e < self.dynamic_model_decel_cap:
        
        # SMART BOOST: If physics requires harder braking, accelerate the ramp step safely
        if required_decel < self.dynamic_model_decel_cap:
          decel_deficit = self.dynamic_model_decel_cap - required_decel
          boost = np.clip(decel_deficit * self.dt * soft_braking_tuning["boost_multiplier"], 0.0, soft_braking_tuning["boost_ceiling"])
          dynamic_ramp_step += boost

        # Step down using the dynamic step
        self.dynamic_model_decel_cap -= dynamic_ramp_step 
        self.dynamic_model_decel_cap = max(self.dynamic_model_decel_cap, soft_braking_tuning["hard_limit_cap"])
        
      else:
        # If the model eases up, reset the cap back to the baseline
        self.dynamic_model_decel_cap += soft_braking_tuning["recovery_step"]
        self.dynamic_model_decel_cap = min(self.dynamic_model_decel_cap, soft_braking_tuning["baseline_cap"]) 

      # 4. Cap the model using the new dynamic cap
      model_a = max(output_a_target_e2e, self.dynamic_model_decel_cap)      
      
      # 5. Blend the capped model with the MPC
      blended_model_a = EXP_MODEL_DECEL_BLEND * model_a + (1.0 - EXP_MODEL_DECEL_BLEND) * output_a_target_mpc      
      
      # 6. Final output
      output_a_target = max(min(output_a_target_mpc, blended_model_a), self.dynamic_model_decel_cap)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      ### ----------- Dynamic Experimental-mode decel softening  ---------------####

      ### ----------- Experimental-mode speed assertiveness ---------------####
      # When the user sets vCruise high and lets e2e manage speed, the model can
      # coast indefinitely. Optionally apply a small positive accel floor when
      # conditions are clearly safe. vCruise is only used as a ceiling guard.
      assertiveness_mode = getattr(frogpilot_toggles, "experimental_speed_assertiveness", 0)
      if (assertiveness_mode > 0 and self.allow_throttle and not self.output_should_stop
          and not force_slow_decel and v_cruise_initialized and v_ego < v_cruise):
        # Lane-context gate: require multi-lane road and usable confidence.
        total_lanes = int(getattr(sm['frogpilotPlan'], "totalLanes", 0))
        current_lane = int(getattr(sm['frogpilotPlan'], "currentLane", 0))
        lane_confidence = str(getattr(sm['frogpilotPlan'], "laneConfidence", "unknown"))
        lane_gate_ok = total_lanes >= 2 and current_lane >= 1 and lane_confidence in ("medium", "high")

        forecast_ok = False
        if assertiveness_mode in (1, 3) and len(v) > 0:
          v_far = float(v[-1])
          forecast_ok = (v_far - v_ego) > 1.0

        no_lead_ok = False
        if assertiveness_mode in (2, 3):
          if has_lead:
            no_lead_ok = d_rel > 60.0 and v_rel >= -0.5
          else:
            no_lead_ok = True

        if assertiveness_mode == 1:
          apply_floor = forecast_ok
        elif assertiveness_mode == 2:
          apply_floor = no_lead_ok
        else:
          apply_floor = forecast_ok and no_lead_ok

        if apply_floor and lane_gate_ok:
          # Lane-count multiplier: more lanes -> higher confidence we're on a highway.
          lane_count_mult = float(np.interp(total_lanes, [2, 3, 4], [0.5, 0.8, 1.0]))
          # Lane-position multiplier: leftmost = full, rightmost = half.
          if total_lanes > 1:
            pos_frac = (current_lane - 1) / (total_lanes - 1)
          else:
            pos_frac = 0.0
          lane_pos_mult = 1.0 - 0.5 * float(np.clip(pos_frac, 0.0, 1.0))
          # Confidence multiplier.
          conf_mult = 1.0 if lane_confidence == "high" else 0.7

          headroom = max(v_cruise - v_ego, 0.0)
          decay = float(np.clip(headroom / max(frogpilot_toggles.experimental_assert_headroom, 0.1), 0.0, 1.0))
          effective_floor = (frogpilot_toggles.experimental_accel_floor
                             * decay * lane_count_mult * lane_pos_mult * conf_mult)
          output_a_target = max(output_a_target, min(effective_floor, ACCEL_MAX))
      ### ----------- End speed assertiveness ---------------####

    # Publish debug telemetry for the onroad graph overlay
    _pub_cap = self.dynamic_model_decel_cap if mode != 'acc' else 0.0
    self.params_memory.put("LongDebugData",
        f"{_pub_cap:.3f},{required_decel:.3f},{output_a_target_mpc:.3f},{output_a_target_e2e:.3f},{output_a_target:.3f}")

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)
