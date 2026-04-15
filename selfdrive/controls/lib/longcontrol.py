from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState

# Standstill creep-to-gap constants (bumper-to-bumper follow at stops)
CREEP_GAP_TARGET = 5.5 #3.5 # 2.5    # meters - desired stopped gap to lead
CREEP_GAP_DEADBAND = 0.1 #0.3 #0.1  # meters - hysteresis to avoid oscillation (dflt 0.3)
CREEP_ACCEL = 0.10 #0.08 #0.3         # m/s^2 - gentle creep acceleration cap  (0.3 default)
CREEP_MAX_SPEED = 0.5 #0.25 #2.5 #0.5 # m/s  - only creep below this ego speed (2 m/s ~ 7 kph) (4 m/s ~ 14 kph)


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill, frogpilot_toggles):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > frogpilot_toggles.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

def long_control_state_trans_old_long(CP, active, long_control_state, v_ego, v_target,
                                      v_target_1sec, brake_pressed, cruise_standstill, frogpilot_toggles):
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < frogpilot_toggles.vEgoStopping and
                  v_target_1sec < frogpilot_toggles.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < frogpilot_toggles.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > frogpilot_toggles.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > frogpilot_toggles.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, frogpilot_toggles, lead_one=None):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill, frogpilot_toggles)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping and 1 == 1:   ###### 1 == 2 todisable the block for testing removing creep
      output_accel = self.last_output_accel
      
      # --- Standstill Creep-to-Gap Logic ---
      #CREEP_GAP_TARGET: The exact distance you want your car to stop behind the bumper of the car in front of you (e.g., 1.5 meters).
      #CREEP_GAP_DEADBAND: The extra distance the lead car needs to pull away before your car wakes up and starts creeping, preventing stuttering (e.g., 0.3 meters).
      #CREEP_ACCEL: The absolute hardest the car is allowed to press the gas pedal while inching forward (e.g., 0.3 m/s²).
      #CREEP_MAX_SPEED: The strict speed limit your car is allowed to reach while in creeping mode (e.g., 0.5 m/s, which is about 1 mph).
      #
      #
      #
      distance_error = 0.0
      if lead_one is not None and lead_one.status:
        distance_error = lead_one.dRel - CREEP_GAP_TARGET

      if not CS.brakePressed and distance_error > CREEP_GAP_DEADBAND:
        # 1. Proportional acceleration: farther away = more acceleration (capped at CREEP_ACCEL)
        target_accel = clip(distance_error * 0.15, 0.0, CREEP_ACCEL)

        # 2. Taper off acceleration as we approach CREEP_MAX_SPEED to prevent speed oscillation
        speed_multiplier = clip(1.0 - (CS.vEgo / max(CREEP_MAX_SPEED, 0.01)), 0.0, 1.0)
        target_accel *= speed_multiplier

        # 3. Break stiction: Use startAccel briefly if completely stopped to release brakes
        ####################################################################################
        ####################################################################################
        ####################################################################################
        if CS.vEgo < 0.05:
          target_accel = max(target_accel, frogpilot_toggles.startAccel)

        # 4. Smoothly ramp up output_accel to prevent jerky throttle application
        jerk_limit = 1.0 * DT_CTRL  # Max 1.0 m/s^2/s jerk rate
        if output_accel < target_accel:
          output_accel += min(target_accel - output_accel, jerk_limit)
        else:
          output_accel -= min(output_accel - target_accel, jerk_limit)

      else:
        # Standard stopping / creep cancellation logic
        if distance_error < 0.0:
          # We are closer than the target gap! Brake actively!
          # The closer we get, the harder we brake.
          active_brake = clip(distance_error * 0.5, frogpilot_toggles.stopAccel, 0.0) 
          # Rapidly step down to the active brake value
          if output_accel > active_brake:
            output_accel -= (frogpilot_toggles.stoppingDecelRate * 3.0) * DT_CTRL
            output_accel = max(output_accel, active_brake)
        elif output_accel > frogpilot_toggles.stopAccel:
          # Bleed off acceleration smoothly instead of snapping immediately to 0.0
          output_accel -= frogpilot_toggles.stoppingDecelRate * DT_CTRL
          output_accel = max(output_accel, frogpilot_toggles.stopAccel)
      # ---------------------------------------------------------
      #else:
        ## Standard stopping / creep cancellation logic
        #if output_accel > frogpilot_toggles.stopAccel:
          ## Bleed off acceleration smoothly instead of snapping immediately to 0.0
          #output_accel -= frogpilot_toggles.stoppingDecelRate * DT_CTRL
          #output_accel = max(output_accel, frogpilot_toggles.stopAccel)

      self.reset()

    elif self.long_control_state == LongCtrlState.stopping and 1 == 2:
      output_accel = self.last_output_accel
      if output_accel > frogpilot_toggles.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= frogpilot_toggles.stoppingDecelRate * DT_CTRL
      self.reset()

    
    elif self.long_control_state == LongCtrlState.starting:
      output_accel = (a_target if frogpilot_toggles.human_acceleration else frogpilot_toggles.startAccel)
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

  def reset_old_long(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update_old_long(self, active, CS, long_plan, accel_limits, t_since_plan, frogpilot_toggles):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target_now = interp(t_since_plan, CONTROL_N_T_IDX, long_plan.accels)

      v_target = interp(frogpilot_toggles.longitudinalActuatorDelay + t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target = 2 * (v_target - v_target_now) / frogpilot_toggles.longitudinalActuatorDelay - a_target_now

      v_target_1sec = interp(frogpilot_toggles.longitudinalActuatorDelay + t_since_plan + 1.0, CONTROL_N_T_IDX, speeds)
    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel
    self.long_control_state = long_control_state_trans_old_long(self.CP, active, self.long_control_state, CS.vEgo,
                                                                v_target, v_target_1sec, CS.brakePressed,
                                                                CS.cruiseState.standstill, frogpilot_toggles)

    if self.long_control_state == LongCtrlState.off:
      self.reset_old_long(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      if output_accel > frogpilot_toggles.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= frogpilot_toggles.stoppingDecelRate * DT_CTRL
      self.reset_old_long(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = frogpilot_toggles.startAccel
      self.reset_old_long(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel
