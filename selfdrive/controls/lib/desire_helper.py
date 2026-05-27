import json

from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

# Phantom lead tracking: simulate dropped adjacent radar leads
PHANTOM_LEAD_MIN_TRACK_TIME = 1.5    # seconds: lead must be tracked this long before simulating
PHANTOM_LEAD_MAX_SIM_TIME = 10.0     # seconds: max time to simulate after drop
PHANTOM_LEAD_ALONGSIDE_FRONT = 4.0   # meters: front boundary of "alongside" zone
PHANTOM_LEAD_ALONGSIDE_REAR = -5.0   # meters: rear boundary of "alongside" zone


class PhantomLeadTracker:
  """Tracks adjacent radar leads after they drop off radar.

  When a lead that was tracked for >= 1.5s drops, we snapshot its absolute
  speed and simulate its position relative to us. Each frame we recompute
  vRel from (lead_abs_speed - current vEgo), so our own speed changes
  (braking, accelerating) are reflected in the simulation.
  """
  def __init__(self):
    self.tracking_time = 0.0
    self.prev_status = False

    self.simulating = False
    self.sim_time = 0.0
    self.sim_dRel = 0.0
    self.sim_lead_speed = 0.0  # absolute speed of lead at drop time

  def update(self, lead_status, lead_dRel, lead_vRel, v_ego, dt):
    if lead_status:
      self.tracking_time += dt
      self.simulating = False
      self.sim_time = 0.0

    elif self.prev_status and not lead_status:
      if self.tracking_time >= PHANTOM_LEAD_MIN_TRACK_TIME:
        self.simulating = True
        self.sim_time = 0.0
        self.sim_dRel = lead_dRel
        self.sim_lead_speed = v_ego + lead_vRel
      self.tracking_time = 0.0

    elif self.simulating:
      self.sim_time += dt
      current_vRel = self.sim_lead_speed - v_ego
      self.sim_dRel += current_vRel * dt

      behind_us = self.sim_dRel < PHANTOM_LEAD_ALONGSIDE_REAR
      far_away = abs(self.sim_dRel) > 5.0
      timed_out = self.sim_time >= PHANTOM_LEAD_MAX_SIM_TIME and far_away
      hard_limit = self.sim_time >= 25.0
      if behind_us or timed_out or hard_limit:
        self.simulating = False

    else:
      self.tracking_time = 0.0

    self.prev_status = lead_status

  @property
  def alongside(self):
    """True if the simulated lead is in the alongside zone."""
    return (self.simulating and
            self.sim_dRel <= PHANTOM_LEAD_ALONGSIDE_FRONT and
            self.sim_dRel >= PHANTOM_LEAD_ALONGSIDE_REAR)
NAVIGATION_TEST_ADJACENT_LEAD_MIN_DISTANCE = 10.0 # 30
NAVIGATION_TEST_LANE_CHANGE_TIME_GAP_SECONDS = 1.8
NAVIGATION_TEST_LANE_CHANGE_CLOSING_EXTRA_SECONDS = 2.2
NAVIGATION_TEST_LANE_CHANGE_CONDITION_TIME = 2.0
NAVIGATION_TEST_LANE_CHANGE_COMPLETE_PROB = 0.02
NAVIGATION_TEST_LANE_CHANGE_COMPLETE_TIME = 4.0
NAVIGATION_TEST_LANE_CHANGE_COOLDOWN = 3.0
NAVIGATION_TEST_MAX_LANE_CHANGES = 4

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

NAVIGATION_TEST_DIRECTIONS = {
  "left": TurnDirection.turnLeft,
  "right": TurnDirection.turnRight,
}

NAVIGATION_TEST_LANE_CHANGE_DESIRES = {
  "left": log.Desire.keepLeft,
  "right": log.Desire.keepRight,
}

class DesireHelper:
  def __init__(self):
    self.params = Params()

    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    # Phantom lead trackers for adjacent lanes
    self.phantom_lead_left = PhantomLeadTracker()
    self.phantom_lead_right = PhantomLeadTracker()

    # FrogPilot variables
    self.lane_change_completed = False

    self.lane_change_wait_timer = 0
    self.navigation_test_lane_change_condition_timer = 0.0
    self.navigation_test_lane_change_condition_direction = "none"
    self.navigation_test_lane_change_active = False
    self.navigation_test_lane_change_timer = 0.0
    self.navigation_test_lane_change_cooldown_timer = 0.0
    self.navigation_test_lane_change_count = 0
    self.navigation_test_lane_change_key = None
    self.navigation_test_prep_status = None

  def navigation_test_command(self):
    if not self.params.get_bool("NavigationTestControl"):
      self.reset_navigation_test_lane_change_plan()
      self.update_navigation_test_prep_status("idle")
      return "none", "none", {}

    command = self.params.get("NavigationTestTurnCommand", encoding="utf-8")
    if command is None:
      self.reset_navigation_test_lane_change_plan()
      self.update_navigation_test_prep_status("idle")
      return "none", "none", {}

    try:
      command_json = json.loads(command)
    except json.JSONDecodeError:
      self.reset_navigation_test_lane_change_plan()
      self.update_navigation_test_prep_status("idle")
      return "none", "none", {}

    direction = command_json.get("direction", "none")
    action = command_json.get("action", "turn" if direction in NAVIGATION_TEST_DIRECTIONS else "none")
    if direction not in NAVIGATION_TEST_DIRECTIONS:
      self.reset_navigation_test_lane_change_plan()
      self.update_navigation_test_prep_status("idle")
      return "none", "none", command_json
    return action, direction, command_json

  def reset_navigation_test_lane_change_plan(self):
    self.navigation_test_lane_change_active = False
    self.navigation_test_lane_change_timer = 0.0
    self.navigation_test_lane_change_cooldown_timer = 0.0
    self.navigation_test_lane_change_count = 0
    self.navigation_test_lane_change_key = None
    self.navigation_test_lane_change_condition_timer = 0.0
    self.navigation_test_lane_change_condition_direction = "none"

  def update_navigation_test_prep_status(self, stage, direction="none", max_lane_changes=NAVIGATION_TEST_MAX_LANE_CHANGES, reason="", diagnostics=None):
    status_data = {
      "stage": stage,
      "direction": direction,
      "completedLaneChanges": self.navigation_test_lane_change_count,
      "maxLaneChanges": max_lane_changes,
      "cooldownRemaining": max(self.navigation_test_lane_change_cooldown_timer, 0.0),
      "reason": reason,
    }
    if diagnostics is not None:
      status_data.update(diagnostics)

    status = json.dumps(status_data)
    if status != self.navigation_test_prep_status:
      self.params.put("NavigationTestPrepStatus", status)
      self.navigation_test_prep_status = status

  def navigation_test_lane_available(self, direction, frogpilotPlan, frogpilot_toggles):
    desired_lane_width = frogpilotPlan.laneWidthLeft if direction == "left" else frogpilotPlan.laneWidthRight
    return desired_lane_width >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.lane_detection

  def navigation_test_lane_change_diagnostics(self, direction, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed):
    lane_available = self.navigation_test_lane_available(direction, frogpilotPlan, frogpilot_toggles)
    phantom_left = self.phantom_lead_left.alongside
    phantom_right = self.phantom_lead_right.alongside
    blindspot_detected = ((carstate.leftBlindspot or phantom_left) and direction == "left") or \
                         ((carstate.rightBlindspot or phantom_right) and direction == "right")
    adjacent_lead = frogpilotRadarState.leadLeft if direction == "left" else frogpilotRadarState.leadRight

    lead_distance = float(adjacent_lead.dRel) if adjacent_lead.status else None
    closing_speed = max(float(-adjacent_lead.vRel), 0.0) if adjacent_lead.status else 0.0
    required_gap = max(
      NAVIGATION_TEST_ADJACENT_LEAD_MIN_DISTANCE,
      float(carstate.vEgo) * NAVIGATION_TEST_LANE_CHANGE_TIME_GAP_SECONDS + closing_speed * NAVIGATION_TEST_LANE_CHANGE_CLOSING_EXTRA_SECONDS,
    )
    adjacent_lead_too_close = adjacent_lead.status and adjacent_lead.dRel < required_gap
    lane_changes_enabled = bool(frogpilot_toggles.lane_changes)
    allowed = lane_changes_enabled and not below_lane_change_speed and lane_available and not blindspot_detected and not adjacent_lead_too_close

    return {
      "allowed": allowed,
      "laneAvailable": lane_available,
      "laneChangesEnabled": lane_changes_enabled,
      "belowLaneChangeSpeed": below_lane_change_speed,
      "blindspotDetected": blindspot_detected,
      "adjacentLeadStatus": bool(adjacent_lead.status),
      "adjacentLeadDistance": lead_distance,
      "adjacentLeadClosingSpeed": closing_speed,
      "requiredGap": required_gap,
    }

  def navigation_test_lane_change_allowed(self, direction, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed):
    return self.navigation_test_lane_change_diagnostics(
      direction, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed
    )["allowed"]

  def navigation_test_lane_change_desire_active(self, action, direction, command_json, lateral_active, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed, lane_change_prob):
    if action != "laneChange" or direction not in NAVIGATION_TEST_LANE_CHANGE_DESIRES:
      self.reset_navigation_test_lane_change_plan()
      self.update_navigation_test_prep_status("idle")
      return False

    max_lane_changes = max(1, int(command_json.get("maxLaneChanges", NAVIGATION_TEST_MAX_LANE_CHANGES)))
    cooldown_seconds = max(0.0, float(command_json.get("laneChangeCooldown", NAVIGATION_TEST_LANE_CHANGE_COOLDOWN)))
    migration_start_distance = round(float(command_json.get("migrationStartDistance", 0.0)), 1)
    migration_key = (direction, migration_start_distance)
    if migration_key != self.navigation_test_lane_change_key:
      self.reset_navigation_test_lane_change_plan()
      self.navigation_test_lane_change_key = migration_key

    if self.navigation_test_lane_change_active:
      self.navigation_test_lane_change_timer += DT_MDL
      complete_by_model = self.navigation_test_lane_change_timer >= NAVIGATION_TEST_LANE_CHANGE_COMPLETE_TIME and lane_change_prob < NAVIGATION_TEST_LANE_CHANGE_COMPLETE_PROB
      complete_by_timeout = self.navigation_test_lane_change_timer >= LANE_CHANGE_TIME_MAX
      if complete_by_model or complete_by_timeout:
        self.navigation_test_lane_change_active = False
        self.navigation_test_lane_change_timer = 0.0
        self.navigation_test_lane_change_count += 1
        self.navigation_test_lane_change_cooldown_timer = cooldown_seconds
        self.navigation_test_lane_change_condition_timer = 0.0
        self.navigation_test_lane_change_condition_direction = "none"
        self.update_navigation_test_prep_status("cooldown", direction, max_lane_changes)
        return False

      self.update_navigation_test_prep_status("changing", direction, max_lane_changes)
      return True

    if self.navigation_test_lane_change_count >= max_lane_changes:
      self.update_navigation_test_prep_status("maxLaneChanges", direction, max_lane_changes)
      return False

    if not self.navigation_test_lane_available(direction, frogpilotPlan, frogpilot_toggles):
      self.update_navigation_test_prep_status("edgeReached", direction, max_lane_changes)
      return False

    if self.navigation_test_lane_change_cooldown_timer > 0.0:
      self.navigation_test_lane_change_cooldown_timer = max(self.navigation_test_lane_change_cooldown_timer - DT_MDL, 0.0)
      self.update_navigation_test_prep_status("cooldown", direction, max_lane_changes)
      return False

    diagnostics = self.navigation_test_lane_change_diagnostics(
      direction, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed
    )
    if not diagnostics["allowed"]:
      if not diagnostics["laneChangesEnabled"]:
        reason = "laneChangesDisabled"
      elif diagnostics["belowLaneChangeSpeed"]:
        reason = "belowLaneChangeSpeed"
      elif not diagnostics["laneAvailable"]:
        reason = "noTargetLane"
      elif diagnostics["blindspotDetected"]:
        reason = "blindspot"
      elif diagnostics["adjacentLeadStatus"]:
        reason = "adjacentLeadGap"
      else:
        reason = "unknown"
      self.update_navigation_test_prep_status("blocked", direction, max_lane_changes, reason=reason, diagnostics=diagnostics)
      return False

    stable = self.navigation_test_lane_change_conditions_stable(action, direction, lateral_active, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed)
    if not stable:
      self.update_navigation_test_prep_status("preparing", direction, max_lane_changes, diagnostics=diagnostics)
      return False

    self.navigation_test_lane_change_active = True
    self.navigation_test_lane_change_timer = 0.0
    self.update_navigation_test_prep_status("changing", direction, max_lane_changes, diagnostics=diagnostics)
    return True

  def navigation_test_lane_change_conditions_stable(self, action, direction, lateral_active, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed):
    conditions_met = (
      action == "laneChange" and
      direction in NAVIGATION_TEST_LANE_CHANGE_DESIRES and
      lateral_active and
      not carstate.standstill and
      self.navigation_test_lane_change_allowed(direction, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed)
    )

    if not conditions_met:
      self.navigation_test_lane_change_condition_timer = 0.0
      self.navigation_test_lane_change_condition_direction = "none"
      return False

    if direction != self.navigation_test_lane_change_condition_direction:
      self.navigation_test_lane_change_condition_timer = 0.0
      self.navigation_test_lane_change_condition_direction = direction

    self.navigation_test_lane_change_condition_timer += DT_MDL
    return self.navigation_test_lane_change_condition_timer >= NAVIGATION_TEST_LANE_CHANGE_CONDITION_TIME

  def update(self, carstate, lateral_active, lane_change_prob, frogpilotPlan, frogpilotRadarState, frogpilot_toggles):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < frogpilot_toggles.minimum_lane_change_speed

    # Update phantom lead trackers with current adjacent radar lead data
    lead_left = frogpilotRadarState.leadLeft
    lead_right = frogpilotRadarState.leadRight
    self.phantom_lead_left.update(bool(lead_left.status), float(lead_left.dRel), float(lead_left.vRel), v_ego, DT_MDL)
    self.phantom_lead_right.update(bool(lead_right.status), float(lead_right.dRel), float(lead_right.vRel), v_ego, DT_MDL)

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or not frogpilot_toggles.lane_changes:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_wait_timer = 0.0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        self.lane_change_wait_timer += DT_MDL

        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        if torque_applied:
          self.lane_change_wait_timer = frogpilot_toggles.lane_change_delay
        else:
          desired_lane_width = frogpilotPlan.laneWidthLeft if carstate.leftBlinker else frogpilotPlan.laneWidthRight
          lane_available = desired_lane_width >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.lane_detection
          torque_applied = lane_available and self.lane_change_wait_timer >= frogpilot_toggles.lane_change_delay and frogpilot_toggles.nudgeless

        # Combine BSM radar + phantom lead simulation for blindspot detection
        phantom_left = self.phantom_lead_left.alongside
        phantom_right = self.phantom_lead_right.alongside
        blindspot_detected = (((carstate.leftBlindspot or phantom_left) and self.lane_change_direction == LaneChangeDirection.left) or
                              ((carstate.rightBlindspot or phantom_right) and self.lane_change_direction == LaneChangeDirection.right))

        if not one_blinker or below_lane_change_speed or self.lane_change_completed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif torque_applied and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_completed = frogpilot_toggles.one_lane_change
          self.lane_change_wait_timer = 0.0

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.lane_change_completed &= one_blinker
    self.prev_one_blinker = one_blinker

    navigation_test_action, navigation_test_direction, navigation_test_command = self.navigation_test_command()
    navigation_test_turn_direction = NAVIGATION_TEST_DIRECTIONS.get(navigation_test_direction, TurnDirection.none) if navigation_test_action == "turn" else TurnDirection.none
    navigation_test_lane_change_desire = NAVIGATION_TEST_LANE_CHANGE_DESIRES.get(navigation_test_direction, log.Desire.none) if navigation_test_action == "laneChange" else log.Desire.none
    navigation_test_lane_change_active = navigation_test_lane_change_desire != log.Desire.none
    navigation_test_lane_change_active &= self.navigation_test_lane_change_desire_active(navigation_test_action, navigation_test_direction, navigation_test_command, lateral_active, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed, lane_change_prob)
    navigation_test_turn_active = navigation_test_turn_direction != TurnDirection.none and lateral_active and not carstate.standstill

    if navigation_test_lane_change_active:
      self.turn_direction = TurnDirection.none
      self.desire = navigation_test_lane_change_desire
    elif navigation_test_turn_active:
      self.turn_direction = navigation_test_turn_direction
      self.desire = TURN_DESIRES[self.turn_direction]
    elif one_blinker and below_lane_change_speed and not carstate.standstill and frogpilot_toggles.use_turn_desires:
      self.turn_direction = TurnDirection.turnLeft if carstate.leftBlinker else TurnDirection.turnRight
      self.desire = TURN_DESIRES[self.turn_direction]
    else:
      self.turn_direction = TurnDirection.none
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
