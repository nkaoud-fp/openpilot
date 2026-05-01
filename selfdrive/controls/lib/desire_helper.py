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
NAVIGATION_TEST_ADJACENT_LEAD_MIN_DISTANCE = 30.0
NAVIGATION_TEST_LANE_CHANGE_CONDITION_TIME = 2.0

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
  "left": log.Desire.laneChangeLeft,
  "right": log.Desire.laneChangeRight,
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

    # FrogPilot variables
    self.lane_change_completed = False

    self.lane_change_wait_timer = 0
    self.navigation_test_lane_change_condition_timer = 0.0
    self.navigation_test_lane_change_condition_direction = "none"

  def navigation_test_command(self):
    if not self.params.get_bool("NavigationTestControl"):
      return "none", "none"

    command = self.params.get("NavigationTestTurnCommand", encoding="utf-8")
    if command is None:
      return "none", "none"

    try:
      command_json = json.loads(command)
    except json.JSONDecodeError:
      return "none", "none"

    direction = command_json.get("direction", "none")
    action = command_json.get("action", "turn" if direction in NAVIGATION_TEST_DIRECTIONS else "none")
    if direction not in NAVIGATION_TEST_DIRECTIONS:
      return "none", "none"
    return action, direction

  def navigation_test_lane_change_allowed(self, direction, carstate, frogpilotPlan, frogpilotRadarState, frogpilot_toggles, below_lane_change_speed):
    if below_lane_change_speed or not frogpilot_toggles.lane_changes:
      return False

    desired_lane_width = frogpilotPlan.laneWidthLeft if direction == "left" else frogpilotPlan.laneWidthRight
    lane_available = desired_lane_width >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.lane_detection
    blindspot_detected = (carstate.leftBlindspot and direction == "left") or (carstate.rightBlindspot and direction == "right")
    adjacent_lead = frogpilotRadarState.leadLeft if direction == "left" else frogpilotRadarState.leadRight
    adjacent_lead_too_close = adjacent_lead.status and adjacent_lead.dRel < NAVIGATION_TEST_ADJACENT_LEAD_MIN_DISTANCE
    return lane_available and not blindspot_detected and not adjacent_lead_too_close

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

    # navd may request a lane change by writing NavigationTestForceBlinker.
    # OR it into the blinker view used by the FSM (carState itself is not
    # modified). The FSM then drives the lane change through the same path
    # as a manual blinker, so the model receives the desire pulse with the
    # blinker context it was trained on.
    forced_blinker = self.params.get("NavigationTestForceBlinker", encoding="utf-8") or "none"
    left_blinker = carstate.leftBlinker or (forced_blinker == "left")
    right_blinker = carstate.rightBlinker or (forced_blinker == "right")
    one_blinker = left_blinker != right_blinker
    below_lane_change_speed = v_ego < frogpilot_toggles.minimum_lane_change_speed

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
          left_blinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        if torque_applied:
          self.lane_change_wait_timer = frogpilot_toggles.lane_change_delay
        else:
          desired_lane_width = frogpilotPlan.laneWidthLeft if left_blinker else frogpilotPlan.laneWidthRight
          lane_available = desired_lane_width >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.lane_detection
          torque_applied = lane_available and self.lane_change_wait_timer >= frogpilot_toggles.lane_change_delay and frogpilot_toggles.nudgeless

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

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

    # Lane changes requested by navd are now driven via NavigationTestForceBlinker,
    # which feeds the FSM at the top of this update(); the FSM then produces the
    # desire through the standard DESIRES table below. The direct-desire branch
    # was unreliable because the model needs the blinker context it was trained on.
    navigation_test_action, navigation_test_direction = self.navigation_test_command()
    navigation_test_turn_direction = NAVIGATION_TEST_DIRECTIONS.get(navigation_test_direction, TurnDirection.none) if navigation_test_action == "turn" else TurnDirection.none
    navigation_test_turn_active = navigation_test_turn_direction != TurnDirection.none and lateral_active and not carstate.standstill

    if navigation_test_turn_active:
      self.turn_direction = navigation_test_turn_direction
      self.desire = TURN_DESIRES[self.turn_direction]
    elif one_blinker and below_lane_change_speed and not carstate.standstill and frogpilot_toggles.use_turn_desires:
      self.turn_direction = TurnDirection.turnLeft if left_blinker else TurnDirection.turnRight
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
