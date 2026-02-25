"""
Unit tests for standstill creep-to-gap behavior in LongControl.
"""
from types import SimpleNamespace
import unittest

from cereal import car
from openpilot.selfdrive.car.toyota.values import CAR as TOYOTA
from openpilot.selfdrive.car.car_helpers import interfaces
from openpilot.selfdrive.controls.lib.longcontrol import (
  LongControl, LongCtrlState, CREEP_ACCEL, CREEP_GAP_TARGET, CREEP_GAP_DEADBAND, CREEP_MAX_SPEED
)


def make_toggles(**kwargs):
  defaults = dict(
    vEgoStopping=0.25,
    vEgoStarting=0.5,
    stopAccel=-0.2,
    stoppingDecelRate=0.3,
    startAccel=0.8,
    human_acceleration=False,
  )
  defaults.update(kwargs)
  return SimpleNamespace(**defaults)


def make_CS(v_ego=0.0, brake_pressed=False, a_ego=0.0, standstill=False):
  CS = car.CarState.new_message()
  CS.vEgo = v_ego
  CS.aEgo = a_ego
  CS.brakePressed = brake_pressed
  CS.cruiseState.standstill = standstill
  return CS


def make_lead(status=True, d_rel=5.0):
  """Return a minimal lead-car object matching the radarState.leadOne interface."""
  return SimpleNamespace(status=status, dRel=d_rel)


def make_long_control():
  CarInterface, CarController, CarState = interfaces[TOYOTA.TOYOTA_COROLLA]
  CP = CarInterface.get_non_essential_params(TOYOTA.TOYOTA_COROLLA)
  return LongControl(CP)


class TestStandstillCreep(unittest.TestCase):

  def _put_in_stopping(self, lc, toggles):
    """Drive the state machine into LongCtrlState.stopping."""
    CS = make_CS(v_ego=0.0)
    # should_stop=True drives the state into stopping
    lc.update(True, CS, 0.0, True, [-3.0, 2.0], toggles)
    self.assertEqual(lc.long_control_state, LongCtrlState.stopping)

  def test_creep_activates_when_standstill_and_gap_exceeds_target(self):
    """Creep acceleration is commanded when stopped and lead gap > target + deadband."""
    lc = make_long_control()
    toggles = make_toggles()
    self._put_in_stopping(lc, toggles)

    CS = make_CS(v_ego=0.0)
    lead = make_lead(status=True, d_rel=CREEP_GAP_TARGET + CREEP_GAP_DEADBAND + 0.5)
    accel = lc.update(True, CS, 0.0, True, [-3.0, 2.0], toggles, lead_one=lead)
    self.assertGreater(accel, 0.0, "Expected positive creep acceleration")
    self.assertAlmostEqual(accel, CREEP_ACCEL, places=5)

  def test_creep_deactivates_within_deadband(self):
    """No creep when lead gap is within the deadband of the target."""
    lc = make_long_control()
    toggles = make_toggles()
    self._put_in_stopping(lc, toggles)

    CS = make_CS(v_ego=0.0)
    # Gap exactly at target (inside deadband — should not creep)
    lead = make_lead(status=True, d_rel=CREEP_GAP_TARGET)
    accel = lc.update(True, CS, 0.0, True, [-3.0, 2.0], toggles, lead_one=lead)
    self.assertLessEqual(accel, 0.0, "Expected no creep within deadband")

  def test_creep_disabled_when_brake_pressed(self):
    """No creep when the driver is pressing the brake."""
    lc = make_long_control()
    toggles = make_toggles()
    self._put_in_stopping(lc, toggles)

    CS = make_CS(v_ego=0.0, brake_pressed=True)
    lead = make_lead(status=True, d_rel=CREEP_GAP_TARGET + CREEP_GAP_DEADBAND + 1.0)
    accel = lc.update(True, CS, 0.0, True, [-3.0, 2.0], toggles, lead_one=lead)
    self.assertLessEqual(accel, 0.0, "Expected no creep when brake pressed")

  def test_creep_disabled_when_no_lead(self):
    """No creep when lead_one is None."""
    lc = make_long_control()
    toggles = make_toggles()
    self._put_in_stopping(lc, toggles)

    CS = make_CS(v_ego=0.0)
    accel = lc.update(True, CS, 0.0, True, [-3.0, 2.0], toggles, lead_one=None)
    self.assertLessEqual(accel, 0.0, "Expected no creep without lead")

  def test_creep_disabled_when_lead_not_valid(self):
    """No creep when lead status is False."""
    lc = make_long_control()
    toggles = make_toggles()
    self._put_in_stopping(lc, toggles)

    CS = make_CS(v_ego=0.0)
    lead = make_lead(status=False, d_rel=10.0)
    accel = lc.update(True, CS, 0.0, True, [-3.0, 2.0], toggles, lead_one=lead)
    self.assertLessEqual(accel, 0.0, "Expected no creep when lead.status is False")

  def test_creep_disabled_at_higher_speed(self):
    """No creep when ego speed is at or above CREEP_MAX_SPEED."""
    lc = make_long_control()
    toggles = make_toggles()
    self._put_in_stopping(lc, toggles)

    CS = make_CS(v_ego=CREEP_MAX_SPEED)
    lead = make_lead(status=True, d_rel=CREEP_GAP_TARGET + CREEP_GAP_DEADBAND + 1.0)
    accel = lc.update(True, CS, 0.0, True, [-3.0, 2.0], toggles, lead_one=lead)
    self.assertLessEqual(accel, 0.0, "Expected no creep at or above CREEP_MAX_SPEED")


if __name__ == "__main__":
  unittest.main()
