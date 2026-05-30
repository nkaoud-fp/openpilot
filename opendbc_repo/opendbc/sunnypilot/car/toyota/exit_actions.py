"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.can_definitions import CanData

GearShifter = structs.CarState.GearShifter

# KWP2000 door lock command to BCM at 0x750 on bus 0.
# Credit: AlexandreSato (FrogPilot)
_LOCK_CMD = b"\x40\x05\x30\x11\x00\x80\x00\x00"
_BCM_ADDR = 0x750
_BCM_BUS  = 0


class ExitActionsController:
  """
  Sends a single door-lock CAN frame when the driver unlatches the seatbelt
  while parked at standstill (onroad trigger for the 'stepping out' case).

  The offroad case (ignition off) is handled separately by
  sunnypilot/selfdrive/pandad/toyota_exit_actions.py, which also covers
  window close and mirror fold via direct Panda with SAFETY_ALLOUTPUT.
  """

  def __init__(self):
    self._prev_seatbelt_unlatched: bool = False
    self._prev_gear_in_park: bool = True
    self._locked_on_exit: bool = False

  def update(self, CS: structs.CarState, auto_lock: bool) -> list[CanData]:
    can_sends: list[CanData] = []

    in_park = CS.gearShifter == GearShifter.park
    at_rest = CS.vEgo < 0.3          # m/s ≈ 1 km/h
    seatbelt_off = CS.seatbeltUnlatched

    # Seatbelt just unlatched while parked and stationary → lock once
    seatbelt_just_released = seatbelt_off and not self._prev_seatbelt_unlatched
    if auto_lock and in_park and at_rest and seatbelt_just_released and not self._locked_on_exit:
      can_sends.append(CanData(address=_BCM_ADDR, dat=_LOCK_CMD, src=_BCM_BUS))
      self._locked_on_exit = True

    # Gear leaving park re-arms the controller for the next cycle
    if self._prev_gear_in_park and not in_park:
      self._locked_on_exit = False

    self._prev_seatbelt_unlatched = seatbelt_off
    self._prev_gear_in_park = in_park

    return can_sends
