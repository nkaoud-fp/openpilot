"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.can_definitions import CanData

GearShifter = structs.CarState.GearShifter

# KWP2000 door lock/unlock commands to BCM at 0x750 on bus 0.
# Credit: AlexandreSato (FrogPilot)
_LOCK_CMD   = b"\x40\x05\x30\x11\x00\x80\x00\x00"
_UNLOCK_CMD = b"\x40\x05\x30\x11\x00\x40\x00\x00"
_BCM_ADDR   = 0x750
_BCM_BUS    = 0


def _make_bcm_msg(data: bytes) -> CanData:
  return CanData(address=_BCM_ADDR, dat=data, src=_BCM_BUS)


class ExitActionsController:
  """
  Sends a single door-lock command when the driver exits (seatbelt unlatches
  while parked at standstill) and a single unlock command when the driver
  re-enters (seatbelt latches while parked).

  Only one CAN frame is sent per transition; the BCM latches the new state.
  """

  def __init__(self):
    self._prev_seatbelt_unlatched: bool = False
    self._prev_gear_in_park: bool = True
    # True once we've locked after an exit, cleared on next entry
    self._locked_on_exit: bool = False

  def update(self, CS: structs.CarState, auto_lock: bool, auto_unlock: bool) -> list[CanData]:
    can_sends: list[CanData] = []

    in_park = CS.gearShifter == GearShifter.park
    at_rest = CS.vEgo < 0.3          # m/s ≈ 1 km/h
    seatbelt_off = CS.seatbeltUnlatched

    # --- exit detection: seatbelt just unlatched while parked and stationary ---
    seatbelt_just_released = seatbelt_off and not self._prev_seatbelt_unlatched
    if auto_lock and in_park and at_rest and seatbelt_just_released and not self._locked_on_exit:
      can_sends.append(_make_bcm_msg(_LOCK_CMD))
      self._locked_on_exit = True

    # --- entry detection: seatbelt just latched while parked (driver sat back down) ---
    seatbelt_just_latched = not seatbelt_off and self._prev_seatbelt_unlatched
    if auto_unlock and in_park and seatbelt_just_latched and self._locked_on_exit:
      can_sends.append(_make_bcm_msg(_UNLOCK_CMD))
      self._locked_on_exit = False

    # If gear leaves park, clear the locked flag so the next park+exit re-arms
    if self._prev_gear_in_park and not in_park:
      self._locked_on_exit = False

    self._prev_seatbelt_unlatched = seatbelt_off
    self._prev_gear_in_park = in_park

    return can_sends
