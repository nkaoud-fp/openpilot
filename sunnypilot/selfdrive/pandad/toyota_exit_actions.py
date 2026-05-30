"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Offroad exit actions for Toyota/Lexus vehicles.
Called once after ignition off (C++ pandad exits) to lock doors,
fold mirrors, and close windows via the Body Control Module (0x750).

Commands (KWP2000 to BCM) sourced from FrogPilot / AlexandreSato.
Mirror fold and window close require SAFETY_ALLOUTPUT because Toyota
safety mode only permits a subset of 0x750 service IDs.
"""

import time

from cereal import car, messaging
from panda import Panda
from openpilot.common.swaglog import cloudlog

# BCM address and bus (same for all Toyota/Lexus)
_BCM_ADDR = 0x750
_BCM_BUS  = 0

# Door lock (allowed under SAFETY_TOYOTA)
_LOCK_CMD          = b"\x40\x05\x30\x11\x00\x80\x00\x00"

# Mirror fold – left and right (requires SAFETY_ALLOUTPUT)
_MIRR_FOLD_L       = b"\xA6\x04\x30\x21\x00\x08\x00\x00"
_MIRR_FOLD_R       = b"\xA5\x04\x30\x21\x00\x08\x00\x00"

# Window close – FL, FR, RL, RR (requires SAFETY_ALLOUTPUT)
_WINDOW_CLOSE_FL   = b"\x90\x04\x30\x01\x05\x20\x00\x00"
_WINDOW_CLOSE_FR   = b"\x91\x04\x30\x01\x05\x20\x00\x00"
_WINDOW_CLOSE_RL   = b"\x93\x04\x30\x01\x05\x20\x00\x00"
_WINDOW_CLOSE_RR   = b"\x92\x04\x30\x01\x05\x20\x00\x00"

_CMD_DELAY = 0.15  # seconds between commands, same as FrogPilot


def _is_toyota(params) -> bool:
  """Return True if the last fingerprinted car was a Toyota/Lexus."""
  try:
    bundle = params.get("CarPlatformBundle")
    if bundle and bundle.get("brand") == "toyota":
      return True
    cp_bytes = params.get("CarParamsPersistent")
    if cp_bytes:
      CP = messaging.log_from_bytes(cp_bytes, car.CarParams)
      return CP.brand == "toyota"
  except Exception:
    pass
  return False


def run_toyota_exit_actions(panda_serial: str, params) -> None:
  """
  Run after ignition off. Opens the Panda, sends the enabled exit commands,
  then closes it so the next startup cycle can proceed normally.
  """
  do_lock    = params.get_bool("ToyotaAutoLockOnExit")
  do_windows = params.get_bool("ToyotaCloseWindowsOnExit")
  do_mirrors = params.get_bool("ToyotaFoldMirrorsOnExit")

  if not (do_lock or do_windows or do_mirrors):
    return

  if not _is_toyota(params):
    return

  cloudlog.info("toyota_exit_actions: running post-ignition exit sequence")

  try:
    with Panda(panda_serial, disable_checks=True) as panda:
      if do_lock:
        panda.set_safety_mode(Panda.SAFETY_TOYOTA)
        panda.can_send(_BCM_ADDR, _LOCK_CMD, _BCM_BUS)
        cloudlog.info("toyota_exit_actions: doors locked")
        time.sleep(_CMD_DELAY)
        panda.send_heartbeat()

      if do_mirrors:
        panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
        for cmd in (_MIRR_FOLD_L, _MIRR_FOLD_R):
          panda.can_send(_BCM_ADDR, cmd, _BCM_BUS)
          time.sleep(_CMD_DELAY)
          panda.send_heartbeat()
        cloudlog.info("toyota_exit_actions: mirrors folded")

      if do_windows:
        panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
        for cmd in (_WINDOW_CLOSE_FL, _WINDOW_CLOSE_FR, _WINDOW_CLOSE_RL, _WINDOW_CLOSE_RR):
          panda.can_send(_BCM_ADDR, cmd, _BCM_BUS)
          time.sleep(_CMD_DELAY)
          panda.send_heartbeat()
        cloudlog.info("toyota_exit_actions: windows closed")

  except Exception:
    cloudlog.exception("toyota_exit_actions: error during exit sequence")
