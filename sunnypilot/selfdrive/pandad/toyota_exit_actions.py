"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Offroad exit actions for Toyota/Lexus vehicles.
Called once after ignition off (C++ pandad exits) to lock doors,
fold mirrors, and close windows via the Body Control Module (0x750).

Driver-exit detection mirrors FrogPilot's wait_for_no_driver approach:
  1. Wait for dmonitoringd to stop (happens automatically when ignition off)
  2. Set IsDriverViewEnabled to restart dmonitoringd with the camera
  3. Wait until no face is detected for FACE_CLEAR_TIME seconds
  4. Clean up IsDriverViewEnabled and send BCM commands

Commands (KWP2000 to BCM) sourced from FrogPilot / AlexandreSato.
Mirror fold and window close require SAFETY_ALLOUTPUT because Toyota
safety mode only permits a subset of 0x750 service IDs.
"""

import time

from cereal import car, log, messaging
from panda import Panda
from openpilot.common.params import Params
from openpilot.common.realtime import DT_HW, DT_DMON
from openpilot.common.swaglog import cloudlog

# BCM address and bus (same for all Toyota/Lexus)
_BCM_ADDR = 0x750
_BCM_BUS  = 0

# Door lock (allowed under SAFETY_TOYOTA)
_LOCK_CMD        = b"\x40\x05\x30\x11\x00\x80\x00\x00"

# Mirror fold – left and right (requires SAFETY_ALLOUTPUT)
_MIRR_FOLD_L     = b"\xA6\x04\x30\x21\x00\x08\x00\x00"
_MIRR_FOLD_R     = b"\xA5\x04\x30\x21\x00\x08\x00\x00"

# Window close – FL, FR, RL, RR (requires SAFETY_ALLOUTPUT)
_WINDOW_CLOSE_FL = b"\x90\x04\x30\x01\x05\x20\x00\x00"
_WINDOW_CLOSE_FR = b"\x91\x04\x30\x01\x05\x20\x00\x00"
_WINDOW_CLOSE_RL = b"\x93\x04\x30\x01\x05\x20\x00\x00"
_WINDOW_CLOSE_RR = b"\x92\x04\x30\x01\x05\x20\x00\x00"

_CMD_DELAY       = 0.15   # seconds between BCM commands (same as FrogPilot)
_PHASE_TIMEOUT   = 120.0  # max seconds to wait in each phase before giving up


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


def _ignition_on(sm) -> bool:
  return any(
    ps.ignitionLine or ps.ignitionCan
    for ps in sm["pandaStates"]
    if ps.pandaType != log.PandaState.PandaType.unknown
  )


def _dmon_running(sm) -> bool:
  return any(
    p.name == "dmonitoringd" and p.running
    for p in sm["managerState"].processes
  )


def _wait_for_driver_exit(params) -> bool:
  """
  Wait until the driver monitoring camera confirms no occupant.

  Phase 1: Wait for dmonitoringd to stop (ignition off → driverview=False).
  Phase 2: Set IsDriverViewEnabled=True to restart dmonitoringd via the camera.
  Phase 3: Wait ToyotaFaceClearTime seconds with no face detected.

  Returns True when safe to send BCM commands, False if ignition came back on
  or a phase timeout was reached.
  """
  try:
    sm = messaging.SubMaster(["managerState", "pandaStates", "driverMonitoringState"])
  except Exception:
    cloudlog.exception("toyota_exit_actions: could not create SubMaster")
    return False

  # Phase 1 — wait for dmonitoringd to stop
  deadline = time.monotonic() + _PHASE_TIMEOUT
  while True:
    sm.update(0)
    if _ignition_on(sm):
      cloudlog.info("toyota_exit_actions: ignition back on during phase 1, aborting")
      return False
    if not _dmon_running(sm):
      break
    if time.monotonic() > deadline:
      cloudlog.warning("toyota_exit_actions: phase 1 timeout — dmonitoringd never stopped")
      return False
    time.sleep(DT_HW)

  cloudlog.info("toyota_exit_actions: dmonitoringd stopped, re-enabling driver view")

  # Phase 2 — re-enable driver view so dmonitoringd restarts and uses the camera
  params.put_bool("IsDriverViewEnabled", True)

  deadline = time.monotonic() + _PHASE_TIMEOUT
  while True:
    sm.update(0)
    if _dmon_running(sm):
      break
    if time.monotonic() > deadline:
      cloudlog.warning("toyota_exit_actions: phase 2 timeout — dmonitoringd never restarted")
      params.remove("IsDriverViewEnabled")
      return False
    time.sleep(DT_HW)

  cloudlog.info("toyota_exit_actions: dmonitoringd running, waiting for empty car")

  # Phase 3 — wait until no face is detected for ToyotaFaceClearTime seconds
  face_clear_secs = float(int(params.get("ToyotaFaceClearTime", return_default=True) or 30))
  deadline = time.monotonic() + _PHASE_TIMEOUT
  face_clear_until = time.monotonic() + face_clear_secs

  while True:
    sm.update(0)

    if _ignition_on(sm):
      cloudlog.info("toyota_exit_actions: ignition back on during phase 3, aborting")
      params.remove("IsDriverViewEnabled")
      return False

    if time.monotonic() > deadline:
      cloudlog.warning("toyota_exit_actions: phase 3 timeout — car never confirmed empty")
      params.remove("IsDriverViewEnabled")
      return False

    # Face detected (or dmon state stale) → reset the clear timer
    if sm["driverMonitoringState"].faceDetected or not sm.alive["driverMonitoringState"]:
      face_clear_until = time.monotonic() + face_clear_secs

    if time.monotonic() >= face_clear_until:
      break

    time.sleep(DT_DMON)

  params.remove("IsDriverViewEnabled")
  cloudlog.info("toyota_exit_actions: car confirmed empty, proceeding")
  return True


def run_toyota_exit_actions(panda_serial: str, params) -> None:
  """
  Called after ignition off. Waits for the driver to exit (via dmonitoringd),
  then sends the enabled BCM exit commands via direct Panda access.
  """
  do_lock    = params.get_bool("ToyotaAutoLockOnExit")
  do_mirrors = params.get_bool("ToyotaFoldMirrorsOnExit")
  do_windows = params.get_bool("ToyotaCloseWindowsOnExit")

  if not (do_lock or do_mirrors or do_windows):
    return

  if not _is_toyota(params):
    return

  cloudlog.info("toyota_exit_actions: waiting for driver to exit")

  if not _wait_for_driver_exit(params):
    return

  cloudlog.info("toyota_exit_actions: running BCM exit sequence")

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
    cloudlog.exception("toyota_exit_actions: error during BCM exit sequence")
