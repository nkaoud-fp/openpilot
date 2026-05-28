#!/usr/bin/env python3
import glob
import os
import smtplib
import ssl
import time

from email.message import EmailMessage

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

NAVIGATION_TEST_EMAIL_STATUS_TIME_FORMAT = "%Y-%m-%d %H:%M:%S"
NAVIGATION_TEST_EMAIL_SUBJECT = "Comma Navigation Test"
NAVIGATION_TEST_EMAIL_BODY = (
  "Attached is the latest FrogPilot navigation test drive log.\n\n"
  "This message was generated automatically when the drive ended."
)
NAVIGATION_TEST_EMAIL_CONFIG_PATH = os.path.abspath(
  os.path.join(os.path.dirname(__file__), "..", "..", "selfdrive", "navd", "config.txt")
)


def _status_prefix():
  return time.strftime(NAVIGATION_TEST_EMAIL_STATUS_TIME_FORMAT, time.localtime())


def _set_status(params, message):
  params.put("NavigationTestEmailLastStatus", f"{_status_prefix()} - {message}")


def set_navigation_test_email_status(message):
  _set_status(Params(), message)


def _read_navigation_test_email_config():
  if not os.path.isfile(NAVIGATION_TEST_EMAIL_CONFIG_PATH):
    return {}

  config = {}
  try:
    with open(NAVIGATION_TEST_EMAIL_CONFIG_PATH, "r", encoding="utf-8") as config_file:
      for raw_line in config_file:
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
          continue

        key, value = line.split("=", 1)
        config[key.strip()] = value.strip().strip('"').strip("'")
  except OSError:
    cloudlog.exception("navigation_test_email.config_read_failed")
    return {}

  return config


def _get_brevo_smtp_key(config, params):
  segmented_key = "".join([
    config.get("BREVO_SMTP_KEY_A", ""),
    config.get("BREVO_SMTP_KEY_B", ""),
    config.get("BREVO_SMTP_KEY_C", ""),
    config.get("BREVO_SMTP_KEY_D", ""),
  ])

  if segmented_key:
    return segmented_key

  return config.get("BREVO_SMTP_KEY") or params.get("NavigationTestEmailSMTPPassword", encoding="utf8")


def queue_navigation_test_log(log_path):
  params = Params()

  if not log_path:
    _set_status(params, "No navigation test log was captured for the last drive")
    return False

  params.put("NavigationTestLastDriveLog", log_path)

  if not os.path.isfile(log_path):
    _set_status(params, "Latest navigation test log file is missing")
    return False

  params.put("NavigationTestEmailPendingLog", log_path)
  _set_status(params, f"Queued {os.path.basename(log_path)}")
  return True


def send_pending_navigation_test_log():
  params = Params()
  pending_log = params.get("NavigationTestEmailPendingLog", encoding="utf8")

  if not pending_log:
    return False

  if not os.path.isfile(pending_log):
    params.remove("NavigationTestEmailPendingLog")
    _set_status(params, "Pending navigation test log no longer exists")
    return False

  config = _read_navigation_test_email_config()

  smtp_host = config.get("BREVO_SMTP_HOST") or params.get("NavigationTestEmailSMTPHost", encoding="utf8") or "smtp-relay.brevo.com"
  smtp_user = config.get("BREVO_SMTP_LOGIN") or params.get("NavigationTestEmailSMTPUser", encoding="utf8")
  smtp_password = _get_brevo_smtp_key(config, params)
  email_from = config.get("SENDER_EMAIL") or params.get("NavigationTestEmailFrom", encoding="utf8")
  email_to = config.get("EMAIL_RECEIVER") or params.get("NavigationTestEmailTo", encoding="utf8")

  missing_config = []
  if not smtp_host:
    missing_config.append("SMTP host")
  if not email_from:
    missing_config.append("from address")
  if not email_to:
    missing_config.append("recipient address")
  if bool(smtp_user) != bool(smtp_password):
    missing_config.append("matching SMTP username/password")

  if missing_config:
    _set_status(params, f"Email config incomplete: {', '.join(missing_config)}")
    return False

  smtp_port_raw = config.get("BREVO_SMTP_PORT") or params.get("NavigationTestEmailSMTPPort", encoding="utf8") or "587"
  try:
    smtp_port = int(smtp_port_raw)
  except (TypeError, ValueError):
    _set_status(params, f"Invalid SMTP port: {smtp_port_raw}")
    return False

  message = EmailMessage()
  message["Subject"] = f"{NAVIGATION_TEST_EMAIL_SUBJECT} - {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}"
  message["From"] = email_from
  message["To"] = email_to
  message.set_content(NAVIGATION_TEST_EMAIL_BODY)

  with open(pending_log, "rb") as log_file:
    message.add_attachment(log_file.read(), maintype="text", subtype="csv", filename=os.path.basename(pending_log))

  log_base, _ = os.path.splitext(pending_log)
  mapbox_response_paths = sorted(glob.glob(f"{log_base}.mapbox.*.json"))
  for response_path in mapbox_response_paths:
    try:
      with open(response_path, "rb") as response_file:
        message.add_attachment(response_file.read(), maintype="application", subtype="json", filename=os.path.basename(response_path))
    except OSError:
      cloudlog.exception("navigation_test_email.mapbox_attach_failed")

  timeout = 20
  try:
    if smtp_port == 465:
      with smtplib.SMTP_SSL(smtp_host, smtp_port, timeout=timeout, context=ssl.create_default_context()) as server:
        if smtp_user and smtp_password:
          server.login(smtp_user, smtp_password)
        server.send_message(message)
    else:
      with smtplib.SMTP(smtp_host, smtp_port, timeout=timeout) as server:
        server.ehlo()
        if smtp_port in (587, 2525):
          server.starttls(context=ssl.create_default_context())
          server.ehlo()
        if smtp_user and smtp_password:
          server.login(smtp_user, smtp_password)
        server.send_message(message)
  except Exception:
    cloudlog.exception("navigation_test_email.failed")
    _set_status(params, f"Send failed for {os.path.basename(pending_log)}")
    return False

  cleanup_failed = False
  try:
    os.remove(pending_log)
  except OSError:
    cloudlog.exception("navigation_test_email.cleanup_failed")
    cleanup_failed = True

  for response_path in mapbox_response_paths:
    try:
      os.remove(response_path)
    except OSError:
      cloudlog.exception("navigation_test_email.mapbox_cleanup_failed")
      cleanup_failed = True

  params.remove("NavigationTestEmailPendingLog")
  if cleanup_failed:
    _set_status(params, f"Sent {os.path.basename(pending_log)} but could not delete one or more files")
    return False

  _set_status(params, f"Sent and deleted {os.path.basename(pending_log)} (+{len(mapbox_response_paths)} mapbox)")
  return True
