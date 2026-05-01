#include "frogpilot/ui/qt/offroad/navigation_settings.h"

#include <QFileInfo>

FrogPilotNavigationPanel::FrogPilotNavigationPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent), parent(parent) {
  QJsonObject shownDescriptions = QJsonDocument::fromJson(QString::fromStdString(params.get("ShownToggleDescriptions")).toUtf8()).object();
  QString className = this->metaObject()->className();

  if (!shownDescriptions.value(className).toBool(false)) {
    forceOpenDescriptions = true;
    shownDescriptions.insert(className, true);
    params.put("ShownToggleDescriptions", QJsonDocument(shownDescriptions).toJson(QJsonDocument::Compact).toStdString());
  }

  primelessLayout = new QStackedLayout();
  addItem(primelessLayout);

  FrogPilotListWidget *settingsList = new FrogPilotListWidget(this);
  ipLabel = new LabelControl(tr("Manage Your Settings At"), tr("Offline..."));
  settingsList->addItem(ipLabel);

  std::vector<QString> searchOptions{tr("Mapbox"), tr("Amap")};
  searchInput = new FrogPilotButtonsControl(tr("Destination Search Provider"),
                                            tr("<b>The search provider used for destination queries</b> in \"Navigate on Openpilot\". "
                                               "Options include Mapbox (recommended) and Amap."),
                                               "", searchOptions, true);
  QObject::connect(searchInput, &FrogPilotButtonsControl::buttonClicked, [this](int id) {
    amapKeyControl1->setVisible(id == 1);
    amapKeyControl2->setVisible(id == 1);

    params.putInt("SearchInput", id);

    update();
  });
  searchInput->setCheckedButton(params.getInt("SearchInput"));
  settingsList->addItem(searchInput);

  createKeyControl(amapKeyControl1, tr("Amap Key #1"), "AMapKey1", "", 39, settingsList);
  createKeyControl(amapKeyControl2, tr("Amap Key #2"), "AMapKey2", "", 39, settingsList);

  createKeyControl(publicMapboxKeyControl, tr("Public Mapbox Key"), "MapboxPublicKey", "pk.", 80, settingsList);
  createKeyControl(secretMapboxKeyControl, tr("Secret Mapbox Key"), "MapboxSecretKey", "sk.", 80, settingsList);

  driveLoggingToggle = new ParamControl("NavigationTestDriveLogging", tr("Navigation Test Drive Logging"),
                                        tr("<b>Save a per-drive CSV log</b> with navigation test maneuver decisions, prep phases, distances, and constraints for tuning."),
                                        "");
  settingsList->addItem(driveLoggingToggle);

  highwayPrepDistanceMinToggle = new FrogPilotParamValueControl("NavigationTestHighwayPrepDistanceMin", tr("Highway Prep Distance Min"),
                                                                tr("<b>Minimum distance before exits, forks, and merges where navigation can start lane positioning.</b>"),
                                                                "", 100, 6000, QString(), std::map<float, QString>(), 50, true);
  settingsList->addItem(highwayPrepDistanceMinToggle);

  highwayPrepDistanceMaxToggle = new FrogPilotParamValueControl("NavigationTestHighwayPrepDistanceMax", tr("Highway Prep Distance Max"),
                                                                tr("<b>Maximum distance before exits, forks, and merges where navigation can start lane positioning.</b>"),
                                                                "", 200, 6000, QString(), std::map<float, QString>(), 50, true);
  settingsList->addItem(highwayPrepDistanceMaxToggle);

  turnPrepDistanceMinToggle = new FrogPilotParamValueControl("NavigationTestTurnPrepDistanceMin", tr("Turn Prep Distance Min"),
                                                             tr("<b>Minimum distance before a branching turn where navigation can start lane positioning.</b>"),
                                                             "", 25, 1000, QString(), std::map<float, QString>(), 5, true);
  settingsList->addItem(turnPrepDistanceMinToggle);

  turnPrepDistanceMaxToggle = new FrogPilotParamValueControl("NavigationTestTurnPrepDistanceMax", tr("Turn Prep Distance Max"),
                                                             tr("<b>Maximum distance before a branching turn where navigation can start lane positioning.</b>"),
                                                             "", 50, 1000, QString(), std::map<float, QString>(), 5, true);
  settingsList->addItem(turnPrepDistanceMaxToggle);

  turnLockoutDistanceMinToggle = new FrogPilotParamValueControl("NavigationTestTurnLockoutDistanceMin", tr("Turn Lane Change Lockout Distance"),
                                                                tr("<b>Minimum remaining distance near a turn where navigation stops asking for more lane changes and commits to the maneuver.</b>"),
                                                                "", 5, 250, QString(), std::map<float, QString>(), 5, true);
  settingsList->addItem(turnLockoutDistanceMinToggle);

  turnSlowdownSpeedToggle = new FrogPilotParamValueControl("NavigationTestTurnSlowdownSpeed", tr("Turn Slowdown Target Speed"),
                                                           tr("<b>Target speed at the turn itself</b> for the gradual navigation slowdown profile."),
                                                           "", 5, 120, QString(), std::map<float, QString>(), 1, true);
  settingsList->addItem(turnSlowdownSpeedToggle);

  lastLogLabel = new LabelControl(tr("Latest Navigation Test Log"), tr("Waiting for first drive..."));
  settingsList->addItem(lastLogLabel);

  autoEmailToggle = new ParamControl("NavigationTestAutoEmail", tr("Email Latest Navigation Test Log"),
                                     tr("<b>Queue the most recent navigation test drive log for email</b> when the drive ends and retry while the device stays offroad."),
                                     "");
  settingsList->addItem(autoEmailToggle);

  createTextControl(smtpHostControl, tr("SMTP Host"), "NavigationTestEmailSMTPHost",
                    tr("<b>Set the SMTP server hostname</b> used for navigation test log emails."), settingsList, false, 1);
  createTextControl(smtpPortControl, tr("SMTP Port"), "NavigationTestEmailSMTPPort",
                    tr("<b>Set the SMTP server port</b> used for navigation test log emails."), settingsList, false, 1, true);
  createTextControl(smtpUserControl, tr("SMTP Username"), "NavigationTestEmailSMTPUser",
                    tr("<b>Set the SMTP username</b> used for navigation test log emails."), settingsList, false, 1);
  createTextControl(smtpPasswordControl, tr("SMTP Password"), "NavigationTestEmailSMTPPassword",
                    tr("<b>Set the SMTP password</b> used for navigation test log emails."), settingsList, true);
  createTextControl(emailFromControl, tr("Email From"), "NavigationTestEmailFrom",
                    tr("<b>Set the sender email address</b> used for navigation test log emails."), settingsList, false, 3);
  createTextControl(emailToControl, tr("Email To"), "NavigationTestEmailTo",
                    tr("<b>Set the recipient email address</b> used for navigation test log emails."), settingsList, false, 3);

  emailStatusLabel = new LabelControl(tr("Navigation Test Email Status"), tr("Idle"));
  settingsList->addItem(emailStatusLabel);

  setupButton = new ButtonControl(tr("Mapbox Setup Instructions"), tr("VIEW"), tr("<b>Instructions on how to set up Mapbox</b> for \"Primeless Navigation\"."), this);
  QObject::connect(setupButton, &ButtonControl::clicked, [this]() {
    openSubPanel();

    updateStep();

    primelessLayout->setCurrentIndex(1);
  });
  settingsList->addItem(setupButton);

  std::vector<QString> filterButtonNames{tr("CANCEL"), tr("Manually Update Speed Limits")};
  updateSpeedLimitsToggle = new FrogPilotButtonControl("SpeedLimitFiller", tr("Speed Limit Filler"),
                                                    tr("<b>Automatically collect missing or incorrect speed limits while you drive</b> using speeds limits sourced from your dashboard (if supported), "
                                                       "Mapbox, and \"Navigate on openpilot\".<br><br>"
                                                       "When you're parked and connected to Wi-Fi, FrogPilot will automatically processes this data into a file "
                                                       "to be used with the tool located at \"SpeedLimitFiller.frogpilot.download\".<br><br>"
                                                       "You can download this file from \"The Pond\" in the \"Download Speed Limits\" menu.<br><br>"
                                                       "Need a step-by-step guide? Visit <b>#speed-limit-filler</b> in the FrogPilot Discord!"),
                                                       "", filterButtonNames);
  QObject::connect(updateSpeedLimitsToggle, &FrogPilotButtonControl::buttonClicked, [this](int id) {
    if (id == 0) {
      if (FrogPilotConfirmationDialog::yesorno(tr("Cancel the speed-limit update?"), this)) {
        updatingLimits = false;

        updateSpeedLimitsToggle->setEnabledButton(0, false);
        updateSpeedLimitsToggle->setValue(tr("Cancelled..."));

        params_memory.remove("UpdateSpeedLimits");

        QTimer::singleShot(2500, [this]() {
          updateSpeedLimitsToggle->clearCheckedButtons(true);
          updateSpeedLimitsToggle->setEnabledButton(0, true);
          updateSpeedLimitsToggle->setValue("");
          updateSpeedLimitsToggle->setVisibleButton(0, false);
          updateSpeedLimitsToggle->setVisibleButton(1, true);

          params_memory.remove("UpdateSpeedLimitsStatus");
        });
      }
    } else if (id == 1) {
      QJsonObject overpassRequests = QJsonDocument::fromJson(QString::fromStdString(params.get("OverpassRequests")).toUtf8()).object();

      int totalRequests = overpassRequests.value("total_requests").toInt(0);
      int maxRequests = overpassRequests.value("max_requests").toInt(10000);
      int savedDay = overpassRequests.value("day").toInt(QDate::currentDate().day());

      int currentDay = QDate::currentDate().day();

      if (savedDay != currentDay) {
        totalRequests = 0;
      }

      if (totalRequests >= maxRequests) {
        QTime now = QTime::currentTime();

        int secondsUntilMidnight = (24 * 3600) - (now.hour() * 3600 + now.minute() * 60 + now.second());
        int hours = secondsUntilMidnight / 3600;
        int minutes = (secondsUntilMidnight % 3600) / 60;

        ConfirmationDialog::alert(QString(tr("You've hit today's request limit.\n\nIt will reset in %1 hours and %2 minutes.")).arg(hours).arg(minutes), this);

        updateSpeedLimitsToggle->clearCheckedButtons(true);
        return;
      }

      updateSpeedLimitsToggle->setVisibleButton(0, true);
      updateSpeedLimitsToggle->setVisibleButton(1, false);

      if (FrogPilotConfirmationDialog::yesorno(tr("This process takes a while. It's recommended to start when you're done driving and connected to stable Wi-Fi. Continue?"), this)) {
        updatingLimits = true;

        updateSpeedLimitsToggle->setValue("Calculating...");

        params_memory.put("UpdateSpeedLimitsStatus", "Calculating...");
        params_memory.putBool("UpdateSpeedLimits", true);
      } else {
        updateSpeedLimitsToggle->setVisibleButton(0, false);
        updateSpeedLimitsToggle->setVisibleButton(1, true);

        updateSpeedLimitsToggle->clearCheckedButtons(true);
      }
    }
  });
  updateSpeedLimitsToggle->setVisibleButton(0, false);
  settingsList->addItem(updateSpeedLimitsToggle);

  ScrollView *settingsPanel = new ScrollView(settingsList, this);
  primelessLayout->addWidget(settingsPanel);

  imageLabel = new QLabel(this);

  ScrollView *instructionsPanel = new ScrollView(imageLabel, this);
  primelessLayout->addWidget(instructionsPanel);

  QObject::connect(parent, &FrogPilotSettingsWindow::closeSubPanel, [this]() {
    primelessLayout->setCurrentIndex(0);

    if (forceOpenDescriptions) {
      amapKeyControl1->showDescription();
      amapKeyControl2->showDescription();
      autoEmailToggle->showDescription();
      driveLoggingToggle->showDescription();
      emailFromControl->showDescription();
      emailToControl->showDescription();
      highwayPrepDistanceMaxToggle->showDescription();
      highwayPrepDistanceMinToggle->showDescription();
      publicMapboxKeyControl->showDescription();
      searchInput->showDescription();
      secretMapboxKeyControl->showDescription();
      setupButton->showDescription();
      smtpHostControl->showDescription();
      smtpPasswordControl->showDescription();
      smtpPortControl->showDescription();
      smtpUserControl->showDescription();
      turnLockoutDistanceMinToggle->showDescription();
      turnPrepDistanceMaxToggle->showDescription();
      turnPrepDistanceMinToggle->showDescription();
      turnSlowdownSpeedToggle->showDescription();
      updateSpeedLimitsToggle->showDescription();
    }
  });
  QObject::connect(static_cast<ToggleControl*>(driveLoggingToggle), &ToggleControl::toggleFlipped, [this](bool) {
    updateEmailControls();
  });
  QObject::connect(static_cast<ToggleControl*>(autoEmailToggle), &ToggleControl::toggleFlipped, [this](bool) {
    updateEmailControls();
  });
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotNavigationPanel::updateState);
}

void FrogPilotNavigationPanel::showEvent(QShowEvent *event) {
  if (forceOpenDescriptions) {
    amapKeyControl1->showDescription();
    amapKeyControl2->showDescription();
    autoEmailToggle->showDescription();
    driveLoggingToggle->showDescription();
    emailFromControl->showDescription();
    emailToControl->showDescription();
    highwayPrepDistanceMaxToggle->showDescription();
    highwayPrepDistanceMinToggle->showDescription();
    publicMapboxKeyControl->showDescription();
    searchInput->showDescription();
    secretMapboxKeyControl->showDescription();
    setupButton->showDescription();
    smtpHostControl->showDescription();
    smtpPasswordControl->showDescription();
    smtpPortControl->showDescription();
    smtpUserControl->showDescription();
    turnLockoutDistanceMinToggle->showDescription();
    turnPrepDistanceMaxToggle->showDescription();
    turnPrepDistanceMinToggle->showDescription();
    turnSlowdownSpeedToggle->showDescription();
    updateSpeedLimitsToggle->showDescription();
  }

  FrogPilotUIState &fs = *frogpilotUIState();
  UIState &s = *uiState();

  FrogPilotUIScene &frogpilot_scene = fs.frogpilot_scene;

  QString ipAddress = fs.wifi->getIp4Address();
  ipLabel->setText(ipAddress.isEmpty() ? tr("Offline...") : QString("%1:8082").arg(ipAddress));

  updateButtons();
  updateMetric(params.getBool("IsMetric"), true);

  setupCompleted = mapboxPublicKeySet && mapboxSecretKeySet;
  updatingLimits = !params_memory.get("UpdateSpeedLimitsStatus").empty() && QString::fromStdString(params_memory.get("UpdateSpeedLimitsStatus")) != "Completed!";

  bool parked = !s.scene.started || fs.frogpilot_scene.parked || fs.frogpilot_toggles.value("frogs_go_moo").toBool();

  int selectedSearchInput = params.getInt("SearchInput");

  amapKeyControl1->setVisible(selectedSearchInput == 1);
  amapKeyControl2->setVisible(selectedSearchInput == 1);

  updateEmailControls();

  updateSpeedLimitsToggle->setVisibleButton(0, updatingLimits);
  updateSpeedLimitsToggle->setVisibleButton(1, !updatingLimits);

  if (updatingLimits) {
    updateSpeedLimitsToggle->setValue(QString::fromStdString(params_memory.get("UpdateSpeedLimitsStatus")));
  } else {
    updateSpeedLimitsToggle->setEnabledButton(1, frogpilot_scene.online && util::system_time_valid() && parked);
    updateSpeedLimitsToggle->setValue(frogpilot_scene.online ? (parked ? "" : "Not parked") : tr("Offline..."));
    updateSpeedLimitsToggle->setVisible(parent->tuningLevel >= parent->frogpilotToggleLevels["SpeedLimitFiller"].toDouble());
  }
}

void FrogPilotNavigationPanel::hideEvent(QHideEvent *event) {
  primelessLayout->setCurrentIndex(0);
}

void FrogPilotNavigationPanel::mousePressEvent(QMouseEvent *event) {
  if (primelessLayout->currentIndex() == 1) {
    closeSubPanel();

    primelessLayout->setCurrentIndex(0);

    if (forceOpenDescriptions) {
      amapKeyControl1->showDescription();
      amapKeyControl2->showDescription();
      autoEmailToggle->showDescription();
      driveLoggingToggle->showDescription();
      emailFromControl->showDescription();
      emailToControl->showDescription();
      highwayPrepDistanceMaxToggle->showDescription();
      highwayPrepDistanceMinToggle->showDescription();
      publicMapboxKeyControl->showDescription();
      searchInput->showDescription();
      secretMapboxKeyControl->showDescription();
      setupButton->showDescription();
      smtpHostControl->showDescription();
      smtpPasswordControl->showDescription();
      smtpPortControl->showDescription();
      smtpUserControl->showDescription();
      turnLockoutDistanceMinToggle->showDescription();
      turnPrepDistanceMaxToggle->showDescription();
      turnPrepDistanceMinToggle->showDescription();
      turnSlowdownSpeedToggle->showDescription();
      updateSpeedLimitsToggle->showDescription();
    }
  }
}

void FrogPilotNavigationPanel::createKeyControl(ButtonControl *&control, const QString &label, const std::string &paramKey, const QString &prefix, const int &minLength, FrogPilotListWidget *list) {
  control = new ButtonControl(label, "", tr("<b>Manage your \"%1\".</b>").arg(label));
  QObject::connect(control, &ButtonControl::clicked, [=] {
    if (control->text() == tr("ADD")) {
      QString key = InputDialog::getText(tr("Enter your %1").arg(label), this).trimmed();

      if (!key.startsWith(prefix)) {
        key = prefix + key;
      }

      if (key.length() >= minLength) {
        params.put(paramKey, key.toStdString());
      } else {
        ConfirmationDialog::alert(tr("Inputted key is invalid or too short!"), this);
      }
    } else {
      if (FrogPilotConfirmationDialog::yesorno(tr("Remove your %1?").arg(label), this)) {
        control->setText(tr("ADD"));

        params.remove(paramKey);
        params_cache.remove(paramKey);

        setupCompleted = false;
      }
    }
  });
  control->setText(QString::fromStdString(params.get(paramKey)).startsWith(prefix) ? tr("REMOVE") : tr("ADD"));
  list->addItem(control);
}

void FrogPilotNavigationPanel::createTextControl(ButtonControl *&control, const QString &label, const std::string &paramKey, const QString &subtitle, FrogPilotListWidget *list, bool secret, int minLength, bool numeric) {
  control = new ButtonControl(label, "", subtitle);
  QObject::connect(control, &ButtonControl::clicked, [=] {
    const QString currentValue = QString::fromStdString(params.get(paramKey));

    if (control->text() == tr("REMOVE")) {
      if (FrogPilotConfirmationDialog::yesorno(tr("Remove your %1?").arg(label), this)) {
        params.remove(paramKey);
        params_cache.remove(paramKey);
      }
    } else {
      QString value = InputDialog::getText(tr("Enter your %1").arg(label), this, "", secret, minLength, currentValue).trimmed();
      if (value.isEmpty()) {
        return;
      }

      if (numeric) {
        bool ok = false;
        int number = value.toInt(&ok);
        if (!ok || number < 1 || number > 65535) {
          ConfirmationDialog::alert(tr("Please enter a valid number between 1 and 65535."), this);
          return;
        }
        value = QString::number(number);
      } else if (value.length() < minLength) {
        ConfirmationDialog::alert(tr("Input is invalid or too short!"), this);
        return;
      }

      params.put(paramKey, value.toStdString());
    }

    control->setText(params.get(paramKey).empty() ? tr("ADD") : tr("REMOVE"));
    updateEmailControls();
  });
  control->setText(params.get(paramKey).empty() ? tr("ADD") : tr("REMOVE"));
  list->addItem(control);
}

void FrogPilotNavigationPanel::updateButtons() {
  amapKeyControl1->setText(params.get("AMapKey1").empty() ? tr("ADD") : tr("REMOVE"));
  amapKeyControl2->setText(params.get("AMapKey2").empty() ? tr("ADD") : tr("REMOVE"));

  mapboxPublicKeySet = QString::fromStdString(params.get("MapboxPublicKey")).startsWith("pk");
  mapboxSecretKeySet = QString::fromStdString(params.get("MapboxSecretKey")).startsWith("sk");

  publicMapboxKeyControl->setText(mapboxPublicKeySet ? tr("REMOVE") : tr("ADD"));
  secretMapboxKeyControl->setText(mapboxSecretKeySet ? tr("REMOVE") : tr("ADD"));
}

void FrogPilotNavigationPanel::updateMetric(bool metric, bool bootRun) {
  if (metric != previousMetric && !bootRun) {
    double distanceConversion = metric ? FOOT_TO_METER : METER_TO_FOOT;
    double speedConversion = metric ? MILE_TO_KM : KM_TO_MILE;

    params.putFloatNonBlocking("NavigationTestHighwayPrepDistanceMin", params.getFloat("NavigationTestHighwayPrepDistanceMin") * distanceConversion);
    params.putFloatNonBlocking("NavigationTestHighwayPrepDistanceMax", params.getFloat("NavigationTestHighwayPrepDistanceMax") * distanceConversion);
    params.putFloatNonBlocking("NavigationTestTurnPrepDistanceMin", params.getFloat("NavigationTestTurnPrepDistanceMin") * distanceConversion);
    params.putFloatNonBlocking("NavigationTestTurnPrepDistanceMax", params.getFloat("NavigationTestTurnPrepDistanceMax") * distanceConversion);
    params.putFloatNonBlocking("NavigationTestTurnLockoutDistanceMin", params.getFloat("NavigationTestTurnLockoutDistanceMin") * distanceConversion);
    params.putFloatNonBlocking("NavigationTestTurnSlowdownSpeed", params.getFloat("NavigationTestTurnSlowdownSpeed") * speedConversion);
  }
  previousMetric = metric;

  static std::map<float, QString> imperialDistanceLabels;
  static std::map<float, QString> imperialHighwayDistanceLabels;
  static std::map<float, QString> imperialSpeedLabels;
  static std::map<float, QString> metricDistanceLabels;
  static std::map<float, QString> metricHighwayDistanceLabels;
  static std::map<float, QString> metricSpeedLabels;

  static bool labelsInitialized = false;
  if (!labelsInitialized) {
    for (int i = 0; i <= 3500; i += 5) {
      imperialDistanceLabels[i] = i == 1 ? QString::number(i) + tr(" foot") : QString::number(i) + tr(" feet");
    }
    for (int i = 0; i <= 20000; i += 50) {
      imperialHighwayDistanceLabels[i] = i == 1 ? QString::number(i) + tr(" foot") : QString::number(i) + tr(" feet");
    }
    for (int i = 0; i <= 75; ++i) {
      imperialSpeedLabels[i] = QString::number(i) + tr(" mph");
    }

    for (int i = 0; i <= 1000; i += 5) {
      metricDistanceLabels[i] = i == 1 ? QString::number(i) + tr(" meter") : QString::number(i) + tr(" meters");
    }
    for (int i = 0; i <= 6000; i += 50) {
      metricHighwayDistanceLabels[i] = i == 1 ? QString::number(i) + tr(" meter") : QString::number(i) + tr(" meters");
    }
    for (int i = 0; i <= 120; ++i) {
      metricSpeedLabels[i] = QString::number(i) + tr(" km/h");
    }

    labelsInitialized = true;
  }

  if (metric) {
    highwayPrepDistanceMinToggle->updateControl(100, 6000, metricHighwayDistanceLabels);
    highwayPrepDistanceMaxToggle->updateControl(200, 6000, metricHighwayDistanceLabels);
    turnPrepDistanceMinToggle->updateControl(25, 1000, metricDistanceLabels);
    turnPrepDistanceMaxToggle->updateControl(50, 1000, metricDistanceLabels);
    turnLockoutDistanceMinToggle->updateControl(5, 250, metricDistanceLabels);
    turnSlowdownSpeedToggle->updateControl(5, 120, metricSpeedLabels);
  } else {
    highwayPrepDistanceMinToggle->updateControl(300, 20000, imperialHighwayDistanceLabels);
    highwayPrepDistanceMaxToggle->updateControl(500, 20000, imperialHighwayDistanceLabels);
    turnPrepDistanceMinToggle->updateControl(100, 3500, imperialDistanceLabels);
    turnPrepDistanceMaxToggle->updateControl(150, 3500, imperialDistanceLabels);
    turnLockoutDistanceMinToggle->updateControl(15, 800, imperialDistanceLabels);
    turnSlowdownSpeedToggle->updateControl(5, 75, imperialSpeedLabels);
  }
}

void FrogPilotNavigationPanel::updateEmailControls() {
  bool driveLoggingEnabled = params.getBool("NavigationTestDriveLogging");
  bool autoEmailEnabled = params.getBool("NavigationTestAutoEmail");

  QString lastLogPath = QString::fromStdString(params.get("NavigationTestLastDriveLog"));
  QString emailStatus = QString::fromStdString(params.get("NavigationTestEmailLastStatus"));

  lastLogLabel->setVisible(driveLoggingEnabled || !lastLogPath.isEmpty());
  if (lastLogPath.isEmpty()) {
    lastLogLabel->setText(tr("Waiting for first drive..."));
  } else {
    QFileInfo logInfo(lastLogPath);
    lastLogLabel->setText(logInfo.fileName());
  }

  smtpHostControl->setVisible(autoEmailEnabled);
  smtpPortControl->setVisible(autoEmailEnabled);
  smtpUserControl->setVisible(autoEmailEnabled);
  smtpPasswordControl->setVisible(autoEmailEnabled);
  emailFromControl->setVisible(autoEmailEnabled);
  emailToControl->setVisible(autoEmailEnabled);
  emailStatusLabel->setVisible(driveLoggingEnabled || autoEmailEnabled || !emailStatus.isEmpty());
  emailStatusLabel->setText(emailStatus.isEmpty() ? tr("Idle") : emailStatus);
}

void FrogPilotNavigationPanel::updateState(const UIState &s, const FrogPilotUIState &fs) {
  if (!isVisible() || s.sm->frame % (UI_FREQ / 2) != 0) {
    return;
  }

  updateButtons();
  updateEmailControls();
  updateMetric(params.getBool("IsMetric"));
  updateStep();

  bool parked = !s.scene.started || fs.frogpilot_scene.parked || fs.frogpilot_toggles.value("frogs_go_moo").toBool();

  if (updatingLimits) {
    if (QString::fromStdString(params_memory.get("UpdateSpeedLimitsStatus")) == "Completed!") {
      updatingLimits = false;

      updateSpeedLimitsToggle->setValue(tr("Completed!"));

      QTimer::singleShot(2500, [this]() {
        updateSpeedLimitsToggle->clearCheckedButtons(true);
        updateSpeedLimitsToggle->setValue("");
        updateSpeedLimitsToggle->setVisibleButton(0, false);
        updateSpeedLimitsToggle->setVisibleButton(1, true);

        params_memory.remove("UpdateSpeedLimitsStatus");
      });
    } else {
      updateSpeedLimitsToggle->setValue(QString::fromStdString(params_memory.get("UpdateSpeedLimitsStatus")));
    }
  } else {
    updateSpeedLimitsToggle->setEnabledButton(1, fs.frogpilot_scene.online && util::system_time_valid() && parked);
    updateSpeedLimitsToggle->setValue(fs.frogpilot_scene.online ? (parked ? "" : "Not parked") : tr("Offline..."));
  }

  parent->keepScreenOn = primelessLayout->currentIndex() == 1 || updatingLimits;
}

void FrogPilotNavigationPanel::updateStep() {
  QString currentStep;
  if (setupCompleted) {
    currentStep = "../../frogpilot/navigation/navigation_training/setup_completed.png";
  } else if (mapboxPublicKeySet && mapboxSecretKeySet) {
    currentStep = "../../frogpilot/navigation/navigation_training/both_keys_set.png";
  } else if (mapboxPublicKeySet) {
    currentStep = "../../frogpilot/navigation/navigation_training/public_key_set.png";
  } else {
    currentStep = "../../frogpilot/navigation/navigation_training/no_keys_set.png";
  }

  QPixmap pixmap;
  pixmap.load(currentStep);
  imageLabel->setPixmap(pixmap.scaledToWidth(1500, Qt::SmoothTransformation));

  update();
}
