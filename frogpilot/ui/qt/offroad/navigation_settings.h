#pragma once

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotNavigationPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotNavigationPanel(FrogPilotSettingsWindow *parent);

signals:
  void closeSubPanel();
  void openSubPanel();

protected:
  void hideEvent(QHideEvent *event);
  void showEvent(QShowEvent *event) override;

private:
  void createKeyControl(ButtonControl *&control, const QString &label, const std::string &paramKey, const QString &prefix, const int &minLength, FrogPilotListWidget *list);
  void createTextControl(ButtonControl *&control, const QString &label, const std::string &paramKey, const QString &subtitle, FrogPilotListWidget *list, bool secret = false, int minLength = 0, bool numeric = false);
  void mousePressEvent(QMouseEvent *event);
  void updateButtons();
  void updateEmailControls();
  void updateMetric(bool metric, bool bootRun = false);
  void updateState(const UIState &s, const FrogPilotUIState &fs);
  void updateStep();

  bool forceOpenDescriptions;
  bool mapboxPublicKeySet;
  bool mapboxSecretKeySet;
  bool previousMetric = false;
  bool setupCompleted;
  bool updatingLimits;

  ButtonControl *amapKeyControl1;
  ButtonControl *amapKeyControl2;
  ButtonControl *publicMapboxKeyControl;
  ButtonControl *secretMapboxKeyControl;
  ButtonControl *setupButton;
  ButtonControl *smtpHostControl;
  ButtonControl *smtpPortControl;
  ButtonControl *smtpUserControl;
  ButtonControl *smtpPasswordControl;
  ButtonControl *emailFromControl;
  ButtonControl *emailToControl;

  ParamControl *autoEmailToggle;
  ParamControl *driveLoggingToggle;
  FrogPilotParamValueControl *highwayPrepDistanceMaxToggle;
  FrogPilotParamValueControl *highwayPrepDistanceMinToggle;
  FrogPilotParamValueControl *turnLockoutDistanceMinToggle;
  FrogPilotParamValueControl *turnPrepDistanceMaxToggle;
  FrogPilotParamValueControl *turnPrepDistanceMinToggle;
  FrogPilotParamValueControl *turnSlowdownStartDistanceToggle;
  FrogPilotParamValueControl *turnSlowdownSpeedToggle;
  FrogPilotButtonControl *updateSpeedLimitsToggle;

  FrogPilotButtonsControl *searchInput;

  FrogPilotSettingsWindow *parent;

  LabelControl *ipLabel;
  LabelControl *emailStatusLabel;
  LabelControl *lastLogLabel;

  Params params;
  Params params_cache{"/cache/params"};
  Params params_memory{"/dev/shm/params"};

  QLabel *imageLabel;

  QStackedLayout *primelessLayout;
};
