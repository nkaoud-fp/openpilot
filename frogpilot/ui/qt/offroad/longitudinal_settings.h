#pragma once

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotLongitudinalPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotLongitudinalPanel(FrogPilotSettingsWindow *parent);

signals:
  void openSubPanel();
  void openSubSubPanel();

protected:
  void showEvent(QShowEvent *event) override;

private:
  void updateMetric(bool metric, bool bootRun);
  void updateToggles();

  bool customPersonalityOpen;
  bool forceOpenDescriptions;
  bool hasDashSpeedLimits;
  bool longitudinalTuneOpen;
  bool hasPCMCruise;
  bool isGM;
  bool isHKGCanFd;
  bool isToyota;
  bool isTSK;
  bool slcOpen;

  int tuningLevel;

  float longitudinalActuatorDelay;
  float startAccel;
  float stopAccel;
  float stoppingDecelRate;
  float vEgoStarting;
  float vEgoStopping;

  std::map<QString, AbstractControl*> toggles;

  QSet<QString> advancedLongitudinalTuneKeys = {"LongitudinalActuatorDelay", "StartAccel", "StopAccel", "StoppingDecelRate", "VEgoStarting", "VEgoStopping"};
  QSet<QString> aggressivePersonalityKeys = {"AggressiveFollow", "AggressiveJerkAcceleration", "AggressiveJerkDeceleration", "AggressiveJerkDanger", "AggressiveJerkSpeed", "AggressiveJerkSpeedDecrease", "ResetAggressivePersonality"};
  QSet<QString> conditionalExperimentalKeys = {"CESpeed", "CESpeedLead", "CECurves", "CELead", "CEModelStopTime", "CENavigation", "CESignalSpeed", "ShowCEMStatus"};
  QSet<QString> curveSpeedKeys = {"CalibratedLateralAcceleration", "CalibrationProgress", "ResetCurveData", "ShowCSCStatus"};
  QSet<QString> customDrivingPersonalityKeys = {"AutoPersonalityProfile", "AggressivePersonalityProfile", "DynamicPersonality" , "RelaxedPersonalityProfile", "StandardPersonalityProfile", "TrafficPersonalityProfile"};

  QSet<QString> dynamicPersonalityKeys = {
      "dy_speedlimit_follow", "dy_speedlimit_jerk_acceleration", "dy_speedlimit_jerk_deceleration", "dy_speedlimit_jerk_speed", "dy_speedlimit_jerk_speed_decrease", "dy_speedlimit_jerk_danger",
      "dy_dynamic_follow_min", "dy_dynamic_follow_max", "dy_dynamic_jerk_acceleration_min", "dy_dynamic_jerk_acceleration_max", "dy_dynamic_jerk_deceleration_min", "dy_dynamic_jerk_deceleration_max",
      "dy_dynamic_jerk_speed_min", "dy_dynamic_jerk_speed_max", "dy_dynamic_jerk_speed_decrease_min", "dy_dynamic_jerk_speed_decrease_max", "dy_dynamic_jerk_danger_min", "dy_dynamic_jerk_danger_max",
      "dy_cf_follow", "dy_cf_jerk_acceleration", "dy_cf_jerk_deceleration", "dy_cf_jerk_speed", "dy_cf_jerk_speed_decrease", "dy_cf_jerk_danger"
    };

  QSet<QString> creepToGapKeys = {"CreepGapTarget", "CreepAccel", "CreepMaxSpeed"};
  QSet<QString> longitudinalTuneKeys = {"AccelerationProfile", "CreepToGap", "DecelerationProfile", "ExperimentalAccelFloor", "ExperimentalAssertHeadroom", "ExperimentalSpeedAssertiveness", "HumanAcceleration", "HumanFollowing", "LeadDetectionThreshold", "MaxDesiredAcceleration", "SoftExperimentalModeBraking", "TacoTune"};
  QSet<QString> softExperimentalBrakingKeys = {"SoftExperimentalBaselineCap", "SoftExperimentalBaseStepSlow", "SoftExperimentalBaseStepFast", "SoftExperimentalDistanceBuffer"};
  QSet<QString> qolKeys = {"CustomCruise", "CustomCruiseLong", "ForceStops", "IncreasedStoppedDistance", "MapGears", "ReverseCruise", "SetSpeedOffset"};
  QSet<QString> relaxedPersonalityKeys = {"RelaxedFollow", "RelaxedJerkAcceleration", "RelaxedJerkDeceleration", "RelaxedJerkDanger", "RelaxedJerkSpeed", "RelaxedJerkSpeedDecrease", "ResetRelaxedPersonality"};
  QSet<QString> speedLimitControllerKeys = {"SLCOffsets", "SLCFallback", "SLCOverride", "SLCPriority", "SLCQOL", "SLCVisuals"};
  QSet<QString> speedLimitControllerOffsetsKeys = {"Offset1", "Offset2", "Offset3", "Offset4", "Offset5", "Offset6", "Offset7"};
  QSet<QString> speedLimitControllerQOLKeys = {"ForceMPHDashboard", "SetSpeedLimit", "SLCConfirmation", "SLCLookaheadHigher", "SLCLookaheadLower", "SLCMapboxFiller"};
  QSet<QString> speedLimitControllerVisualKeys = {"ShowSLCOffset", "SpeedLimitSources"};
  QSet<QString> standardPersonalityKeys = {"StandardFollow", "StandardJerkAcceleration", "StandardJerkDeceleration", "StandardJerkDanger", "StandardJerkSpeed", "StandardJerkSpeedDecrease", "ResetStandardPersonality"};
  QSet<QString> trafficPersonalityKeys = {"TrafficFollow", "TrafficJerkAcceleration", "TrafficJerkDeceleration", "TrafficJerkDanger", "TrafficJerkSpeed", "TrafficJerkSpeedDecrease", "ResetTrafficPersonality"};

  QSet<QString> parentKeys;

  FrogPilotParamValueControl *longitudinalActuatorDelayToggle;
  FrogPilotParamValueControl *startAccelToggle;
  FrogPilotParamValueControl *stopAccelToggle;
  FrogPilotParamValueControl *stoppingDecelRateToggle;
  FrogPilotParamValueControl *vEgoStartingToggle;
  FrogPilotParamValueControl *vEgoStoppingToggle;

  FrogPilotSettingsWindow *parent;

  LabelControl *calibratedLateralAccelerationLabel;
  LabelControl *calibrationProgressLabel;

  QJsonObject frogpilotToggleLevels;

  Params params;
  Params params_cache{"/cache/params"};
  Params params_default{"/dev/shm/params_default"};
  Params params_memory{"/dev/shm/params"};
};


