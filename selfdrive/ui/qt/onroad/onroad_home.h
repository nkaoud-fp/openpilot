#pragma once

#include "selfdrive/ui/qt/onroad/alerts.h"
#include "selfdrive/ui/qt/onroad/annotated_camera.h"

#include "frogpilot/ui/qt/onroad/frogpilot_onroad.h"

class OnroadWindow : public QWidget {
  Q_OBJECT

public:
  OnroadWindow(QWidget* parent = 0);
  bool isMapVisible() const { return map && map->isVisible(); }
  void showMapPanel(bool show) { if (map) map->setVisible(show); }

signals:
  void mapPanelRequested();

private:
  void createMapWidget();
  void paintEvent(QPaintEvent *event);
  void mousePressEvent(QMouseEvent* e) override;
  OnroadAlerts *alerts;
  AnnotatedCameraWidget *nvg;
  QColor bg = bg_colors[STATUS_DISENGAGED];
  QWidget *map = nullptr;
  QHBoxLayout* split;
  QVBoxLayout *main_layout; // To store the pointer to your main layout
  bool prev_headless_mode_state = false; // To track the previous state of headless_mode


  // FrogPilot variables
  void resizeEvent(QResizeEvent *event);

  FrogPilotOnroadWindow *frogpilot_onroad;

private slots:
  void offroadTransition(bool offroad);
  void primeChanged(bool prime);
  void updateState(const UIState &s, const FrogPilotUIState &fs);
};
