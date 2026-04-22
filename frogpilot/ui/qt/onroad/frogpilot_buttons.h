#pragma once

#include <QMovie>
#include <QString>

#include "selfdrive/ui/qt/onroad/buttons.h"

class DistanceButton : public QPushButton {
  Q_OBJECT

public:
  explicit DistanceButton(QWidget *parent = 0);

  void updateState(const UIScene &scene, const FrogPilotUIScene &frogpilot_scene);

private:
  void paintEvent(QPaintEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void updateTheme();

  bool traffic_mode_active;

  int personality;

  Params params_memory{"/dev/shm/params"};

  QMap<int, QPair<QPixmap, QSharedPointer<QMovie>>> icon_map;
};

class NavigationTestButton : public QPushButton {
  Q_OBJECT

public:
  explicit NavigationTestButton(QWidget *parent = 0);

  void updateState();

private:
  bool selectDestination();
  void paintEvent(QPaintEvent *event) override;

  Params params;

  bool navigation_test_enabled = false;
  QString navigation_test_destination;
};
