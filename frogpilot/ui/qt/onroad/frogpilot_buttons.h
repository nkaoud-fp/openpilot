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

  bool navigationTestEnabled();
  void updateState();

signals:
  void destinationPickerRequested();

private:
  void paintEvent(QPaintEvent *event) override;

  Params params;

  bool navigation_test_enabled = false;
  QString navigation_test_destination;
};

class NavigationDestinationButton : public QPushButton {
  Q_OBJECT

public:
  explicit NavigationDestinationButton(const QString &label, const QString &destination_id, double latitude, double longitude, const QString &place_name, QWidget *parent = 0);

signals:
  void destinationSelected();

private:
  void paintEvent(QPaintEvent *event) override;
  void selectDestination();

  Params params;

  QString destination_id;
  QString label;
  QString place_name;

  double latitude;
  double longitude;
};
