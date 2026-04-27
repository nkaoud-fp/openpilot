#include "frogpilot/ui/qt/onroad/frogpilot_buttons.h"

#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>

#include "selfdrive/ui/qt/util.h"

namespace {

QString navigationTestDestinationLabel(const std::string &destination) {
  if (destination == "home") return "HOM";
  if (destination == "share") return "SHR";
  if (destination == "work") return "WRK";
  if (destination == "school") return "SCH";
  return "NAV";
}

QByteArray navigationTestDestinationJson(double latitude, double longitude, const QString &place_name) {
  QJsonObject destination_json{
    {"latitude", latitude},
    {"longitude", longitude},
    {"place_name", place_name},
  };
  return QJsonDocument(destination_json).toJson(QJsonDocument::Compact);
}

}  // namespace

DistanceButton::DistanceButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size + UI_BORDER_SIZE, btn_size);

  QObject::connect(frogpilotUIState(), &FrogPilotUIState::themeUpdated, this, &DistanceButton::updateTheme);
  QObject::connect(this, &QPushButton::pressed, [this] {params_memory.putBool("OnroadDistanceButtonPressed", true);});
  QObject::connect(this, &QPushButton::released, [this] {params_memory.putBool("OnroadDistanceButtonPressed", false);});
}

void DistanceButton::showEvent(QShowEvent *event) {
  updateTheme();
}

void DistanceButton::updateTheme() {
  for (QMap<int, QPair<QPixmap, QSharedPointer<QMovie>>>::iterator it = icon_map.begin(); it != icon_map.end(); ++it) {
    QSharedPointer<QMovie> movie = it.value().second;
    if (!movie.isNull()) {
      QObject::disconnect(movie.data(), nullptr, this, nullptr);
      movie->stop();
    }
  }

  icon_map.clear();

  QPixmap traffic_img, aggressive_img, standard_img, relaxed_img;
  QSharedPointer<QMovie> traffic_gif, aggressive_gif, standard_gif, relaxed_gif;

  loadImage("../../frogpilot/assets/active_theme/distance_icons/traffic", traffic_img, traffic_gif, QSize(btn_size, btn_size), this);
  loadImage("../../frogpilot/assets/active_theme/distance_icons/aggressive", aggressive_img, aggressive_gif, QSize(btn_size, btn_size), this);
  loadImage("../../frogpilot/assets/active_theme/distance_icons/standard", standard_img, standard_gif, QSize(btn_size, btn_size), this);
  loadImage("../../frogpilot/assets/active_theme/distance_icons/relaxed", relaxed_img, relaxed_gif, QSize(btn_size, btn_size), this);

  icon_map.insert(0, qMakePair(traffic_img, traffic_gif));
  icon_map.insert(1, qMakePair(aggressive_img, aggressive_gif));
  icon_map.insert(2, qMakePair(standard_img, standard_gif));
  icon_map.insert(3, qMakePair(relaxed_img, relaxed_gif));
}

void DistanceButton::updateState(const UIScene &scene, const FrogPilotUIScene &frogpilot_scene) {
  bool state_changed = (traffic_mode_active != frogpilot_scene.traffic_mode_enabled) ||
                       (personality != static_cast<int>(scene.personality) + 1 && !traffic_mode_active);

  if (!state_changed) {
    return;
  }

  personality = static_cast<int>(scene.personality) + 1;
  traffic_mode_active = frogpilot_scene.traffic_mode_enabled;

  update();
}

void DistanceButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  QPair<QPixmap, QSharedPointer<QMovie>> icon = icon_map.value(traffic_mode_active ? 0 : personality);
  QPixmap img = icon.first;
  QMovie *gif = icon.second.data();

  drawIcon(p, rect().center() + QPoint(UI_BORDER_SIZE / 2, 0), gif ? gif->currentPixmap() : img, Qt::transparent, 1.0);
}

NavigationTestButton::NavigationTestButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size);

  QObject::connect(this, &QPushButton::clicked, [this] {
    if (params.getBool("NavigationTestControl")) {
      params.putBool("NavigationTestControl", false);
      params.remove("NavDestination");
      params.remove("NavDestinationWaypoints");
      params.remove("NavigationTestTurnCommand");
    } else {
      emit destinationPickerRequested();
    }

    updateState();
  });
}

void NavigationTestButton::updateState() {
  const bool enabled = params.getBool("NavigationTestControl");
  std::string selected_destination = params.get("NavigationTestSelectedDestination");
  if (selected_destination.empty()) {
    selected_destination = "home";
  }
  const QString destination = enabled ? navigationTestDestinationLabel(selected_destination) : "NAV";
  if (navigation_test_enabled == enabled && navigation_test_destination == destination) {
    return;
  }

  navigation_test_enabled = enabled;
  navigation_test_destination = destination;
  update();
}

bool NavigationTestButton::navigationTestEnabled() {
  return params.getBool("NavigationTestControl");
}

void NavigationTestButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const QColor background = navigation_test_enabled ? QColor(0, 163, 108, 210) : QColor(0, 0, 0, 166);
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), QPixmap(), background, isDown() ? 0.6 : 1.0);

  p.setPen(Qt::white);
  p.setFont(InterFont(54, QFont::Bold));
  p.drawText(rect(), Qt::AlignCenter, navigation_test_destination);
}

NavigationDestinationButton::NavigationDestinationButton(const QString &label, const QString &destination_id, double latitude, double longitude, const QString &place_name, QWidget *parent) : QPushButton(parent), destination_id(destination_id), label(label), place_name(place_name), latitude(latitude), longitude(longitude) {
  setFixedSize(btn_size * 1.45, btn_size / 2);

  QObject::connect(this, &QPushButton::clicked, this, &NavigationDestinationButton::selectDestination);
}

void NavigationDestinationButton::selectDestination() {
  params.put("NavigationTestSelectedDestination", destination_id.toStdString());
  if (destination_id == "share") {
    const std::string selection_token = std::to_string(QDateTime::currentMSecsSinceEpoch());
    params.put("NavigationTestShareSelectionToken", selection_token);
    params.remove("NavDestination");
  } else {
    params.put("NavDestination", navigationTestDestinationJson(latitude, longitude, place_name).toStdString());
  }
  params.remove("NavDestinationWaypoints");
  params.remove("NavigationTestTurnCommand");
  params.putBool("NavigationTestControl", true);
  emit destinationSelected();
}

void NavigationDestinationButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const qreal opacity = isDown() ? 0.75 : 1.0;
  p.setOpacity(opacity);
  p.setBrush(isDown() ? QColor(45, 45, 45, 230) : QColor(0, 0, 0, 205));
  p.setPen(QPen(QColor(255, 255, 255, 65), 3));
  p.drawRoundedRect(rect().adjusted(2, 2, -2, -2), height() / 2, height() / 2);
  p.setOpacity(1.0);

  p.setPen(Qt::white);
  p.setFont(InterFont(36, QFont::Bold));
  p.drawText(rect(), Qt::AlignCenter, label);
}
