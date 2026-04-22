#include "frogpilot/ui/qt/onroad/frogpilot_buttons.h"

#include <array>

#include <QDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMouseEvent>
#include <QPainter>

#include "selfdrive/ui/qt/util.h"

namespace {

struct NavigationTestDestination {
  const char *id;
  const char *button_text;
  const char *place_name;
  double latitude;
  double longitude;
};

constexpr std::array<NavigationTestDestination, 3> navigation_test_destinations = {{
  {"home", "Home", "Navigation test - Home", 24.675764, 46.581478},
  {"work", "Work", "Navigation test - Work", 24.714778, 46.683775},
  {"school", "School", "Navigation test - School", 24.781423, 46.622246},
}};

QString navigationTestDestinationLabel(const std::string &destination) {
  if (destination == "home") return "HOM";
  if (destination == "work") return "WRK";
  if (destination == "school") return "SCH";
  return "NAV";
}

QByteArray navigationTestDestinationJson(const NavigationTestDestination &destination) {
  QJsonObject destination_json{
    {"latitude", destination.latitude},
    {"longitude", destination.longitude},
    {"place_name", destination.place_name},
  };
  return QJsonDocument(destination_json).toJson(QJsonDocument::Compact);
}

class NavigationDestinationDialog : public QDialog {
public:
  explicit NavigationDestinationDialog(QWidget *parent) : QDialog(parent ? parent->window() : nullptr) {
    setModal(true);
    setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    setAttribute(Qt::WA_TranslucentBackground);

    if (QWidget *window = parentWidget()) {
      setGeometry(window->rect());
    } else {
      resize(1920, 1080);
    }
  }

  const NavigationTestDestination *selectedDestination() const {
    return selected_destination;
  }

private:
  void paintEvent(QPaintEvent *event) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(0, 0, 0, 185));

    p.translate(width(), 0);
    p.rotate(90);

    QRect logical_rect(0, 0, height(), width());
    p.setPen(Qt::white);
    p.setFont(InterFont(46, QFont::DemiBold));
    p.drawText(logical_rect.adjusted(0, 150, 0, 0), Qt::AlignHCenter | Qt::AlignTop, tr("Choose destination"));

    p.setFont(InterFont(44, QFont::DemiBold));
    for (int i = 0; i < static_cast<int>(navigation_test_destinations.size()); ++i) {
      const QRect button_rect = destinationButtonRect(i);
      p.setPen(QPen(QColor(255, 255, 255, 95), 4));
      p.setBrush(QColor(25, 25, 25, 230));
      p.drawRoundedRect(button_rect, 24, 24);

      p.setPen(Qt::white);
      p.drawText(button_rect, Qt::AlignCenter, tr(navigation_test_destinations[i].button_text));
    }
  }

  void mousePressEvent(QMouseEvent *event) override {
    const QPoint logical_point(event->pos().y(), width() - event->pos().x());
    for (int i = 0; i < static_cast<int>(navigation_test_destinations.size()); ++i) {
      if (destinationButtonRect(i).contains(logical_point)) {
        selected_destination = &navigation_test_destinations[i];
        accept();
        return;
      }
    }
    reject();
  }

  QRect destinationButtonRect(int index) const {
    const int logical_width = height();
    const int logical_height = width();
    const int button_width = 520;
    const int button_height = 150;
    const int spacing = 36;
    const int destination_count = static_cast<int>(navigation_test_destinations.size());
    const int total_height = (button_height * destination_count) + (spacing * (destination_count - 1));
    const int x = (logical_width - button_width) / 2;
    const int y = (logical_height - total_height) / 2 + index * (button_height + spacing);
    return QRect(x, y, button_width, button_height);
  }

  const NavigationTestDestination *selected_destination = nullptr;
};

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
    } else if (selectDestination()) {
      params.putBool("NavigationTestControl", true);
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

bool NavigationTestButton::selectDestination() {
  NavigationDestinationDialog dialog(this);
  if (dialog.exec() != QDialog::Accepted || dialog.selectedDestination() == nullptr) {
    return false;
  }

  const NavigationTestDestination *destination = dialog.selectedDestination();
  params.put("NavigationTestSelectedDestination", destination->id);
  params.put("NavDestination", navigationTestDestinationJson(*destination).toStdString());
  params.remove("NavDestinationWaypoints");
  params.remove("NavigationTestTurnCommand");
  return true;
}

void NavigationTestButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  const QColor background = navigation_test_enabled ? QColor(0, 163, 108, 210) : QColor(0, 0, 0, 166);
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), QPixmap(), background, isDown() ? 0.6 : 1.0);

  p.setPen(Qt::white);
  p.setFont(InterFont(54, QFont::Bold));
  p.drawText(rect().adjusted(0, 18, 0, 0), Qt::AlignCenter, navigation_test_destination);

  p.setFont(InterFont(27, QFont::DemiBold));
  p.setPen(QColor(255, 255, 255, navigation_test_enabled ? 240 : 175));
  p.drawText(rect().adjusted(0, 104, 0, 0), Qt::AlignHCenter | Qt::AlignTop, navigation_test_enabled ? tr("ON") : tr("OFF"));
}
