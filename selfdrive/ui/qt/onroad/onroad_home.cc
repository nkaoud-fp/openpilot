#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QApplication>
#include <QPainter>
#include <QStackedLayout>

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/maps/map_panel.h"
#endif

#include "selfdrive/ui/qt/util.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  //main_layout->setMargin(UI_BORDER_SIZE); // NIZ remove
  main_layout = new QVBoxLayout(this); // MODIFIED: Assign to member 'main_layout'
  //main_layout->setContentsMargins(UI_BORDER_SIZE, UI_BORDER_SIZE * 23, UI_BORDER_SIZE, UI_BORDER_SIZE);  // NIZ add
  main_layout->setMargin(UI_BORDER_SIZE); // NIZ remove?
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraWidget("camerad", VISION_STREAM_ROAD, true, this);
    split->insertWidget(0, arCam);
  }

  if (getenv("MAP_RENDER_VIEW")) {
    CameraWidget *map_render = new CameraWidget("navd", VISION_STREAM_MAP, false, this);
    split->insertWidget(0, map_render);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::primeChanged, this, &OnroadWindow::primeChanged);
}

void OnroadWindow::updateState(const UIState &s) {
  if (!s.scene.started) {
    return;
  }

  if (s.scene.map_on_left || s.scene.full_map) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  alerts->updateState(s);
  nvg->updateState(alerts->alert_height, s);

  bool shouldUpdate = false;

  bool new_custom_ui_active_state = s.scene.hide_map_icon;
  if (this->custom_ui_active != new_custom_ui_active_state) {
    this->custom_ui_active = new_custom_ui_active_state;
    if (this->custom_ui_active) {
      main_layout->setContentsMargins(UI_BORDER_SIZE, UI_BORDER_SIZE * 23, UI_BORDER_SIZE, UI_BORDER_SIZE); // MODIFIED
    } else {
      main_layout->setContentsMargins(UI_BORDER_SIZE, UI_BORDER_SIZE, UI_BORDER_SIZE, UI_BORDER_SIZE); // MODIFIED
    }
    shouldUpdate = true;
  }

  QColor bgColor = bg_colors[s.status];
  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    shouldUpdate = true;
  }

  // FrogPilot variables
  const UIScene &scene = s.scene;

  acceleration = scene.acceleration;
  accelerationJerk = scene.acceleration_jerk;
  accelerationJerkDifference = scene.acceleration_jerk_difference;
  blindSpotLeft = scene.blind_spot_left;
  blindSpotRight = scene.blind_spot_right;
  fps = scene.fps;
  friction = scene.friction;
  latAccel = scene.lat_accel;
  liveValid = scene.live_valid;
  showBlindspot = scene.show_blind_spot;
  showFPS = scene.show_fps;
  showJerk = scene.jerk_metrics;
  showSignal = scene.signal_metrics;
  showSteering = scene.steering_metrics;
  showTuning = scene.lateral_tuning_metrics;
  speedJerk = scene.speed_jerk;
  speedJerkDifference = scene.speed_jerk_difference;
  steer = scene.steer;
  steeringAngleDeg = scene.steering_angle_deg;
  turnSignalLeft = scene.turn_signal_left;
  turnSignalRight = scene.turn_signal_right;

  if (showBlindspot || showFPS || showJerk || showSignal || showSteering || showTuning) {
    shouldUpdate = true;
  }

  if (shouldUpdate) {
    update();
  }
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  // FrogPilot variables
  UIState *s = uiState();
  UIScene &scene = s->scene;
  QPoint pos = e->pos();

  if (scene.speed_limit_changed && nvg->newSpeedLimitRect.contains(pos)) {
    params_memory.putBool("SpeedLimitAccepted", true);
    return;
  }

#ifdef ENABLE_MAPS
  if (map != nullptr) {
    // Switch between map and sidebar when using navigate on openpilot
    bool sidebarVisible = geometry().x() > 0;
    bool show_map = scene.navigate_on_openpilot ? sidebarVisible : !sidebarVisible;
    map->setVisible(show_map && !map->isVisible());
    if (scene.big_map) {
      map->setFixedWidth(width());
    } else {
      map->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
    }
  }
#endif
  // propagation event to parent(HomeWindow)
  QWidget::mousePressEvent(e);
}

void OnroadWindow::createMapWidget() {
#ifdef ENABLE_MAPS
  auto m = new MapPanel(get_mapbox_settings());
  map = m;
  QObject::connect(m, &MapPanel::mapPanelRequested, this, &OnroadWindow::mapPanelRequested);
  QObject::connect(nvg->map_settings_btn, &MapSettingsButton::clicked, m, &MapPanel::toggleMapSettings);
  nvg->map_settings_btn->setEnabled(true);

  m->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
  split->insertWidget(0, m);
  // hidden by default, made visible when navRoute is published
  m->setVisible(false);
#endif
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && !MAPBOX_TOKEN.isEmpty()) {
      createMapWidget();
    }
  }
#endif
  alerts->clear();
}

void OnroadWindow::primeChanged(bool prime) {
#ifdef ENABLE_MAPS
  if (map && (!prime && MAPBOX_TOKEN.isEmpty())) {
    nvg->map_settings_btn->setEnabled(false);
    nvg->map_settings_btn->setVisible(false);
    map->deleteLater();
    map = nullptr;
  } else if (!map && (prime || !MAPBOX_TOKEN.isEmpty())) {
    createMapWidget();
  }
#endif
}

// niz replace full function
void OnroadWindow::paintEvent(QPaintEvent *event) {

  QPainter p(this);
  QRect CRECT = this->contentsRect(); // This rect respects main_layout's margins.

  UIState *s = uiState();
  SubMaster &sm_local = *(s->sm); // Used for sm.frame to avoid conflicts if OnroadWindow had a member 'sm'
  QColor current_bg_status_color = bg_colors[s->status];

  // Fill the background of the content area (where nvg and map will be placed by the layout)
  p.fillRect(CRECT, current_bg_status_color);

  // niz add == try after bg color
  QRect screenRect = this->rect(); // Full widget rect
  // Draw the top black rectangle, covering anything that might be there.
  //p.fillRect(QRect(0, 0, screenRect.width(), UI_BORDER_SIZE * 22), Qt::black);


  // Steering indicators (are on the extreme left/right edges of the window, but below top bar)
  if (showSteering) {
    static float smoothedSteer = 0.0;
    smoothedSteer = 0.1 * std::abs(steer) + 0.9 * smoothedSteer;
    if (std::abs(smoothedSteer - steer) < 0.01) {
      smoothedSteer = steer;
    }

    QLinearGradient gradient(CRECT.topLeft(), CRECT.bottomLeft()); // Gradient over content area
    gradient.setColorAt(0.0, bg_colors[STATUS_TRAFFIC_MODE_ACTIVE]);
    gradient.setColorAt(0.15, bg_colors[STATUS_EXPERIMENTAL_MODE_ACTIVE]);
    gradient.setColorAt(0.5, bg_colors[STATUS_CONDITIONAL_OVERRIDDEN]);
    gradient.setColorAt(0.85, bg_colors[STATUS_ENGAGED]);
    gradient.setColorAt(1.0, bg_colors[STATUS_ENGAGED]);
    QBrush brush(gradient);

    if (steeringAngleDeg != 0) {
      int visibleHeight = CRECT.height() * smoothedSteer; // Use height of the content area
      QRect rectToFill, rectToHide;

      if (steeringAngleDeg < 0) { // Left steering
        rectToFill = QRect(screenRect.x(), CRECT.y() + CRECT.height() - visibleHeight, UI_BORDER_SIZE, visibleHeight);
        rectToHide = QRect(screenRect.x(), CRECT.y(), UI_BORDER_SIZE, CRECT.height() - visibleHeight);
      } else { // Right steering
        rectToFill = QRect(screenRect.x() + screenRect.width() - UI_BORDER_SIZE, CRECT.y() + CRECT.height() - visibleHeight, UI_BORDER_SIZE, visibleHeight);
        rectToHide = QRect(screenRect.x() + screenRect.width() - UI_BORDER_SIZE, CRECT.y(), UI_BORDER_SIZE, CRECT.height() - visibleHeight);
      }
      p.fillRect(rectToFill, brush);
      p.fillRect(rectToHide, current_bg_status_color);
    }
  }

  // Blindspot / Signal indicators (fill halves of the content area)
  if (showBlindspot || showSignal) {
    static bool leftFlickerActive = false;
    static bool rightFlickerActive = false;

    std::function<QColor(bool, bool, bool&)> getBorderColor =
      [&](bool blindSpot, bool turnSignal, bool &flickerActive) -> QColor {
      if (showSignal && turnSignal) {
        if (blindSpot) {
          if (sm_local.frame % (UI_FREQ / 5) == 0) {
            flickerActive = !flickerActive;
          }
          return flickerActive ? bg_colors[STATUS_TRAFFIC_MODE_ACTIVE] : bg_colors[STATUS_CONDITIONAL_OVERRIDDEN];
        } else if (sm_local.frame % (UI_FREQ / 2) == 0) {
          flickerActive = !flickerActive;
        }
        return flickerActive ? bg_colors[STATUS_CONDITIONAL_OVERRIDDEN] : current_bg_status_color;
      } else if (showBlindspot && blindSpot) {
        return bg_colors[STATUS_TRAFFIC_MODE_ACTIVE];
      } else {
        return current_bg_status_color;
      }
    };

    QColor borderColorLeft = getBorderColor(blindSpotLeft, turnSignalLeft, leftFlickerActive);
    QColor borderColorRight = getBorderColor(blindSpotRight, turnSignalRight, rightFlickerActive);

    p.fillRect(CRECT.x(), CRECT.y(), CRECT.width() / 2, CRECT.height(), borderColorLeft);
    p.fillRect(CRECT.x() + CRECT.width() / 2, CRECT.y(), CRECT.width() / 2, CRECT.height(), borderColorRight);
  }

  // Text overlays (logics, FPS) are drawn relative to nvg's geometry
  QRect text_area = nvg->geometry(); // Area nvg occupies, in OnroadWindow's coordinates

  QString logicsDisplayString;
  if (showJerk) {
    if (bg == bg_colors[STATUS_ENGAGED] || bg == bg_colors[STATUS_TRAFFIC_MODE_ACTIVE]) {
      maxAcceleration = std::max(maxAcceleration, acceleration);
    }
    maxAccelTimer = maxAcceleration == acceleration && maxAcceleration != 0 ? UI_FREQ * 5 : maxAccelTimer - 1;

    logicsDisplayString += QString("Acceleration: %1 %2 - ").arg(acceleration, 0, 'f', 2).arg(nvg->accelerationUnit);
    logicsDisplayString += QString("Max: %1 %2 | ").arg(maxAcceleration, 0, 'f', 2).arg(nvg->accelerationUnit);
    logicsDisplayString += QString("Acceleration Jerk: %1 | ").arg(accelerationJerk, 0, 'f', 2);
    logicsDisplayString += QString("Speed Jerk: %1").arg(speedJerk, 0, 'f', 2);
  }
  if (showTuning) {
    if (!logicsDisplayString.isEmpty()) {
      logicsDisplayString += " | ";
    }
    logicsDisplayString += QString("Friction: %1 | ").arg(liveValid ? QString::number(friction, 'f', 2) : "Calculating...");
    logicsDisplayString += QString("Lateral Acceleration: %1").arg(liveValid ? QString::number(latAccel, 'f', 2) : "Calculating...");
  }

  if (!logicsDisplayString.isEmpty()) {
    p.save();
    p.setFont(InterFont(28, QFont::DemiBold));
    p.setRenderHint(QPainter::TextAntialiasing);
    QFontMetrics fontMetrics(p.font());

    int stringWidth = fontMetrics.horizontalAdvance(logicsDisplayString);
    int x = text_area.x() + (text_area.width() - stringWidth) / 2; // Centered in nvg area
    int y = text_area.top() + (fontMetrics.height() / 1.5);     // Near top of nvg area

    QStringList parts = logicsDisplayString.split("|");
    int current_x_offset = 0;
    for (QString part_str : parts) {
      QString current_part = part_str.trimmed();
      if (current_part.isEmpty()) continue;

      // Determine if a separator is needed (if not the last actual part)
      bool addSeparator = (&part_str != &parts.last() && !parts.last().trimmed().isEmpty());
      if (part_str == parts.last() && parts.size() > 1) { // Check if it's the last segment from the split
        QString preceding_text = logicsDisplayString.left(logicsDisplayString.lastIndexOf(part_str));
        if (!preceding_text.trimmed().endsWith("|")) { // if original string didn't end with | for this part
          addSeparator = false;
        } else if (logicsDisplayString.endsWith(part_str)) { // if it is the very last part
             addSeparator = false;
        }
      }


      QString textToDraw = current_part;
      if (addSeparator) {
        textToDraw += " | ";
      }

      if (current_part.startsWith("Max:") && maxAccelTimer > 0) {
        // Split "Acceleration: val - " and "Max: val"
        QString accel_part = QString("Acceleration: %1 %2 - ").arg(acceleration, 0, 'f', 2).arg(nvg->accelerationUnit);
        QString max_part = QString("Max: %1 %2").arg(maxAcceleration, 0, 'f', 2).arg(nvg->accelerationUnit);

        p.setPen(whiteColor());
        p.drawText(x + current_x_offset, y, accel_part);
        current_x_offset += fontMetrics.horizontalAdvance(accel_part);

        p.setPen(redColor());
        p.drawText(x + current_x_offset, y, max_part);
        current_x_offset += fontMetrics.horizontalAdvance(max_part);
        if (addSeparator) {
             p.setPen(whiteColor()); // Separator color
             p.drawText(x + current_x_offset, y, " | ");
             current_x_offset += fontMetrics.horizontalAdvance(" | ");
        }

      } else if (current_part.startsWith("Acceleration Jerk:") && accelerationJerkDifference != 0) {
        QString baseText = QString("Acceleration Jerk: %1").arg(accelerationJerk, 0, 'f', 2);
        p.setPen(whiteColor());
        p.drawText(x + current_x_offset, y, baseText);
        current_x_offset += fontMetrics.horizontalAdvance(baseText);

        QString diffText = QString(" (%1)").arg(accelerationJerkDifference, 0, 'f', 2);
        p.setPen(redColor());
        p.drawText(x + current_x_offset, y, diffText);
        current_x_offset += fontMetrics.horizontalAdvance(diffText);
         if (addSeparator) {
             p.setPen(whiteColor());
             p.drawText(x + current_x_offset, y, " | ");
             current_x_offset += fontMetrics.horizontalAdvance(" | ");
        }

      } else if (current_part.startsWith("Speed Jerk:") && speedJerkDifference != 0) {
        QString baseText = QString("Speed Jerk: %1").arg(speedJerk, 0, 'f', 2);
        p.setPen(whiteColor());
        p.drawText(x + current_x_offset, y, baseText);
        current_x_offset += fontMetrics.horizontalAdvance(baseText);

        QString diffText = QString(" (%1)").arg(speedJerkDifference, 0, 'f', 2);
        p.setPen(redColor());
        p.drawText(x + current_x_offset, y, diffText);
        current_x_offset += fontMetrics.horizontalAdvance(diffText);
        if (addSeparator && showTuning) { // Original logic had conditional separator
             p.setPen(whiteColor());
             p.drawText(x + current_x_offset, y, " | ");
             current_x_offset += fontMetrics.horizontalAdvance(" | ");
        } else if (addSeparator) { // if not showTuning but separator needed
             p.setPen(whiteColor());
             p.drawText(x + current_x_offset, y, " | ");
             current_x_offset += fontMetrics.horizontalAdvance(" | ");
        }


      } else {
        p.setPen(whiteColor());
        p.drawText(x + current_x_offset, y, textToDraw);
        current_x_offset += fontMetrics.horizontalAdvance(textToDraw);
      }
    }
    p.restore();
  }

  if (showFPS) {
    qint64 currentMillis = QDateTime::currentMSecsSinceEpoch();
    static std::queue<std::pair<qint64, float>> fpsQueue;
    static float avgFPS = 0.0;
    static float maxFPS = 0.0;
    static float minFPS = 99.9;

    minFPS = std::min(minFPS, fps);
    maxFPS = std::max(maxFPS, fps);
    fpsQueue.push({currentMillis, fps});
    while (!fpsQueue.empty() && currentMillis - fpsQueue.front().first > 60000) {
      fpsQueue.pop();
    }
    if (!fpsQueue.empty()) {
      float totalFPS = 0.0;
      for (auto tempQueue = fpsQueue; !tempQueue.empty(); tempQueue.pop()) {
        totalFPS += tempQueue.front().second;
      }
      avgFPS = totalFPS / fpsQueue.size();
    }

    QString fpsDisplayString = QString("FPS: %1 | Min: %2 | Max: %3 | Avg: %4")
        .arg(qRound(fps))
        .arg(qRound(minFPS))
        .arg(qRound(maxFPS))
        .arg(qRound(avgFPS));

    p.setFont(InterFont(28, QFont::DemiBold));
    p.setRenderHint(QPainter::TextAntialiasing);
    p.setPen(whiteColor());
    QFontMetrics fontMetrics(p.font());
    int textWidth = fontMetrics.horizontalAdvance(fpsDisplayString);
    int xPos = text_area.x() + (text_area.width() - textWidth) / 2; // Centered in nvg area
    int yPos = text_area.bottom() - 5 - UI_BORDER_SIZE; // Near bottom of nvg area, above its border
    p.drawText(xPos, yPos, fpsDisplayString);
  }



  
}
