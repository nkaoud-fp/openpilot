#include "selfdrive/ui/qt/onroad/onroad_home.h"

#include <QPainter>
#include <QStackedLayout>

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_helpers.h"
#include "selfdrive/ui/qt/maps/map_panel.h"
#endif

#include "selfdrive/ui/qt/util.h"

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  //QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout = new QVBoxLayout(this); 
  main_layout->setMargin(UI_BORDER_SIZE);
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

  // FrogPilot variables
  frogpilot_onroad = new FrogPilotOnroadWindow(this);
  frogpilot_onroad->setAttribute(Qt::WA_TransparentForMouseEvents, true);
}

void OnroadWindow::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);

  frogpilot_onroad->setGeometry(rect());
}

void OnroadWindow::updateState(const UIState &s, const FrogPilotUIState &fs) {
  if (!s.scene.started) {
    return;
  }

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  alerts->updateState(s, fs);
  nvg->updateState(s, fs);

  QColor bgColor = bg_colors[s.status];
  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
  
  // headless_mode Expand the TOP boarder
  if (fs.frogpilot_toggles.value("headless_mode").toBool() != prev_headless_mode_state) { // prev_headless_mode_state needs to be a new member variable
    if (fs.frogpilot_toggles.value("headless_mode").toBool()) {
      main_layout->setContentsMargins(UI_BORDER_SIZE/2, (UI_BORDER_SIZE * 25) + (UI_BORDER_SIZE/2), UI_BORDER_SIZE/2, UI_BORDER_SIZE/2); // devide by 2 to get thin boarder
    } else {
      main_layout->setMargin(UI_BORDER_SIZE);
    }
    prev_headless_mode_state = fs.frogpilot_toggles.value("headless_mode").toBool() ; // Update the stored state
    //shouldUpdate = true; // Request a repaint because margins changed
    update(); // Request a repaint because margins changed
  }
  
  // FrogPilot variables
  frogpilot_onroad->bg = bg;
  frogpilot_onroad->fps = nvg->fps;

  nvg->frogpilot_nvg->alertHeight = alerts->alertHeight;

  frogpilot_onroad->updateState(s, fs);
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  FrogPilotUIState &fs = *frogpilotUIState();
  QJsonObject &frogpilot_toggles = fs.frogpilot_toggles;
  SubMaster &fpsm = *(fs.sm);

  if (fpsm["frogpilotPlan"].getFrogpilotPlan().getSpeedLimitChanged() && nvg->frogpilot_nvg->newSpeedLimitRect.contains(e->pos())) {
    fs.params_memory.putBool("SpeedLimitAccepted", true);
    return;
  }

#ifdef ENABLE_MAPS
  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    bool show_map = !sidebarVisible;
    map->setVisible(show_map && !map->isVisible());
    if (map->isVisible() && frogpilot_toggles.value("full_map").toBool()) {
      nvg->frogpilot_nvg->bigMapOpen = false;

      map->setFixedSize(this->size());

      alerts->setVisible(false);
      nvg->setVisible(false);
    } else if (map->isVisible() && frogpilot_toggles.value("big_map").toBool()) {
      nvg->frogpilot_nvg->bigMapOpen = true;

      map->setFixedWidth(topWidget(this)->width() * 3 / 4 - UI_BORDER_SIZE);

      alerts->setVisible(true);
      nvg->setVisible(true);
    } else {
      nvg->frogpilot_nvg->bigMapOpen = false;

      map->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);

      alerts->setVisible(true);
      nvg->setVisible(true);
    }
    nvg->screen_recorder->setVisible(!map->isVisible() && frogpilot_toggles.value("screen_recorder").toBool());
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

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));

  // Add these two lines to get fs and frogpilot_toggles
  FrogPilotUIState &fs = *frogpilotUIState();
  QJsonObject &frogpilot_toggles = fs.frogpilot_toggles;

  // Draw the top black rectangle in headless mode to make the top area over the boarder black, covering anything that might be there.
  if (frogpilot_toggles.value("headless_mode").toBool()) {
    // Draw the top black rectangle to make the top area over the boarder black, covering anything that might be there.
    QRect screenRect = this->rect();
    p.fillRect(QRect(0, 0, screenRect.width(), UI_BORDER_SIZE * 25), Qt::black);
  }
}
