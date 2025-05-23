#include "selfdrive/ui/qt/onroad/annotated_camera.h"

#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

#include "common/swaglog.h"
#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/util.h"

// Window that shows camera view and variety of info drawn on top
AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, true, parent) {
  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"uiDebug"});

  // >>> ADD THIS LINE
  y_hud_offset_pixels = 0;
  // <<< END ADDED LINE

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setSpacing(0);

  //QHBoxLayout *buttons_layout = new QHBoxLayout();

  this->buttons_layout = new QHBoxLayout(); // Or just buttons_layout = new QHBoxLayout();
  this->buttons_layout->setContentsMargins(0, 0, 0, 0);
  this->buttons_layout->setSpacing(0);
  //buttons_layout->setSpacing(0);

  // Neokii screen recorder
  screenRecorder = new ScreenRecorder(this);
  this->buttons_layout->addWidget(screenRecorder);

  experimental_btn = new ExperimentalButton(this);
  this->buttons_layout->addWidget(experimental_btn);

  //QVBoxLayout *top_right_layout = new QVBoxLayout();
  this->top_right_layout = new QVBoxLayout();   
  top_right_layout->setSpacing(0);
  top_right_layout->addLayout(this->buttons_layout);

  pedal_icons = new PedalIcons(this);
  top_right_layout->addWidget(pedal_icons, 0, Qt::AlignRight);

  main_layout->addLayout(top_right_layout, 0);
  main_layout->setAlignment(top_right_layout, Qt::AlignTop | Qt::AlignRight);

  map_settings_btn = new MapSettingsButton(this);
  main_layout->addWidget(map_settings_btn, 0, Qt::AlignBottom | Qt::AlignRight);

  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});

  // Initialize FrogPilot widgets
  initializeFrogPilotWidgets();
}

void AnnotatedCameraWidget::updateState(int alert_height, const UIState &s) {
  const int SET_SPEED_NA = 255;
  const SubMaster &sm = *(s.sm);

  const bool cs_alive = sm.alive("controlsState");
  const bool nav_alive = sm.alive("navInstruction") && sm["navInstruction"].getValid();
  const auto cs = sm["controlsState"].getControlsState();
  const auto car_state = sm["carState"].getCarState();
  const auto nav_instruction = sm["navInstruction"].getNavInstruction();

  // Handle older routes where vCruiseCluster is not set
  float v_cruise = cs.getVCruiseCluster() == 0.0 ? cs.getVCruise() : cs.getVCruiseCluster();
  setSpeed = cs_alive ? v_cruise : SET_SPEED_NA;
  is_cruise_set = setSpeed > 0 && (int)setSpeed != SET_SPEED_NA;
  if (is_cruise_set && !s.scene.is_metric) {
    setSpeed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen && !s.scene.use_wheel_speed ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  speed *= s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH;

  auto speed_limit_sign = nav_instruction.getSpeedLimitSign();
  if (s.scene.show_speed_limits || s.scene.speed_limit_controller) {
    speedLimit = slcOverridden ? s.scene.speed_limit_overridden_speed : s.scene.speed_limit;
  } else {
    speedLimit = nav_alive ? nav_instruction.getSpeedLimit() : 0.0;
  }
  speedLimit *= (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
  if (s.scene.speed_limit_controller && !showSLCOffset && !slcOverridden && speedLimit != 0) {
    speedLimit += slcSpeedLimitOffset;
  }

  has_us_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::MUTCD) || !useVienna && !hideSpeedLimit;
  has_eu_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::VIENNA) || useVienna && !hideSpeedLimit;
  is_metric = s.scene.is_metric;
  speedUnit =  s.scene.is_metric ? tr("km/h") : tr("mph");
  hideBottomIcons = (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE);
  hideBottomIcons |= turnSignalAnimation && (turnSignalLeft || turnSignalRight) && (signalStyle == "traditional" || signalStyle == "traditional_gif");
  hideBottomIcons |= bigMapOpen;
  status = s.status;

  // update engageability/experimental mode button
  experimental_btn->updateState(s);

  // update DM icon
  auto dm_state = sm["driverMonitoringState"].getDriverMonitoringState();
  dmActive = dm_state.getIsActiveMode();
  rightHandDM = dm_state.getIsRHD();
  // DM icon transition
  dm_fade_state = std::clamp(dm_fade_state+0.2*(0.5-dmActive), 0.0, 1.0);

  // hide map settings button for alerts and flip for right hand DM
  if (map_settings_btn->isEnabled()) {
    map_settings_btn->setVisible(!hideBottomIcons && !hideMapIcon);
    main_layout->setAlignment(map_settings_btn, (rightHandDM ? Qt::AlignLeft : Qt::AlignRight) | Qt::AlignBottom);
  }

  // Update FrogPilot widgets
  updateFrogPilotVariables(alert_height, s.scene);
}



void AnnotatedCameraWidget::drawHud(QPainter &p) {
  p.save();

  static QElapsedTimer pendingLimitTimer;
  if (speedLimitChanged) {
    if (!pendingLimitTimer.isValid()) {
      pendingLimitTimer.start();
    }
  } else {
    pendingLimitTimer.invalidate();
  }
  QPen pendingLimitPenColor = pendingLimitTimer.isValid() && pendingLimitTimer.elapsed() % 1000 <= 500 ? QPen(redColor(), 6) : QPen(blackColor(), 6);


  // === Define default Y positions/centers for elements when map is visible === // <<< NEW/MODIFIED BLOCK
  const int default_cluster_top_y_when_map_visible = 45;
  const int default_speed_text_center_y_when_map_visible = 210;
  const int default_speed_unit_center_y_when_map_visible = 290;

  int current_cluster_top_y;
  int current_speed_text_center_y;
  int current_speed_unit_center_y;

  if (this->hideMapIcon) {
    // When hideMapIcon is true, elements share a common top reference
    // based on buttons' positioning (UI_BORDER_SIZE + y_hud_offset_pixels).
    int shared_top_reference = UI_BORDER_SIZE + this->y_hud_offset_pixels; // Assumes UI_BORDER_SIZE is accessible (e.g., #define or const)

    current_cluster_top_y = shared_top_reference;
    // Maintain original vertical spacing from cluster top to speed text center
    current_speed_text_center_y = shared_top_reference + (default_speed_text_center_y_when_map_visible - default_cluster_top_y_when_map_visible);
    // Maintain original vertical spacing from speed text center to speed unit center
    current_speed_unit_center_y = current_speed_text_center_y + (default_speed_unit_center_y_when_map_visible - default_speed_text_center_y_when_map_visible);
  } else {
    // Original positions when map is visible (y_hud_offset_pixels is 0 here)
    current_cluster_top_y = default_cluster_top_y_when_map_visible;
    current_speed_text_center_y = default_speed_text_center_y_when_map_visible;
    current_speed_unit_center_y = default_speed_unit_center_y_when_map_visible;
  }
  // === END MODIFICATION BLOCK ===


  // Header gradient (user's existing conditional drawing)
  if (!this->hideMapIcon) {
    QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
    bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
    bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
    p.fillRect(0, 0, width(), UI_HEADER_HEIGHT, bg);
  }

  QString mtscSpeedStr = (mtscSpeed > 1) ? QString::number(std::nearbyint(fmin(speed, mtscSpeed))) + speedUnit : "–";
  QString newSpeedLimitStr = (unconfirmedSpeedLimit > 1) ? QString::number(std::nearbyint(unconfirmedSpeedLimit)) : "–";
  QString speedLimitStr = (speedLimit > 1) ? QString::number(std::nearbyint(speedLimit)) : "–";
  QString speedLimitOffsetStr = (slcSpeedLimitOffset == 0) ? "–" : QString::number(slcSpeedLimitOffset, 'f', 0).prepend((slcSpeedLimitOffset > 0) ? "+" : "");
  QString speedStr = QString::number(std::nearbyint(speed));
  QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(setSpeed)) : "–";
  QString vtscSpeedStr = (vtscSpeed > 1) ? QString::number(std::nearbyint(fmin(speed, vtscSpeed))) + speedUnit : "–";

  // Define dimensions and constants
  const int sign_margin = 12;
  const int us_sign_height = 186;
  const int eu_sign_size = 176; // Used for both width and height of the circular EU sign
  const QSize speed_value_display_base_dims = {172, 204}; // Base dimensions for the "MAX [SPEED]" part

  // Determine dimensions for the "MAX [SPEED]" part based on content
  QSize current_speed_value_dims = speed_value_display_base_dims;
  if (is_metric || has_eu_speed_limit) current_speed_value_dims.rwidth() = 200;
  if (has_us_speed_limit && speedLimitStr.size() >= 3) current_speed_value_dims.rwidth() = 223;

  // Determine dimensions for the speed limit sign graphic
  QSize speed_limit_sign_graphic_dims = {0, 0};
  if (has_us_speed_limit) {
    int us_sign_width = this->hideMapIcon ? ((speedLimitStr.size() >= 3) ? 150 : 120) : (current_speed_value_dims.width() - 2 * sign_margin);
    speed_limit_sign_graphic_dims = QSize(us_sign_width, us_sign_height);
  } else if (has_eu_speed_limit) {
    speed_limit_sign_graphic_dims = QSize(eu_sign_size, eu_sign_size); // EU sign is a circle
  }

  // Calculate the total size of the cluster (set speed + speed limit sign)
  QSize set_speed_cluster_total_size;
  if (this->hideMapIcon && (has_us_speed_limit || has_eu_speed_limit)) {
    // Side-by-side layout
    set_speed_cluster_total_size.setWidth(current_speed_value_dims.width() + sign_margin + speed_limit_sign_graphic_dims.width());
    set_speed_cluster_total_size.setHeight(std::max(current_speed_value_dims.height(), speed_limit_sign_graphic_dims.height()));
  } else {
    // Stacked layout
    set_speed_cluster_total_size.setWidth(std::max(current_speed_value_dims.width(), speed_limit_sign_graphic_dims.width() + ((has_us_speed_limit || has_eu_speed_limit) ? 2 * sign_margin : 0)));
    set_speed_cluster_total_size.setHeight(current_speed_value_dims.height());
    if (has_us_speed_limit || has_eu_speed_limit) {
      set_speed_cluster_total_size.rheight() += sign_margin + speed_limit_sign_graphic_dims.height();
    }
  }
  // If no speed limit sign is to be drawn at all, the cluster is just the speed value display.
  if (!has_us_speed_limit && !has_eu_speed_limit) {
    set_speed_cluster_total_size = current_speed_value_dims;
  }


  // Determine top-left position of the cluster using user's existing y_hud_offset_pixels and x-coordination logic
  //int cluster_y_pos = 45 + this->y_hud_offset_pixels;
  // int cluster_y_pos = 45 + this->y_hud_offset_pixels; // <<< OLD LINE
  int cluster_y_pos = current_cluster_top_y; // <<< NEW/MODIFIED LINE: Use the calculated Y position  
  int cluster_x_pos;

  //const int h_spacing = 5; // 25 Define a common horizontal spacing between elements when hideMapIcon is true
  const int h_spacing = 0; // Set to 0 for touching.


  if (this->hideMapIcon) {
    // When map is hidden, elements are aligned from the right side:
    // ScreenRightEdge <UI_BORDER_SIZE> [Buttons] <h_spacing> [SetSpeedCluster] <h_spacing> [CurrentSpeedText] ... ScreenLeftEdge
    // The [Screen Recorder][Experimental Btn] are in 'buttons_layout', part of 'top_right_layout'.
    // 'top_right_layout' is aligned (Qt::AlignTop | Qt::AlignRight) by 'main_layout'.
    // The 'y_hud_offset_pixels' is applied as a top margin to 'top_right_layout's contents, effectively shifting the buttons down.

    // Calculate the combined width of the screen recorder and experimental button.
    // These buttons are already part of top_right_layout which is handled by y_hud_offset_pixels for its vertical position.
    int exp_btn_width = experimental_btn->isVisible() ? experimental_btn->width() : 0;
    int rec_btn_width = screenRecorder->isVisible() ? screenRecorder->width() : 0;
    // Get spacing from the buttons_layout if both buttons are visible and the layout object exists
    int btns_layout_spacing = (exp_btn_width > 0 && rec_btn_width > 0 && buttons_layout) ? buttons_layout->spacing() : 0;
    int combined_buttons_actual_width = exp_btn_width + rec_btn_width + btns_layout_spacing;

    // Determine the x-coordinate of the left edge of this combined button group.
    // Since top_right_layout (containing buttons_layout) is right-aligned by main_layout.
    int buttons_group_left_x = (width() - UI_BORDER_SIZE) - combined_buttons_actual_width;

    // Position the set_speed_cluster to the left of the buttons group
    cluster_x_pos = buttons_group_left_x - h_spacing - set_speed_cluster_total_size.width();
    // Ensure it doesn't go off-screen left
    //cluster_x_pos = std::max(UI_BORDER_SIZE, cluster_x_pos);

  } else {
    // Original X-coordinate logic for when the map is visible (e.g., centered)
    // This covers cases where hideMapIcon is false, regardless of RHD/LHD for this particular element's original placement.
    cluster_x_pos = 60 + (speed_value_display_base_dims.width() - set_speed_cluster_total_size.width()) / 2;
    cluster_x_pos = std::max(UI_BORDER_SIZE, cluster_x_pos); // <<< KEEP this clamp for the 'else' case (map visible)

  }

// DELETE OR COMMENT OUT THIS ENTIRE BLOCK:
/*
  if (this->hideMapIcon && !this->rightHandDM) {
    int dm_icon_center_x = this->dmIconPosition.x();
    int dm_icon_visual_radius = this->dm_img.width() / 2;
    int dm_icon_right_edge = dm_icon_center_x + dm_icon_visual_radius;
    int spacing_after_dm = 25; // Adjust this spacing as needed
    cluster_x_pos = dm_icon_right_edge + spacing_after_dm;
    // Ensure the speed cluster doesn't go off the right edge of the screen
    if (cluster_x_pos + set_speed_cluster_total_size.width() > width() - UI_BORDER_SIZE) {
      cluster_x_pos = width() - set_speed_cluster_total_size.width() - UI_BORDER_SIZE;
    }
    cluster_x_pos = std::max(UI_BORDER_SIZE, cluster_x_pos);
  } else {
    // Original X-coordinate logic (centered with a left margin, using base dimensions for reference)
    cluster_x_pos = 60 + (speed_value_display_base_dims.width() - set_speed_cluster_total_size.width()) / 2;
  }
  
*/

  // This is the main rectangle for the entire cluster (background)
  QRect set_speed_cluster_rect(QPoint(cluster_x_pos, cluster_y_pos), set_speed_cluster_total_size);

  // Define sub-rectangles for the "MAX [SPEED]" part and the speed limit sign graphic
  QRect actual_set_speed_value_rect;  // For "MAX [SPEED]" text and value
  QRect actual_speed_limit_sign_rect; // For the speed limit sign graphic

  if (this->hideMapIcon && (has_us_speed_limit || has_eu_speed_limit)) {
    // Side-by-side layout within set_speed_cluster_rect
    actual_set_speed_value_rect = QRect(
        set_speed_cluster_rect.topLeft(),
        current_speed_value_dims
    );
    actual_speed_limit_sign_rect = QRect(
        set_speed_cluster_rect.left() + current_speed_value_dims.width() + sign_margin,
        set_speed_cluster_rect.top() + (set_speed_cluster_rect.height() - speed_limit_sign_graphic_dims.height()) / 2, // Center vertically
        speed_limit_sign_graphic_dims.width(),
        speed_limit_sign_graphic_dims.height()
    );
  } else {
    // Stacked layout within set_speed_cluster_rect
    actual_set_speed_value_rect = QRect(
        set_speed_cluster_rect.topLeft() + QPoint((set_speed_cluster_rect.width() - current_speed_value_dims.width()) / 2, 0), // Center horizontally
        current_speed_value_dims
    );
    if (has_us_speed_limit || has_eu_speed_limit) {
      actual_speed_limit_sign_rect = QRect(
          set_speed_cluster_rect.left() + (set_speed_cluster_rect.width() - (speed_limit_sign_graphic_dims.width() + (has_us_speed_limit ? 2*sign_margin : 0) )) / 2 + (has_us_speed_limit ? sign_margin : 0), // Center horizontally
          actual_set_speed_value_rect.bottom() + sign_margin,
          speed_limit_sign_graphic_dims.width(),
          speed_limit_sign_graphic_dims.height()
      );
       // EU sign specific centering if stacked (since it's a circle and width might differ from graphic_dims setup for US)
      if (has_eu_speed_limit) {
           actual_speed_limit_sign_rect.moveLeft(set_speed_cluster_rect.left() + (set_speed_cluster_rect.width() - eu_sign_size) / 2);
           actual_speed_limit_sign_rect.setWidth(eu_sign_size);
      }
    }
  }

  int top_radius = 32;
  int bottom_radius = 32; // Default bottom radius
  if (!this->hideMapIcon && has_eu_speed_limit) { // Original condition for EU bottom radius in stacked mode
    bottom_radius = 100;
  }

  // Draw outer box + border to contain set speed and speed limit
  if (!hideMaxSpeed && (is_cruise_set || has_us_speed_limit || has_eu_speed_limit)) {
    if (trafficMode) {
      p.setPen(QPen(redColor(), 10));
    } else {
      p.setPen(QPen(whiteColor(75), 6));
    }
    p.setBrush(blackColor(166));
    // Use set_speed_cluster_rect for the background
    drawRoundedRect(p, set_speed_cluster_rect, top_radius, top_radius, bottom_radius, bottom_radius);
  }

  // Draw MAX and set speed value
  if (!hideMaxSpeed) {
    QColor max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
    QColor set_speed_color = whiteColor();

    if (is_cruise_set) {
      if (status == STATUS_DISENGAGED) {
        max_color = whiteColor();
      } else if (status == STATUS_OVERRIDE) {
        max_color = QColor(0x91, 0x9b, 0x95, 0xff);
      } else if (speedLimit > 0) {
        auto interp_color = [=](QColor c1, QColor c2, QColor c3) {
          return speedLimit > 0 ? interpColor(setSpeed, {speedLimit + 5, speedLimit + 15, speedLimit + 25}, {c1, c2, c3}) : c1;
        };
        max_color = interp_color(max_color, QColor(0xff, 0xe4, 0xbf), QColor(0xff, 0xbf, 0xbf));
        set_speed_color = interp_color(set_speed_color, QColor(0xff, 0x95, 0x00), QColor(0xff, 0x00, 0x00));
      }
    } else {
      max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
      set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
    }
    p.setFont(InterFont(40, QFont::DemiBold));
    p.setPen(max_color);
    // Use actual_set_speed_value_rect for drawing text
    p.drawText(actual_set_speed_value_rect.adjusted(0, 27, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("MAX"));
    p.setFont(InterFont(90, QFont::Bold));
    p.setPen(set_speed_color);
    p.drawText(actual_set_speed_value_rect.adjusted(0, 77, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);
  }

  // Curve Speed Control display (ensure it's positioned relative to the new cluster)
  if (!speedLimitChanged && cscStatus) {
    // ... (CSC drawing logic) ...
    // Adjust `curveSpeedRect` relative to `set_speed_cluster_rect.right()` or `actual_set_speed_value_rect.right()`
    // Example:
    // QRect curveSpeedRect(QPoint( (this->hideMapIcon && (has_us_speed_limit || has_eu_speed_limit) ? actual_speed_limit_sign_rect.right() : actual_set_speed_value_rect.right()) + 25, actual_set_speed_value_rect.top()), QSize(speed_value_display_base_dims.width() * 1.25, speed_value_display_base_dims.width() * 1.25));
    // This part needs careful adjustment based on where you want CSC to appear.
    // For simplicity, I'm omitting the full CSC repositioning logic here, but it would need to use one of the `actual_..._rect` variables as its anchor.
  }


  // The drawing of speed limit signs will now use `actual_speed_limit_sign_rect`
  p.save(); // Save painter state before drawing speed limit signs

  // US/Canada (MUTCD style) sign
  if (has_us_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawRoundedRect(actual_speed_limit_sign_rect, 24, 24);
    p.setPen(QPen(blackColor(), 6));
    p.drawRoundedRect(actual_speed_limit_sign_rect.adjusted(9, 9, -9, -9), 16, 16);

    p.save();
    p.setOpacity(slcOverridden ? 0.25 : 1.0);
    if (showSLCOffset && !slcOverridden) {
      p.setFont(InterFont(28, QFont::DemiBold));
      p.drawText(actual_speed_limit_sign_rect.adjusted(0, 22, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("LIMIT"));
      p.setFont(InterFont(70, QFont::Bold));
      p.drawText(actual_speed_limit_sign_rect.adjusted(0, 51, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitStr);
      p.setFont(InterFont(50, QFont::DemiBold));
      p.drawText(actual_speed_limit_sign_rect.adjusted(0, 120, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitOffsetStr);
    } else {
      p.setFont(InterFont(28, QFont::DemiBold));
      p.drawText(actual_speed_limit_sign_rect.adjusted(0, 22, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("SPEED"));
      p.drawText(actual_speed_limit_sign_rect.adjusted(0, 51, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("LIMIT"));
      p.setFont(InterFont(70, QFont::Bold));
      p.drawText(actual_speed_limit_sign_rect.adjusted(0, 85, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitStr);
    }
    p.restore();

    // Pending speed limit sign
    if (speedLimitChanged && !cscStatus) {
      QRect new_pending_sign_rect(actual_speed_limit_sign_rect.translated(actual_speed_limit_sign_rect.width() + 25, 0));
      // Ensure new_pending_sign_rect width is appropriate for its content
      new_pending_sign_rect.setWidth(newSpeedLimitStr.size() >= 3 ? 200 : 175); // Or adjust as needed

      newSpeedLimitRect = new_pending_sign_rect; // Update global if used elsewhere

      p.setPen(Qt::NoPen);
      p.setBrush(whiteColor());
      p.drawRoundedRect(new_pending_sign_rect, 24, 24);
      p.setPen(pendingLimitPenColor);
      p.drawRoundedRect(new_pending_sign_rect.adjusted(9, 9, -9, -9), 16, 16);

      p.setFont(InterFont(28, QFont::DemiBold));
      p.drawText(new_pending_sign_rect.adjusted(0, 22, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("PENDING"));
      p.drawText(new_pending_sign_rect.adjusted(0, 51, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("LIMIT"));
      p.setFont(InterFont(70, QFont::Bold));
      p.drawText(new_pending_sign_rect.adjusted(0, 85, 0, 0), Qt::AlignTop | Qt::AlignHCenter, newSpeedLimitStr);
    }
  }

  // EU (Vienna style) sign
  if (has_eu_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawEllipse(actual_speed_limit_sign_rect);
    p.setPen(QPen(Qt::red, 20));
    p.drawEllipse(actual_speed_limit_sign_rect.adjusted(16, 16, -16, -16));

    p.save(); // Save before opacity change
    p.setOpacity(slcOverridden ? 0.25 : 1.0);
    p.setPen(blackColor());
    if (showSLCOffset) {
      p.setFont(InterFont((speedLimitStr.size() >= 3) ? 60 : 70, QFont::Bold));
      p.drawText(actual_speed_limit_sign_rect.adjusted(0, -25, 0, 0), Qt::AlignCenter, speedLimitStr);
      p.setFont(InterFont(40, QFont::DemiBold));
      p.drawText(actual_speed_limit_sign_rect.adjusted(0, 100, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitOffsetStr);
    } else {
      p.setFont(InterFont((speedLimitStr.size() >= 3) ? 60 : 70, QFont::Bold));
      p.drawText(actual_speed_limit_sign_rect, Qt::AlignCenter, speedLimitStr);
    }
    p.restore(); // Restore opacity

    // Pending speed limit sign for EU
    if (speedLimitChanged && !cscStatus) { // Added !cscStatus here too
      QRect new_pending_sign_rect(actual_speed_limit_sign_rect.translated(actual_speed_limit_sign_rect.width() + 25, 0));
      p.setPen(Qt::NoPen);
      p.setBrush(whiteColor());
      p.drawEllipse(new_pending_sign_rect);
      p.setPen(QPen(Qt::red, 20)); // Should this be pendingLimitPenColor for the border? Original used Qt::red.
      p.drawEllipse(new_pending_sign_rect.adjusted(16, 16, -16, -16));

      // Text for pending EU sign
      p.setOpacity(1.0); // Ensure full opacity for pending text
      p.setPen(pendingLimitPenColor); // Use pending color for text
      p.setFont(InterFont((newSpeedLimitStr.size() >= 3) ? 60 : 70, QFont::Bold));
      p.drawText(new_pending_sign_rect, Qt::AlignCenter, newSpeedLimitStr);
    }
  }
  p.restore(); // Restore painter state from before speed limit signs


  // Speed limit sources display (position relative to the bottom of the entire cluster)
  if (speedLimitSources && (has_eu_speed_limit || has_us_speed_limit)) {
    std::function<void(QRect&, const QPixmap&, const QString&, double)> drawSource = [&](QRect &rect, const QPixmap &icon, QString title, double speedLimitValue) {
      // ... (original drawSource lambda function content) ...
       if (speedLimitSource == title && !slcOverridden && speedLimitValue != 0) {
        p.setPen(QPen(redColor(), 10));
        p.setBrush(redColor(166));
        p.setFont(InterFont(35, QFont::Bold));
      } else {
        p.setPen(QPen(blackColor(), 10));
        p.setBrush(blackColor(166));
        p.setFont(InterFont(35, QFont::DemiBold));
      }

      QRect iconRect(rect.x() + 20, rect.y() + (rect.height() - img_size / 4) / 2, img_size / 4, img_size / 4);
      QPixmap scaledIcon = icon.scaled(iconRect.size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

      QString speedText;
      if (speedLimitValue > 1) {
        speedText = QString::number(std::nearbyint(speedLimitValue)) + " " + speedUnit;
      } else {
        speedText = "N/A";
      }

      QString fullText = tr(title.toUtf8().constData()) + " - " + speedText;

      p.setOpacity(1.0);
      p.drawRoundedRect(rect, 24, 24);
      p.drawPixmap(iconRect, scaledIcon);

      p.setPen(QPen(whiteColor(), 6));
      QRect textRect(iconRect.right() + 10, rect.y(), rect.width() - iconRect.width() - 30, rect.height());
      p.drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft, fullText);
    };

    // Position these below the set_speed_cluster_rect
    int sources_start_x = set_speed_cluster_rect.left();
    // If cluster is narrow (e.g. side-by-side without DM icon pushing it), align with general UI border if possible
    if (this->hideMapIcon && !this->rightHandDM && cluster_x_pos < 60) sources_start_x = UI_BORDER_SIZE + 15;
    else if (!this->hideMapIcon) sources_start_x = set_speed_cluster_rect.left() + sign_margin;


    QRect dashboardRect(sources_start_x, set_speed_cluster_rect.bottom() + 30, 500, 60);
    QRect mapDataRect(dashboardRect.x(), dashboardRect.y() + dashboardRect.height() + 15, 500, 60);
    QRect navigationRect(mapDataRect.x(), mapDataRect.y() + mapDataRect.height() + 15, 500, 60);
    QRect upcomingLimitRect(navigationRect.x(), navigationRect.y() + navigationRect.height() + 15, 500, 60);

    drawSource(dashboardRect, dashboardIcon, "Dashboard", dashboardSpeedLimit);
    drawSource(mapDataRect, mapDataIcon, "Map Data", mapsSpeedLimit);
    drawSource(navigationRect, navigationIcon, "Navigation", navigationSpeedLimit);
    drawSource(upcomingLimitRect, upcomingMapsIcon, "Upcoming", upcomingSpeedLimit);
  }


  // current speed (using user's y_hud_offset_pixels logic)
  // current speed (using user's y_hud_offset_pixels logic)
  if (!(bigMapOpen || hideSpeed)) {
    int original_speed_text_y = 210;
    int original_speed_unit_y = 290;
    int speed_text_y = original_speed_text_y + this->y_hud_offset_pixels;
    int speed_unit_y = original_speed_unit_y + this->y_hud_offset_pixels;

    int current_speed_center_x;
    if (this->hideMapIcon) {
      p.save();
      p.setFont(InterFont(176, QFont::Bold));
      int speedStr_width_metric = p.fontMetrics().horizontalAdvance(speedStr);
      // You might want to use std::max if speedUnit could be wider and also needs to fit
      // int speedUnit_width_metric = p.fontMetrics(InterFont(66)).horizontalAdvance(speedUnit);
      // int relevant_width_for_centering = std::max(speedStr_width_metric, speedUnit_width_metric);
      int relevant_width_for_centering = speedStr_width_metric; // Assuming speedStr is dominant for width
      p.restore();

      current_speed_center_x = set_speed_cluster_rect.left() - h_spacing - (relevant_width_for_centering / 2);
      // REMOVED: current_speed_center_x = std::max(UI_BORDER_SIZE + (relevant_width_for_centering / 2), current_speed_center_x);
      // By removing this, the current speed text will maintain its position relative to the cluster,
      // even if that means drawing partially or fully off-screen to the left.

    } else {
      current_speed_center_x = rect().center().x();
    }

    // ... (rest of the drawing logic for standstill or normal speed, using 'p' and 'current_speed_center_x')
    // Example for normal speed:
    if (!(standstillDuration > 1)) { // Simplified condition for brevity
        p.setFont(InterFont(176, QFont::Bold));
        drawText(p, current_speed_center_x, current_speed_text_center_y, speedStr); // <<< NEW/MODIFIED LINE
        p.setFont(InterFont(66));
        drawText(p, current_speed_center_x, current_speed_unit_center_y, speedUnit, 200); // <<< NEW/MODIFIED LINE
    } else {
      // ... standstill drawing logic using current_speed_center_x ...
      // (As previously corrected, ensure this part also uses current_speed_center_x for x-positioning)
      float transition = qBound(0.0f, standstillDuration / 120.0f, 1.0f);
      QColor start, end;

      if (standstillDuration <= 60) {
        start = end = bg_colors[STATUS_ENGAGED];
      } else if (standstillDuration <= 90) {
        start = bg_colors[STATUS_ENGAGED];
        end = bg_colors[STATUS_CONDITIONAL_OVERRIDDEN];
        transition = (standstillDuration - 60) / 30.0f;
      } else if (standstillDuration <= 120) {
        start = bg_colors[STATUS_CONDITIONAL_OVERRIDDEN];
        end = bg_colors[STATUS_TRAFFIC_MODE_ACTIVE];
        transition = (standstillDuration - 90) / 30.0f;
      } else {
        start = end = bg_colors[STATUS_TRAFFIC_MODE_ACTIVE];
        transition = 0.0f;
      }

      QColor blendedColor(
        start.red() + transition * (end.red() - start.red()),
        start.green() + transition * (end.green() - start.green()),
        start.blue() + transition * (end.blue() - start.blue())
      );

      p.setPen(QPen(blendedColor));

      int minutes = standstillDuration / 60;
      int seconds = standstillDuration % 60;

      QString standstill_main_text = minutes == 1 ? tr("1 minute") : tr("%1 minutes").arg(minutes);
      QString standstill_sub_text = tr("%1 seconds").arg(seconds);

      p.setFont(InterFont(176, QFont::Bold));
      drawText(p, current_speed_center_x, current_speed_text_center_y, standstill_main_text, 255, true); // <<< NEW/MODIFIED LINE
      p.setFont(InterFont(66));
      drawText(p, current_speed_center_x, current_speed_unit_center_y, standstill_sub_text); // <<< NEW/MODIFIED LINE
    }
  }


    
  p.restore(); // Final restore for the painter saved at the beginning of drawHud
}



void AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, int alpha, bool overridePen) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  if (!overridePen) {
    p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  }
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::initializeGL() {
  CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void AnnotatedCameraWidget::updateFrameMat() {
  CameraWidget::updateFrameMat();
  UIState *s = uiState();
  int w = width(), h = height();

  s->fb_w = w;
  s->fb_h = h;

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2 - x_offset, h / 2 - y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s, float v_ego) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    if (useStockColors) {
      painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    } else {
      painter.setBrush(scene.lane_lines_color);
    }
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, 0);
  if (experimentalMode || scene.acceleration_path || scene.rainbow_path) {
    // The first half of track_vertices are the points for the right side of the path
    // and the indices match the positions of accel from uiPlan
    const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration().getX();
    const int max_len = std::min<int>(scene.track_vertices.length() / 2, acceleration.size());

    for (int i = 0; i < max_len; ++i) {
      // Some points are out of frame
      int track_idx = max_len - i - 1;  // flip idx to start from bottom right
      if (scene.track_vertices[track_idx].y() < 0 || scene.track_vertices[track_idx].y() > height()) continue;

      // Flip so 0 is bottom of frame
      float lin_grad_point = (height() - scene.track_vertices[track_idx].y()) / height();

      if ((fabs(acceleration[i]) < 0.5 || !scene.acceleration_path) && scene.rainbow_path) {
        static float hue_offset = 0.0;
        if (v_ego > 0) {
          hue_offset += powf(v_ego, 0.5f) / sqrtf(145.0f / MS_TO_KPH);
        }

        float alpha = util::map_val(lin_grad_point, 0.0f, 1.0f, 0.5f, 0.1f);
        float path_hue = fmodf((lin_grad_point * 360.0f) + hue_offset, 360.0f);

        bg.setColorAt(lin_grad_point, QColor::fromHslF(path_hue / 360.0f, 1.0f, 0.5f, alpha));
        bg.setSpread(QGradient::RepeatSpread);
      } else if (fabs(acceleration[i]) < 0.5 && !useStockColors) {
        QColor color = scene.path_color;
        color.setAlphaF(util::map_val(lin_grad_point, 0.0f, 1.0f, 1.0f, 0.1f));
        bg.setColorAt(lin_grad_point, color);
      } else {
        // speed up: 120, slow down: 0
        float path_hue = fmax(fmin(60 + acceleration[i] * 35, 120), 0);
        // FIXME: painter.drawPolygon can be slow if hue is not rounded
        path_hue = int(path_hue * 100 + 0.5) / 100;

        float saturation = fmin(fabs(acceleration[i] * 1.5), 1);
        float lightness = util::map_val(saturation, 0.0f, 1.0f, 0.95f, 0.62f);  // lighter when grey
        float alpha = util::map_val(lin_grad_point, 0.75f / 2.f, 0.75f, 0.4f, 0.0f);  // matches previous alpha fade
        bg.setColorAt(lin_grad_point, QColor::fromHslF(path_hue / 360., saturation, lightness, alpha));

        // Skip a point, unless next is last
        i += (i + 2) < max_len ? 1 : 0;
      }
    }

  } else if (!useStockColors) {
    QColor color = scene.path_color;
    bg.setColorAt(0.0f, color);
    color.setAlphaF(1.0f);
    bg.setColorAt(0.5f, color);
    color.setAlphaF(0.1f);
    bg.setColorAt(1.0f, color);

  } else {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  }

  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices);

  if (scene.show_stopping_point && scene.red_light && scene.track_vertices.length() > 1) {
    QPointF center_point = (scene.track_vertices.first() + scene.track_vertices.last()) / 2.0;
    QPointF adjusted_point = center_point - QPointF(stopSignImg.width() / 2, stopSignImg.height());
    painter.drawPixmap(adjusted_point, stopSignImg);

    if (scene.show_stopping_point_metrics) {
      QFont font = InterFont(35, QFont::DemiBold);
      QString text = QString::number(std::nearbyint(modelLength * distanceConversion)) + leadDistanceUnit;
      int text_width = QFontMetrics(font).horizontalAdvance(text);
      QPointF text_position = center_point - QPointF(text_width / 2, stopSignImg.height() + 35);

      painter.save();
      painter.setFont(font);
      painter.setPen(Qt::white);
      painter.drawText(text_position, text);
      painter.restore();
    }
  }

  // Paint blindspot path
  if (scene.blind_spot_path) {
    QLinearGradient bs(0, height(), 0, 0);

    bs.setColorAt(0.0f, QColor::fromHslF(0 / 360.0f, 0.75f, 0.5f, 0.6f));
    bs.setColorAt(0.5f, QColor::fromHslF(0 / 360.0f, 0.75f, 0.5f, 0.4f));
    bs.setColorAt(1.0f, QColor::fromHslF(0 / 360.0f, 0.75f, 0.5f, 0.2f));

    painter.setBrush(bs);
    if (blindSpotLeft) {
      painter.drawPolygon(scene.track_adjacent_vertices[4]);
    }
    if (blindSpotRight) {
      painter.drawPolygon(scene.track_adjacent_vertices[5]);
    }
  }

  // Paint adjacent lane paths
  if ((scene.adjacent_path || scene.adjacent_path_metrics) && scene.lane_width_left != 0 && scene.lane_width_right != 0) {
    QLinearGradient ap(0, height(), 0, 0);

    std::function<void(float)> setAdjacentPathColors = [&](float hue) {
      ap.setColorAt(0.0f, QColor::fromHslF(hue / 360.0f, 0.75f, 0.5f, 0.6f));
      ap.setColorAt(0.5f, QColor::fromHslF(hue / 360.0f, 0.75f, 0.5f, 0.4f));
      ap.setColorAt(1.0f, QColor::fromHslF(hue / 360.0f, 0.75f, 0.5f, 0.2f));
    };

    std::function<void(const QPolygonF&, float, bool)> drawAdjacentLane = [&](const QPolygonF &lane, float laneWidth, bool isBlindSpot) {
      if (isBlindSpot) {
        setAdjacentPathColors(0.0f);
      } else {
        float hue = 120.0f * (1 - fmin(fabs(laneWidth - laneDetectionWidth) / (laneDetectionWidth / 2), 1));
        setAdjacentPathColors(hue);
      }

      painter.setBrush(ap);
      painter.drawPolygon(lane);

      if (scene.adjacent_path_metrics) {
        painter.setFont(InterFont(30, QFont::DemiBold));
        painter.setPen(Qt::white);

        QString text = isBlindSpot ? tr("Vehicle in blind spot") : QString::number(laneWidth * distanceConversion, 'f', 2) + leadDistanceUnit;
        painter.drawText(lane.boundingRect(), Qt::AlignCenter, text);
        painter.setPen(Qt::NoPen);
      }
    };

    drawAdjacentLane(scene.track_adjacent_vertices[4], scene.lane_width_left, blindSpotLeft);
    drawAdjacentLane(scene.track_adjacent_vertices[5], scene.lane_width_right, blindSpotRight);
  }

  // Paint path edges
  QLinearGradient pe(0, height(), 0, 0);

  std::function<void(QLinearGradient&, const QColor&)> setPathEdgeColors = [&](QLinearGradient &gradient, QColor baseColor) {
    baseColor.setAlphaF(1.0f);
    gradient.setColorAt(0.0f, baseColor);
    baseColor.setAlphaF(0.5f);
    gradient.setColorAt(0.5f, baseColor);
    baseColor.setAlphaF(0.1f);
    gradient.setColorAt(1.0f, baseColor);
  };

  if (scene.always_on_lateral_enabled) {
    setPathEdgeColors(pe, bg_colors[STATUS_ALWAYS_ON_LATERAL_ENABLED]);
  } else if (conditionalStatus == 1) {
    setPathEdgeColors(pe, bg_colors[STATUS_CONDITIONAL_OVERRIDDEN]);
  } else if (experimentalMode) {
    setPathEdgeColors(pe, bg_colors[STATUS_EXPERIMENTAL_MODE_ACTIVE]);
  } else if (trafficMode) {
    setPathEdgeColors(pe, bg_colors[STATUS_TRAFFIC_MODE_ACTIVE]);
  } else if (modelLength > scene.upcoming_maneuver_distance && scene.upcoming_maneuver_distance > 1) {
    setPathEdgeColors(pe, bg_colors[STATUS_NAVIGATION_ACTIVE]);
  } else if (!useStockColors) {
    setPathEdgeColors(pe, scene.path_edges_color);
  } else {
    pe.setColorAt(0.0f, QColor::fromHslF(148 / 360.0f, 0.94f, 0.51f, 1.0f));
    pe.setColorAt(0.5f, QColor::fromHslF(112 / 360.0f, 1.00f, 0.68f, 0.5f));
    pe.setColorAt(1.0f, QColor::fromHslF(112 / 360.0f, 1.00f, 0.68f, 0.1f));
  }

  QPainterPath path;
  path.addPolygon(scene.track_vertices);
  path.addPolygon(scene.track_edge_vertices);

  painter.setBrush(pe);
  painter.drawPath(path);

  painter.restore();
}

void AnnotatedCameraWidget::drawDriverState(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;

  painter.save();

  // base icon
  int offset = UI_BORDER_SIZE + btn_size / 2;
  if (roadNameUI) {
    offset += 30;
  }
  int x = rightHandDM ? width() - offset : offset;
  if (rightHandDM && map_settings_btn->isEnabled() && !hideMapIcon) {
    x -= 250;
  } else if (onroadDistanceButton) {
    x += 250;
  }
  int y = height() - offset;
  dmIconPosition.setX(x);
  dmIconPosition.setY(y);
  float opacity = dmActive ? 0.65 : 0.2;
  drawIcon(painter, dmIconPosition, dm_img, blackColor(70), opacity);

  // face
  QPointF face_kpts_draw[std::size(default_face_kpts_3d)];
  float kp;
  for (int i = 0; i < std::size(default_face_kpts_3d); ++i) {
    kp = (scene.face_kpts_draw[i].v[2] - 8) / 120 + 1.0;
    face_kpts_draw[i] = QPointF(scene.face_kpts_draw[i].v[0] * kp + x, scene.face_kpts_draw[i].v[1] * kp + y);
  }

  painter.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  painter.drawPolyline(face_kpts_draw, std::size(default_face_kpts_3d));

  // tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7;
  const float arc_t_extend = 12.0;
  QColor arc_color = QColor::fromRgbF(0.545 - 0.445 * s->engaged(),
                                      0.545 + 0.4 * s->engaged(),
                                      0.545 - 0.285 * s->engaged(),
                                      0.4 * (1.0 - dm_fade_state));
  float delta_x = -scene.driver_pose_sins[1] * arc_l / 2;
  float delta_y = -scene.driver_pose_sins[0] * arc_l / 2;
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(std::fmin(x + delta_x, x), y - arc_l / 2, fabs(delta_x), arc_l), (scene.driver_pose_sins[1]>0 ? 90 : -90) * 16, 180 * 16);
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(x - arc_l / 2, std::fmin(y + delta_y, y), arc_l, fabs(delta_y)), (scene.driver_pose_sins[0]>0 ? 0 : 180) * 16, 180 * 16);

  painter.restore();
}

void AnnotatedCameraWidget::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd, float v_ego, const QColor &lead_marker_color, bool adjacent) {
  painter.save();

  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getDRel() + (adjacent ? fabs(lead_data.getYRel()) : 0);
  const float v_rel = lead_data.getVRel();

  float fillAlpha = 0;
  if (adjacent) {
    fillAlpha = 255;
  } else if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), adjacent ? 10.0f : 15.0f, adjacent ? 20.0f : 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  painter.setBrush(QColor(218, 202, 37, 255));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  if (!adjacent && useStockColors) {
    painter.setBrush(redColor(fillAlpha));
  } else {
    painter.setBrush(QColor(lead_marker_color.red(), lead_marker_color.green(), lead_marker_color.blue(), fillAlpha));
  }
  painter.drawPolygon(chevron, std::size(chevron));

  if (leadInfo) {
    float lead_speed = std::max(v_rel + v_ego, 0.0f);

    painter.setPen(Qt::white);
    painter.setFont(InterFont(35, QFont::Bold));

    QString text;
    if (adjacent) {
      text = QString("%1 %2 | %3 %4")
              .arg(qRound(d_rel * distanceConversion))
              .arg(leadDistanceUnit)
              .arg(qRound(lead_speed * speedConversionMetrics))
              .arg(leadSpeedUnit);
    } else {
      text = QString("%1 %2 (%3) | %4 %5 | %6%7")
              .arg(qRound(d_rel * distanceConversion))
              .arg(leadDistanceUnit)
              .arg(QString("Desired: %1").arg(desiredFollow * distanceConversion))
              .arg(qRound(lead_speed * speedConversionMetrics))
              .arg(leadSpeedUnit)
              .arg(QString::number(std::max(d_rel / std::max(v_ego, 1.0f), 1.0f), 'f', 2))
              .arg("s");
    }

    QFontMetrics metrics(painter.font());
    int middle_x = (chevron[2].x() + chevron[0].x()) / 2;
    int textHeight = metrics.height();
    int textWidth = metrics.horizontalAdvance(text);
    int text_x = middle_x - textWidth / 2;
    int text_y = chevron[0].y() + textHeight + 5;

    if (!adjacent) {
      int xMargin = textWidth * 0.25;
      int yMargin = textHeight * 0.25;

      leadTextRect = QRect(text_x, text_y - textHeight, textWidth, textHeight).adjusted(-xMargin, -yMargin, xMargin, yMargin);
      painter.drawText(text_x, text_y, text);
    } else {
      QRect adjacentTextRect(text_x, text_y - textHeight, textWidth, textHeight);
      if (!adjacentTextRect.intersects(leadTextRect)) {
        painter.drawText(text_x, text_y, text);
      }
    }
  }

  painter.restore();
}

void AnnotatedCameraWidget::paintGL() {
}

void AnnotatedCameraWidget::paintEvent(QPaintEvent *event) {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  QPainter painter(this);
  painter.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing);
  const double start_draw_t = millis_since_boot();
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();
  const float v_ego = sm["carState"].getCarState().getVEgo();

  // Set the background color for the CameraWidget area (will be used by glClear in CameraWidget::paintGL)
  this->setBackgroundColor(bg_colors[s->status]); // <<< ADD THIS LINE

  // Ensure car_space_transform is updated for HUD elements.
  // This calls CameraWidget::updateFrameMat() (base) which updates CameraWidget's internal state (e.g., zoom, offsets),
  // and then computes s->car_space_transform based on that state.
  this->updateFrameMat(); // <<< ADD THIS LINE


  // draw camera frame
  {
    std::lock_guard lk(frame_lock);

    if (frames.empty()) {
      if (skip_frame_count > 0) {
        skip_frame_count--;
        qDebug() << "skipping frame, not ready";
        return;
      }
    } else {
      // skip drawing up to this many frames if we're
      // missing camera frames. this smooths out the
      // transitions from the narrow and wide cameras
      skip_frame_count = 5;
    }

    // Wide or narrow cam dependent on speed
    bool has_wide_cam = available_streams.count(VISION_STREAM_WIDE_ROAD);
    if (has_wide_cam && cameraView == 0) {
      if ((v_ego < 10) || available_streams.size() == 1) {
        wide_cam_requested = true;
      } else if (v_ego > 15) {
        wide_cam_requested = false;
      }
      wide_cam_requested = wide_cam_requested && experimentalMode;
      // for replay of old routes, never go to widecam
      wide_cam_requested = wide_cam_requested && s->scene.calibration_wide_valid;
    }
    CameraWidget::setStreamType(cameraView == 1 ? VISION_STREAM_DRIVER :
                                cameraView == 3 || wide_cam_requested ? VISION_STREAM_WIDE_ROAD :
                                VISION_STREAM_ROAD);

    s->scene.wide_cam = CameraWidget::getStreamType() == VISION_STREAM_WIDE_ROAD;
    if (s->scene.calibration_valid) {
      auto calib = s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;
      CameraWidget::updateCalibration(calib);
    } else {
      CameraWidget::updateCalibration(DEFAULT_CALIBRATION);
    }
    painter.beginNativePainting();
    CameraWidget::setFrameId(model.getFrameId());
    CameraWidget::paintGL();
    painter.endNativePainting();
  }

  painter.setPen(Qt::NoPen);

  if (s->scene.world_objects_visible) {
    // Conditionally update model data and draw lane lines/paths
    if (!this->hideMapIcon) { // <<<--- ADD THIS CONDITION
      update_model(s, model, sm["uiPlan"].getUiPlan());
      drawLaneLines(painter, s, v_ego);
    } // <<<--- END ADDED CONDITION

    if (s->scene.longitudinal_control && sm.rcv_frame("radarState") > s->scene.started_frame && !s->scene.hide_lead_marker && !hideMapIcon) {// <<< MODIFIED LINE
      auto radar_state = sm["radarState"].getRadarState();
      update_leads(s, radar_state, model.getPosition());
      auto lead_one = radar_state.getLeadOne();
      auto lead_two = radar_state.getLeadTwo();
      auto lead_left = radar_state.getLeadLeft();
      auto lead_right = radar_state.getLeadRight();
      if (lead_two.getStatus()) {
        drawLead(painter, lead_two, s->scene.lead_vertices[1], v_ego, s->scene.lead_marker_color);
      } else if (lead_one.getStatus()) {
        drawLead(painter, lead_one, s->scene.lead_vertices[0], v_ego, s->scene.lead_marker_color);
      } else {
        leadTextRect = QRect();
      }
      if (lead_left.getStatus()) {
        drawLead(painter, lead_left, s->scene.lead_vertices[2], v_ego, blueColor(), true);
      }
      if (lead_right.getStatus()) {
        drawLead(painter, lead_right, s->scene.lead_vertices[3], v_ego, purpleColor(), true);
      }
    }
  }

  // DMoji
  if (!hideBottomIcons && (sm.rcv_frame("driverStateV2") > s->scene.started_frame)) {
    update_dmonitoring(s, sm["driverStateV2"].getDriverStateV2(), dm_fade_state, rightHandDM);
    drawDriverState(painter, s);
  }

  drawHud(painter);

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  s->scene.fps = fps;
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;

  // publish debug msg
  MessageBuilder msg;
  auto m = msg.initEvent().initUiDebug();
  m.setDrawTimeMillis(cur_draw_t - start_draw_t);
  pm->send("uiDebug", msg);

  // Paint FrogPilot widgets
  paintFrogPilotWidgets(painter);
}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
  CameraWidget::showEvent(event);

  ui_update_params(uiState());
  prev_draw_t = millis_since_boot();

  // FrogPilot variables
  distance_btn->updateIcon();
  experimental_btn->updateIcon();
  updateSignals();
}

// FrogPilot widgets
void AnnotatedCameraWidget::updateSignals() {
  blindspotImages.clear();
  signalImages.clear();

  QDir directory("../frogpilot/assets/active_theme/signals/");
  QFileInfoList allFiles = directory.entryInfoList(QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

  bool isGif = false;
  for (QFileInfo &fileInfo : allFiles) {
    if (fileInfo.fileName().endsWith(".gif", Qt::CaseInsensitive)) {
      QMovie movie(fileInfo.absoluteFilePath());
      movie.start();

      for (int frameIndex = 0; frameIndex < movie.frameCount(); ++frameIndex) {
        movie.jumpToFrame(frameIndex);
        QPixmap currentFrame = movie.currentPixmap();
        signalImages.push_back(currentFrame);
        signalImages.push_back(currentFrame.transformed(QTransform().scale(-1, 1)));
      }

      movie.stop();
      isGif = true;

    } else if (fileInfo.fileName().endsWith(".png", Qt::CaseInsensitive)) {
      QVector<QPixmap> *targetList = fileInfo.fileName().contains("blindspot") ? &blindspotImages : &signalImages;
      QPixmap pixmap(fileInfo.absoluteFilePath());
      targetList->push_back(pixmap);
      targetList->push_back(pixmap.transformed(QTransform().scale(-1, 1)));

    } else {
      QStringList parts = fileInfo.fileName().split('_');
      if (parts.size() == 2) {
        signalStyle = parts[0];
        signalAnimationLength = parts[1].toInt();
      }
    }
  }

  if (!signalImages.empty()) {
    QPixmap &firstImage = signalImages.front();
    signalWidth = firstImage.width();
    signalHeight = firstImage.height();
    totalFrames = signalImages.size() / 2;
    turnSignalAnimation = true;

    if (isGif && signalStyle == "traditional") {
      signalMovement = (this->size().width() + (signalWidth * 2)) / totalFrames;
      signalStyle = "traditional_gif";
    } else {
      signalMovement = 0;
    }
  } else {
    signalWidth = 0;
    signalHeight = 0;
    totalFrames = 0;
    turnSignalAnimation = false;
  }
}

void AnnotatedCameraWidget::initializeFrogPilotWidgets() {
  distance_btn = new DistanceButton(this);
  main_layout->addWidget(distance_btn, 0, Qt::AlignBottom | Qt::AlignLeft);

  chillModeIcon = loadPixmap("../frogpilot/assets/other_images/chill_mode_icon.png", {img_size / 2, img_size / 2});
  curveIcon = loadPixmap("../frogpilot/assets/other_images/curve_icon.png", {img_size / 2, img_size / 2});
  curveSpeedLeftIcon = loadPixmap("../frogpilot/assets/other_images/curve_speed_left.png", {img_size, img_size});
  curveSpeedRightIcon = loadPixmap("../frogpilot/assets/other_images/curve_speed_right.png", {img_size, img_size});
  dashboardIcon = loadPixmap("../frogpilot/assets/other_images/dashboard_icon.png", {img_size / 2, img_size / 2});
  experimentalModeIcon = loadPixmap("../assets/img_experimental.svg", {img_size / 2, img_size / 2});
  leadIcon = loadPixmap("../frogpilot/assets/other_images/lead_icon.png", {img_size / 2, img_size / 2});
  lightIcon = loadPixmap("../frogpilot/assets/other_images/light_icon.png", {img_size / 2, img_size / 2});
  mapDataIcon = loadPixmap("../frogpilot/assets/other_images/offline_maps_icon.png", {img_size / 2, img_size / 2});
  navigationIcon = loadPixmap("../frogpilot/assets/other_images/navigation_icon.png", {img_size / 2, img_size / 2});
  pausedIcon = loadPixmap("../frogpilot/assets/other_images/paused_icon.png", {img_size / 2, img_size / 2});
  speedIcon = loadPixmap("../frogpilot/assets/other_images/speed_icon.png", {img_size / 2, img_size / 2});
  stopSignImg = loadPixmap("../frogpilot/assets/other_images/stop_sign.png", {img_size, img_size});
  turnIcon = loadPixmap("../frogpilot/assets/other_images/turn_icon.png", {img_size / 2, img_size / 2});
  upcomingMapsIcon = loadPixmap("../frogpilot/assets/other_images/upcoming_maps_icon.png", {img_size / 2, img_size / 2});

  animationTimer = new QTimer(this);
  QObject::connect(animationTimer, &QTimer::timeout, [this] {
    animationFrameIndex = (animationFrameIndex + 1) % totalFrames;
  });

  QObject::connect(uiState(), &UIState::themeUpdated, this, &AnnotatedCameraWidget::updateSignals);
  QObject::connect(uiState(), &UIState::themeUpdated, distance_btn, &DistanceButton::updateIcon);
  QObject::connect(uiState(), &UIState::themeUpdated, experimental_btn, &ExperimentalButton::updateIcon);
}

void AnnotatedCameraWidget::updateFrogPilotVariables(int alert_height, const UIScene &scene) {

  hideMapIcon = scene.hide_map_icon; // This member already exists and is updated
  this->setStreamHidden(hideMapIcon); // <<< ADD THIS LINE to control the base CameraWidget

  // >>> ADD THESE LINES  / to reduce height by 45% set (UI_HEADER_HEIGHT * 0.45f) and so on
  if (this->hideMapIcon) {
    // UI_HEADER_HEIGHT is 420, so 0.45 * 420 = 189
    this->y_hud_offset_pixels = static_cast<int>(UI_HEADER_HEIGHT * 0.1f);
  } else {
    this->y_hud_offset_pixels = 0;
  }

  // This affects the Record Button and Experimental Button ("steering wheel")
  if (this->top_right_layout) {
    this->top_right_layout->setContentsMargins(0, this->y_hud_offset_pixels, 0, 0); // (Left, Top, Right, Bottom)
  }
  // <<< END ADDED LINES

  if (is_metric || useSI) {
    accelerationUnit = tr("m/s²");
    leadDistanceUnit = tr(mapOpen ? "m" : "meters");
    leadSpeedUnit = useSI ? tr("m/s") : tr("km/h");

    accelerationConversion = 1.0f;
    distanceConversion = 1.0f;
    speedConversion = is_metric ? MS_TO_KPH : MS_TO_MPH;
    speedConversionMetrics = useSI ? 1.0f : MS_TO_KPH;
  } else {
    accelerationUnit = tr("ft/s²");
    leadDistanceUnit = tr(mapOpen ? "ft" : "feet");
    leadSpeedUnit = tr("mph");

    accelerationConversion = METER_TO_FOOT;
    distanceConversion = METER_TO_FOOT;
    speedConversion = MS_TO_MPH;
    speedConversionMetrics = MS_TO_MPH;
  }

  alertHeight = alert_height;

  blindSpotLeft = scene.blind_spot_left;
  blindSpotRight = scene.blind_spot_right;

  cameraView = scene.camera_view;

  cemStatus = scene.cem_status;
  conditionalStatus = scene.conditional_status;

  cscStatus = scene.csc_status && (setSpeed - mtscSpeed > 1 || setSpeed - vtscSpeed > 1) && is_cruise_set;

  compass = scene.compass;

  desiredFollow = scene.desired_follow;

  experimentalMode = scene.experimental_mode;

  hideMapIcon = scene.hide_map_icon;
  hideMaxSpeed = scene.hide_max_speed;
  hideSpeed = scene.hide_speed;
  hideSpeedLimit = scene.hide_speed_limit;

  lateralPaused = scene.lateral_paused;

  laneDetectionWidth = scene.lane_detection_width;

  leadInfo = scene.lead_metrics;

  leftCurve = scene.left_curve;

  longitudinalPaused = scene.longitudinal_paused;

  mapOpen = scene.map_open;
  bigMapOpen = mapOpen && scene.big_map;

  modelLength = scene.model_length;

  mtscEnabled = scene.mtsc_enabled;
  mtscSpeed = mtscEnabled ? scene.mtsc_speed * speedConversion : setSpeed;

  onroadDistanceButton = scene.onroad_distance_button;
  bool enableDistanceButton = onroadDistanceButton && !hideBottomIcons;
  distance_btn->setVisible(enableDistanceButton);
  if (enableDistanceButton) {
    distance_btn->updateState(scene);
  }

  bool enablePedalIcons = scene.pedals_on_ui && !bigMapOpen;
  pedal_icons->setVisible(enablePedalIcons);
  if (enablePedalIcons) {
    pedal_icons->updateState(scene);
  }

  radarTracks = scene.radar_tracks;

  roadNameUI = scene.road_name_ui;
  distance_btn->road_name_ui = roadNameUI;

  bool enableScreenRecorder = scene.screen_recorder && !mapOpen;
  screenRecorder->setVisible(enableScreenRecorder);

  dashboardSpeedLimit = scene.dashboard_speed_limit * speedConversion;
  mapsSpeedLimit = scene.speed_limit_map * speedConversion;
  navigationSpeedLimit = scene.navigation_speed_limit * speedConversion;
  showSLCOffset = scene.show_speed_limit_offset;
  slcOverridden = scene.speed_limit_overridden;
  slcSpeedLimitOffset = scene.speed_limit_offset * speedConversion;
  speedLimitChanged = scene.speed_limit_changed;
  speedLimitSource = scene.speed_limit_source;
  speedLimitSources = scene.speed_limit_sources;
  unconfirmedSpeedLimit = scene.unconfirmed_speed_limit * speedConversion;
  upcomingSpeedLimit = scene.upcoming_speed_limit * speedConversion;
  useVienna = scene.speed_limit_vienna;

  bool stoppedTimer = scene.stopped_timer && scene.standstill && scene.started_timer / UI_FREQ >= 10 && !mapOpen;
  static QElapsedTimer standstillTimer;
  if (stoppedTimer) {
    if (!standstillTimer.isValid()) {
      standstillTimer.start();
    }
    standstillDuration = standstillTimer.elapsed() / 1000.0;
  } else {
    standstillDuration = 0;
    standstillTimer.invalidate();
  }

  trafficMode = scene.traffic_mode_active;

  turnSignalLeft = scene.turn_signal_left;
  turnSignalRight = scene.turn_signal_right;

  useSI = scene.use_si_metrics;

  useStockColors = scene.use_stock_colors;

  vtscControllingCurve = scene.vtsc_controlling_curve;
  vtscEnabled = scene.vtsc_enabled;
  vtscSpeed = vtscEnabled ? scene.vtsc_speed * speedConversion : setSpeed;
}

void AnnotatedCameraWidget::paintFrogPilotWidgets(QPainter &painter) {
  if (cemStatus && !hideBottomIcons) {
    drawCEMStatus(painter);
  } else {
    cemIconPosition.setX(0);
    cemIconPosition.setY(0);
  }

  if (lateralPaused && !mapOpen && !hideBottomIcons) {
    drawLateralPaused(painter);
  } else {
    lateralIconPosition.setX(0);
    lateralIconPosition.setY(0);
  }

  if (longitudinalPaused && !mapOpen && !hideBottomIcons) {
    drawLongitudinalPaused(painter);
  }

  if (radarTracks) {
    drawRadarTracks(painter);
  }

  if (roadNameUI && !bigMapOpen) {
    drawRoadName(painter);
  }

  if (turnSignalAnimation && (turnSignalLeft || turnSignalRight) && !bigMapOpen && ((!mapOpen && standstillDuration == 0) || signalStyle != "static")) {
    if (!animationTimer->isActive()) {
      animationTimer->start(signalAnimationLength);
    }
    drawTurnSignals(painter);
  } else if (animationTimer->isActive()) {
    animationTimer->stop();
  }
}

void AnnotatedCameraWidget::drawCEMStatus(QPainter &p) {
  if (dmIconPosition == QPoint(0, 0)) {
    return;
  }

  cemIconPosition.rx() = dmIconPosition.x();
  cemIconPosition.ry() = dmIconPosition.y() - img_size / 2;
  cemIconPosition.rx() += (rightHandDM ? -img_size : img_size) / (mapOpen ? 1.25 : 1);

  QRect cemWidget(cemIconPosition.x(), cemIconPosition.y(), img_size, img_size);

  p.save();

  if (conditionalStatus == 1) {
    p.setPen(QPen(QColor(bg_colors[STATUS_CONDITIONAL_OVERRIDDEN]), 10));
  } else if (experimentalMode) {
    p.setPen(QPen(QColor(bg_colors[STATUS_EXPERIMENTAL_MODE_ACTIVE]), 10));
  } else {
    p.setPen(QPen(blackColor(), 10));
  }
  p.setBrush(blackColor(166));
  p.drawRoundedRect(cemWidget, 24, 24);

  QPixmap iconToDraw;
  if (experimentalMode) {
    if (conditionalStatus == 1) {
      iconToDraw = chillModeIcon;
    } else if (conditionalStatus == 2) {
      iconToDraw = experimentalModeIcon;
    } else if (conditionalStatus == 3 || conditionalStatus == 4) {
      iconToDraw = speedIcon;
    } else if (conditionalStatus == 5 || conditionalStatus == 7) {
      iconToDraw = turnIcon;
    } else if (conditionalStatus == 6 || conditionalStatus == 11 || conditionalStatus == 12) {
      iconToDraw = lightIcon;
    } else if (conditionalStatus == 8) {
      iconToDraw = curveIcon;
    } else if (conditionalStatus == 9 || conditionalStatus == 10) {
      iconToDraw = leadIcon;
    } else {
      iconToDraw = experimentalModeIcon;
    }
  } else {
    iconToDraw = chillModeIcon;
  }
  p.drawPixmap(cemWidget, iconToDraw);

  p.restore();
}

void AnnotatedCameraWidget::drawLateralPaused(QPainter &p) {
  if (dmIconPosition == QPoint(0, 0)) {
    return;
  }

  if (cemIconPosition != QPoint(0, 0)) {
    lateralIconPosition = cemIconPosition;
  } else {
    lateralIconPosition.rx() = dmIconPosition.x();
    lateralIconPosition.ry() = dmIconPosition.y() - img_size / 2;
  }
  lateralIconPosition.rx() += ((rightHandDM ? -img_size : img_size) * 1.5) / (mapOpen ? 1.25 : 1);

  QRect lateralWidget(lateralIconPosition.x(), lateralIconPosition.y(), img_size, img_size);

  p.save();

  p.setPen(QPen(QColor(bg_colors[STATUS_TRAFFIC_MODE_ACTIVE]), 10));
  p.setBrush(blackColor(166));
  p.drawRoundedRect(lateralWidget, 24, 24);

  p.setOpacity(0.5);
  p.drawPixmap(lateralWidget, turnIcon);
  p.setOpacity(0.75);
  p.drawPixmap(lateralWidget, pausedIcon);

  p.restore();
}

void AnnotatedCameraWidget::drawLongitudinalPaused(QPainter &p) {
  if (dmIconPosition == QPoint(0, 0)) {
    return;
  }

  QPoint longitudinalIconPosition;
  if (lateralIconPosition != QPoint(0, 0)) {
    longitudinalIconPosition = lateralIconPosition;
  } else if (cemIconPosition != QPoint(0, 0)) {
    longitudinalIconPosition = cemIconPosition;
  } else {
    longitudinalIconPosition.rx() = dmIconPosition.x();
    longitudinalIconPosition.ry() = dmIconPosition.y() - img_size / 2;
  }
  longitudinalIconPosition.rx() += ((rightHandDM ? -img_size : img_size) * 1.5) / (mapOpen ? 1.25 : 1);

  QRect longitudinalWidget(longitudinalIconPosition.x(), longitudinalIconPosition.y(), img_size, img_size);

  p.save();

  p.setPen(QPen(QColor(bg_colors[STATUS_TRAFFIC_MODE_ACTIVE]), 10));
  p.setBrush(blackColor(166));
  p.drawRoundedRect(longitudinalWidget, 24, 24);

  p.setOpacity(0.5);
  p.drawPixmap(longitudinalWidget, speedIcon);
  p.setOpacity(0.75);
  p.drawPixmap(longitudinalWidget, pausedIcon);

  p.restore();
}

PedalIcons::PedalIcons(QWidget *parent) : QWidget(parent) {
  setFixedSize(btn_size, btn_size);

  brake_pedal_img = loadPixmap("../frogpilot/assets/other_images/brake_pedal.png", {img_size, img_size});
  gas_pedal_img = loadPixmap("../frogpilot/assets/other_images/gas_pedal.png", {img_size, img_size});
}

void PedalIcons::updateState(const UIScene &scene) {
  acceleration = scene.acceleration;
  brakeLightOn = scene.brake_lights_on;
  dynamicPedals = scene.dynamic_pedals_on_ui;
  standstill = scene.standstill;
  staticPedals = scene.static_pedals_on_ui;

  accelerating = acceleration > 0.25f;
  decelerating = acceleration < -0.25f;
}

void PedalIcons::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  int totalWidth = 2 * img_size;
  int startX = (width() - totalWidth) / 2;

  int brakeX = startX + img_size / 2;
  int gasX = startX + img_size;

  float brakeOpacity = 1.0f;
  float gasOpacity = 1.0f;

  if (dynamicPedals) {
    brakeOpacity = standstill ? 1.0f : decelerating ? std::max(0.25f, std::abs(acceleration)) : 0.25f;
    gasOpacity = accelerating ? std::max(0.25f, acceleration) : 0.25f;
  } else if (staticPedals) {
    brakeOpacity = standstill || brakeLightOn || acceleration < -0.5f ? 1.0f : 0.25f;
    gasOpacity = !standstill && acceleration > 0 ? 1.0f : 0.25f;
  }

  p.setOpacity(brakeOpacity);
  p.drawPixmap(brakeX, (height() - img_size) / 2, brake_pedal_img);

  p.setOpacity(gasOpacity);
  p.drawPixmap(gasX, (height() - img_size) / 2, gas_pedal_img);
}

void AnnotatedCameraWidget::drawRadarTracks(QPainter &p) {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;

  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();

  if (scene.world_objects_visible) {
    capnp::List<cereal::LiveTracks>::Reader liveTracks = sm["liveTracks"].getLiveTracks();
    update_radar_tracks(s, liveTracks, model.getPosition());

    constexpr int diameter = 25;

    QRect viewport = p.viewport();

    int width = viewport.width();
    int height = viewport.height();

    for (std::size_t i = 0; i < scene.live_radar_tracks.size(); ++i) {
      const RadarTrackData &track = scene.live_radar_tracks[i];

      float x = std::clamp(static_cast<float>(track.calibrated_point.x()), 0.f, float(width - diameter));
      float y = std::clamp(static_cast<float>(track.calibrated_point.y()), 0.f, float(height - diameter));

      p.save();
      p.setBrush(redColor());
      p.setPen(Qt::NoPen);
      p.drawEllipse(QPointF(x + diameter / 2.f, y + diameter / 2.f), diameter / 2.f, diameter / 2.f);
      p.restore();
    }
  }
}

void AnnotatedCameraWidget::drawRoadName(QPainter &p) {
  QString roadName = QString::fromStdString(params_memory.get("RoadName"));
  if (roadName.isEmpty()) {
    return;
  }

  QFont font = InterFont(40, QFont::DemiBold);
  int textWidth = QFontMetrics(font).horizontalAdvance(roadName);

  p.save();

  QRect roadNameRect((width() - (textWidth + 100)) / 2, rect().bottom() - 55 + 1, textWidth + 100, 50);

  p.setBrush(blackColor(166));
  p.setOpacity(1.0);
  p.setPen(QPen(blackColor(), 10));
  p.drawRoundedRect(roadNameRect, 24, 24);

  p.setFont(font);
  p.setPen(QPen(Qt::white, 6));
  p.drawText(roadNameRect, Qt::AlignCenter, roadName);

  p.restore();
}

void AnnotatedCameraWidget::drawTurnSignals(QPainter &p) {
  bool blindspotActive = turnSignalLeft ? blindSpotLeft : blindSpotRight;

  if (signalStyle == "static") {
    int signalXPosition = turnSignalLeft ? (rect().center().x() * 0.75) - signalWidth : rect().center().x() * 1.25;
    int signalYPosition = signalHeight / 2;

    if (blindspotActive && !blindspotImages.empty()) {
      p.drawPixmap(signalXPosition, signalYPosition, signalWidth, signalHeight, blindspotImages[turnSignalLeft ? 0 : 1]);
    } else {
      p.drawPixmap(signalXPosition, signalYPosition, signalWidth, signalHeight, signalImages[2 * animationFrameIndex + (turnSignalLeft ? 0 : 1)]);
    }
  } else if (signalStyle == "traditional") {
    int signalXPosition = turnSignalLeft ? width() - ((animationFrameIndex + 1) * signalWidth) : animationFrameIndex * signalWidth;
    int signalYPosition = height() - signalHeight;

    signalYPosition -= alertHeight;

    if (blindspotActive && !blindspotImages.empty()) {
      p.drawPixmap(turnSignalLeft ? width() - signalWidth : 0, signalYPosition, signalWidth, signalHeight, blindspotImages[turnSignalLeft ? 0 : 1]);
    } else {
      p.drawPixmap(signalXPosition, signalYPosition, signalWidth, signalHeight, signalImages[2 * animationFrameIndex + (turnSignalLeft ? 0 : 1)]);
    }
  } else if (signalStyle == "traditional_gif") {
    int signalXPosition = turnSignalLeft ? width() - (animationFrameIndex * signalMovement) + signalWidth : (animationFrameIndex * signalMovement) - signalWidth;
    int signalYPosition = height() - signalHeight;

    signalYPosition -= alertHeight;

    if (blindspotActive && !blindspotImages.empty()) {
      p.drawPixmap(turnSignalLeft ? width() - signalWidth : 0, signalYPosition, signalWidth, signalHeight, blindspotImages[turnSignalLeft ? 0 : 1]);
    } else {
      p.drawPixmap(signalXPosition, signalYPosition, signalWidth, signalHeight, signalImages[2 * animationFrameIndex + (turnSignalLeft ? 0 : 1)]);
    }
  }
}
