#define K_IMPL
#include "selfdrive/ui/qt/k_home.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>

#include <QrCode.hpp>

#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/popup.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  connect(sidebar, &Sidebar::openAlerts, this, [=] {
    (new Popup("Alerts", alerts.text, Popup::OK, this))->exec();
  });
  connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);
  connect(sidebar, &Sidebar::openTerms, this, &HomeWindow::openTerms);
  connect(sidebar, &Sidebar::openTraining, this, &HomeWindow::openTraining);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome();
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  connect(onroad, &OnroadWindow::openSettings, this, &HomeWindow::openSettings);
  slayout->addWidget(onroad);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
  setAttribute(Qt::WA_NoSystemBackground);
  connect(uiState(), &UIState::offroadTransition, this, &HomeWindow::offroadTransition);
  connect(uiState(), &UIState::uiUpdate, home, &OffroadHome::updateState);

  timer = new QTimer(this);
  timer->callOnTimeout(this, &HomeWindow::refresh);
}

void HomeWindow::showEvent(QShowEvent *event) {
  refresh();
  timer->start(10 * 1000);
}

void HomeWindow::hideEvent(QHideEvent *event) {
  timer->stop();
}

void HomeWindow::refresh() {
  alerts.refresh();
  sidebar->setAlertCount(alerts.unread);
  home->hasSevereAlerts = alerts.hasSevere;
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::offroadTransition(bool offroad) {
  sidebar->setVisible(offroad);
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
}

void HomeWindow::showDriverView(bool show) {
  if (show) {
    emit closeSettings();
    slayout->setCurrentWidget(driver_view);
  } else {
    slayout->setCurrentWidget(home);
  }
  sidebar->setVisible(show == false);
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Handle sidebar collapsing
  if (onroad->isVisible() && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible() && !onroad->isMapVisible());
  }
}

StatusLabel::StatusLabel(const QString& text, QPixmap icon, QWidget *parent = 0)
  : QWidget(parent), text(text), icon(icon) {
  font.setPixelSize(40);
  font.setWeight(QFont::DemiBold);
  opt.setWrapMode(QTextOption::WordWrap);
  setMinimumWidth(QFontMetrics(font).horizontalAdvance(text) + 60);
}

void StatusLabel::paintEvent(QPaintEvent *e) {
  QWidget::paintEvent(e);
  QPainter p(this);
  p.setFont(font);
  p.drawPixmap(width() * 0.11, 0, width() * 0.78, width() * 0.78, icon);
  p.drawText(QRect(width() * 0.26, width() * 0.2, width() * 0.5, width() * 0.38), text, opt);
}

QFrame *horizontal_rule(QWidget *parent) {
  auto line = new QFrame(parent);
  line->setFrameShape(QFrame::StyledPanel);
  line->setStyleSheet(R"(
    margin-left: 90px;
    margin-right: 100px;
    border-width: 1px;
    border-bottom-style: solid;
    border-color: gray;
  )");
  line->setFixedHeight(2);
  return line;
}

QrWidget::QrWidget(const char *content, QWidget *parent) : QWidget(parent) {
  setContent(content);
}

void QrWidget::paintEvent(QPaintEvent *e) {
  QPainter p(this);
  p.fillRect(rect(), Qt::white);
  p.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform, false);
  const int pad = 20;
  p.drawPixmap(pad, pad, width() - 2 * pad, height() - 2 * pad, img);
}

void QrWidget::setContent(const char *content) {
  using qrcodegen::QrCode;

  QrCode qr = QrCode::encodeText(content, QrCode::Ecc::LOW);
  qint32 sz = qr.getSize();
  QImage im(sz, sz, QImage::Format_RGB32);

  QRgb black = qRgb(0, 0, 0);
  QRgb white = qRgb(255, 255, 255);
  for (int y = 0; y < sz; y++) {
    for (int x = 0; x < sz; x++) {
      im.setPixel(x, y, qr.getModule(x, y) ? black : white);
    }
  }
  img = QPixmap::fromImage(im, Qt::MonoOnly);
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  auto main_layout = new QGridLayout(this);
  main_layout->setContentsMargins(25, 25, 25, 25);
  main_layout->setHorizontalSpacing(35);

  auto status = new QWidget(this);
  status->setAttribute(Qt::WA_StyledBackground);
  status->setFixedWidth(500);
  status->setStyleSheet("background-color: rgb(32, 32, 32); border-radius: 25px;");
  auto status_layout = new QVBoxLayout(status);
  status_layout->setContentsMargins(50, 50, 50, 50);
  status_layout->setSpacing(50);
  device_text = new StatusLabel("SYSTEM\nREADY", status_icons[cereal::DeviceState::ThermalStatus::GREEN], this);
  device_text->setStyleSheet("padding-top: 300px;");
  device_text->setMinimumHeight(250);
  status_layout->addWidget(device_text);
  status_layout->addWidget(horizontal_rule(this));
  auto device_temperature_label = new QLabel("DEVICE TEMPERATURE");
  device_temperature_label->setStyleSheet("font-size: 50px;");
  device_temperature_label->setWordWrap(true);
  device_temperature_label->setAlignment(Qt::AlignHCenter);
  device_temperature_label->setFixedHeight(150);
  status_layout->addWidget(device_temperature_label);
  temperature_text = new StatusLabel("0 °C", status_icons[cereal::DeviceState::ThermalStatus::RED], this);
  status_layout->addWidget(temperature_text);
  main_layout->addWidget(status, 0, 0, 2, 1);

  auto qr = new QWidget(this);
  qr->setAttribute(Qt::WA_StyledBackground);
  auto qr_layout = new QHBoxLayout(qr);
  auto qr_label = new QLabel("Pair your device with KommuApp by scanning the QR code to view drive data.");
  qr_label->setStyleSheet("font-size: 50px; padding: 15px;");
  qr_label->setWordWrap(true);
  qr_label->setMaximumWidth(650);
  qr_layout->addWidget(qr_label);
  auto qr_code = new QrWidget("https://kommu.ai", this);
  qr_code->setFixedSize(300, 300);
  qr_layout->addWidget(qr_code);
  main_layout->addWidget(qr, 0, 1, 1, 2);

  auto updates = new ClickableWidget(this);
  updates->setAttribute(Qt::WA_StyledBackground);
  updates->setStyleSheet("background-color: rgb(32, 32, 32); border-radius: 25px; font-size: 40px;");
  auto updates_layout = new QVBoxLayout(updates);
  updates_layout->setContentsMargins(50, 50, 50, 50);
  updates_text = new QLabel("loading...");
  updates_text->setAlignment(Qt::AlignTop);
  updates_text->setFixedHeight(425);
  updates_text->setWordWrap(true);
  updates_layout->addWidget(updates_text);
  connect(updates, &ClickableWidget::clicked, [=] {
    (new Popup("Release Notes", updates_text->text(), Popup::OK, this))->exec();
  });
  main_layout->addWidget(updates, 1, 1);

  auto drive = new QWidget(this);
  drive->setAttribute(Qt::WA_StyledBackground);
  drive->setStyleSheet("background-color: rgb(32, 32, 32); border-radius: 25px; font-size: 40px;");
  auto drive_layout = new QVBoxLayout(drive);
  drive_layout->setContentsMargins(50, 50, 50, 50);
  auto drive_header = new QLabel("Drive Data");
  drive_header->setStyleSheet("font-size: 50px;");
  drive_layout->addWidget(drive_header);
  drive_layout->addWidget(new QLabel("Remaining Upload"));
  remaining_upload = new QLabel("loading...");
  remaining_upload->setAlignment(Qt::AlignRight);
  drive_layout->addWidget(remaining_upload);
  drive_layout->addWidget(horizontal_rule(this));
  drive_layout->addWidget(new QLabel("Upload Speed"));
  upload_speed = new QLabel("loading...");
  upload_speed->setAlignment(Qt::AlignRight);
  drive_layout->addWidget(upload_speed);
  main_layout->addWidget(drive, 1, 2);

  setStyleSheet(R"(
    * {
     color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QLabel {
      font-size: 50px;
    }
  )");
}

void OffroadHome::updateState(const UIState& s) {
  auto& sm = *(s.sm);

  bool hasError = false;
  hasError |= s.scene.pandaType == cereal::PandaState::PandaType::UNKNOWN;

  bool initialising = false;
  initialising |= hasSevereAlerts;

  if (hasError) {
    device_text->icon = status_icons[cereal::DeviceState::ThermalStatus::RED];
    device_text->text = "DEVICE ERROR";
  }
  else if (initialising) {
    device_text->icon = status_icons[cereal::DeviceState::ThermalStatus::YELLOW];
    device_text->text = "GETTING READY";
  } else {
    device_text->icon = status_icons[cereal::DeviceState::ThermalStatus::GREEN];
    device_text->text = "SYSTEM READY";
  }
  device_text->update();

  auto deviceState = sm["deviceState"].getDeviceState();
  temperature_text->icon = status_icons[deviceState.getThermalStatus()];
  temperature_text->text = QString::number((int)deviceState.getAmbientTempC()) + "°C";
  temperature_text->update();

  auto uploaderState = sm["uploaderState"].getUploaderState();
  remaining_upload->setText(QString::number(uploaderState.getImmediateQueueSize() + uploaderState.getRawQueueSize()) + " MB");
  upload_speed->setText(QString::number(uploaderState.getLastSpeed()) + " MB/s");

  auto newUpdateState = (int) params.getBool("UpdateAvailable");
  if (update_state != newUpdateState) {
    update_state = newUpdateState;
    updates_text->setText(QString::fromStdString(
        ((update_state) ?
          "An update is available!\nNext version:\n\n" :
          "You're up to date!\nCurrent version:\n\n")
        + params.get("ReleaseNotes")));
  }
}
