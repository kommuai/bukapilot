#include "selfdrive/ui/qt/home.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPainter>
#include <QString>
#include <QTextOption>
#include <cstdlib>
#include <QrCode.hpp>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/util.h"

using qrcodegen::QrCode;


QFrame *home_horizontal_line(QWidget *parent) {
  QFrame *line = new QFrame(parent);
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

StatusLabel::StatusLabel( const QString &text, const QString &icon_dir, QWidget *parent) : QWidget( parent){

    this->icon_dir = icon_dir;
    this->text = text;

    update();
}

void StatusLabel::setText(const QString &text)
{
    if (this->text == text)
        return;
    this->text = text;
    update();
}

void StatusLabel::setIconDir(const QString &icon_dir)
{
    if (this->icon_dir == icon_dir)
        return;
    this->icon_dir = icon_dir;
    update();
}


void StatusLabel::paintEvent(QPaintEvent *e) {
  QWidget::paintEvent(e);
  QTextOption option (Qt::AlignCenter);
  option.setWrapMode(QTextOption:: WordWrap);
  QPainter p(this);
  QPixmap icon(icon_dir);
  p.drawPixmap(width()*0.11,0,width()*0.78,width()*0.78, icon);
  QFont font=p.font() ;
  int font_size = 40;
  font.setPixelSize(font_size);
  p.setFont(font);
  font.setWeight(QFont::DemiBold);
  p.drawText(QRect(width()*0.26,width()*0.2,width()*0.5,width()*0.38), text, option);

}



// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);

  QObject::connect(sidebar, &Sidebar::openTerms, this, &HomeWindow::openTerms);
  QObject::connect(sidebar, &Sidebar::openTraining, this, &HomeWindow::openTraining);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome();
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);

  QObject::connect(this, &HomeWindow::update, home->status, &StatusWidget::updateState);
  QObject::connect(this, &HomeWindow::update, home->drive, &DriveWidget::updateState);
  QObject::connect(this, &HomeWindow::update, onroad, &OnroadWindow::update);
  QObject::connect(this, &HomeWindow::offroadTransitionSignal, onroad, &OnroadWindow::offroadTransitionSignal);
  QObject::connect(onroad, &OnroadWindow::openSettings, this, &HomeWindow::openSettings);

  driver_view = new DriverViewWindow(this);
  connect(driver_view, &DriverViewWindow::done, [=] {
    showDriverView(false);
  });
  slayout->addWidget(driver_view);
}

void HomeWindow::showSidebar(bool show) {
  sidebar->setVisible(show);
}

void HomeWindow::offroadTransition(bool offroad) {
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }

  sidebar->setVisible(offroad);
  emit offroadTransitionSignal(offroad);
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

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {

  QGridLayout* main_layout = new QGridLayout(this);
  main_layout->setContentsMargins(0,25,25,25);

  status = new StatusWidget();
  updates = new UpdatesWidget();
  qr = new QrWidget();
  drive = new DriveWidget();
  status->setMaximumWidth(450);

  drive -> setAttribute(Qt::WA_StyledBackground);
  status -> setAttribute(Qt::WA_StyledBackground);

  main_layout->addWidget(status,0,0,2,1);
  main_layout->addWidget(qr,0,1,1,2);
  main_layout->addWidget(updates,1,1);
  main_layout->addWidget(drive,1,2);
  main_layout->setHorizontalSpacing(35);

  // set up refresh timer
  timer = new QTimer(this);
  setStyleSheet(R"(
    * {
     color: white;
    }
    OffroadHome {
      background-color: black;
    }
    OffroadHome > QPushButton {
      padding: 15px 30px;
      border-radius: 5px;
      font-size: 40px;
      font-weight: 500;
    }
    OffroadHome > QLabel {
      font-size: 55px;
    }
  )");

}



void StatusWidget::updateState(const UIState &s) {

  auto &sm = *(s.sm);

  QString status_pixmap_dir = "../assets/kommu/green_circle.png";
  QString status_text = "SYSTEM READY";

  auto deviceState = sm["deviceState"].getDeviceState();
  QString temp_pixmap_dir = "../assets/kommu/red_circle.png";
  auto ts = deviceState.getThermalStatus();

  if (ts == cereal::DeviceState::ThermalStatus::GREEN) {
    temp_pixmap_dir = "../assets/kommu/green_circle.png";
  } else if (ts == cereal::DeviceState::ThermalStatus::YELLOW) {
    temp_pixmap_dir = "../assets/kommu/yellow_circle.png";
  }

  if (s.scene.pandaType == cereal::PandaState::PandaType::UNKNOWN) {
    status_pixmap_dir = "../assets/kommu/red_circle.png";
    status_text = "DEVICE ERROR";
  }

  device_txt -> setIconDir(status_pixmap_dir);
  device_txt -> setText(status_text);
  temp_txt -> setIconDir(temp_pixmap_dir);
  temp_txt -> setText(QString::number((int)deviceState.getAmbientTempC()) + " °C");
}

void DriveWidget::updateState(const UIState &s) {

  auto &sm = *(s.sm);
  auto uploaderState = sm["uploaderState"].getUploaderState();
  uint32_t remainingUploadSize = uploaderState.getImmediateQueueSize() + uploaderState.getRawQueueSize();
  uint32_t remainingUploadCount = uploaderState.getImmediateQueueCount() + uploaderState.getRawQueueCount();
  float uploadSpeed = uploaderState.getLastSpeed();

  if (remainingUploadCount == 0) {
    rem_upl_val -> setText("0 MB");
    upl_spd_val -> setText("0 MB/s");
  } else {
    rem_upl_val -> setText(QString::number(remainingUploadSize) + " MB");
    upl_spd_val -> setText(QString::number((int)uploadSpeed) + " MB/s");
  }

}


void OffroadHome::showEvent(QShowEvent *event) {
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

StatusWidget::StatusWidget(QWidget *parent) : QWidget(parent){
    status_layout = new QVBoxLayout(this);
    QString device_img_dir = "../assets/kommu/green_circle.png";
    device_txt = new StatusLabel("SYSTEM READY",device_img_dir);

    QLabel *device_temp_label = new QLabel("DEVICE TEMPERATURE");
    device_temp_label -> setWordWrap(true);
    device_temp_label -> setAlignment( Qt::AlignCenter);
    QFont label_font;
    label_font.setPixelSize(50);
    device_temp_label -> setFont(label_font);
    device_temp_label->setFixedWidth(375);
    device_temp_label->setFixedHeight(100);

    QString temp_img_dir = "../assets/kommu/red_circle.png";
    temp_txt = new StatusLabel("0 °C",temp_img_dir);

    status_layout->addSpacing(50);
    status_layout -> addWidget(device_txt);
    device_txt->setStyleSheet("padding-top:300px;");
    device_txt -> setMinimumHeight(250);
    status_layout -> addWidget(home_horizontal_line());
    status_layout -> addWidget(device_temp_label);
    status_layout -> addWidget(temp_txt);
    status_layout -> setSpacing(50);
    setStyleSheet("background-color: rgb(32, 32, 32);border-radius: 25px;");
}

QrWidget::QrWidget(QWidget *parent) : QWidget(parent){

    qr_layout = new QHBoxLayout(this);
    setStyleSheet("background-color: rgb(0, 0, 0);");
    QLabel *qr_desc = new QLabel("Pair your device with KommuApp by scanning the QR Code to view drive data.");
    QLabel *qr_code = new QLabel;
    qr_desc->setStyleSheet("font-size: 50px;padding: 15px");
    qr_desc->setWordWrap(true);

    Params params = Params();
    QString qr_link = "https://linktree.kommu.ai/";
    QString qr_dongle = QString::fromStdString(params.get("DongleId", false));
    QString qr_text = qr_link + qr_dongle;
    QrCode qr = QrCode::encodeText(qr_text.toUtf8().data(), QrCode::Ecc::LOW);
    qint32 sz = qr.getSize();
    // make the image larger so we can have a white border
    QImage im(sz + 2, sz + 2, QImage::Format_RGB32);
    QRgb black = qRgb(0, 0, 0);
    QRgb white = qRgb(255, 255, 255);

    for (int y = 0; y < sz + 2; y++) {
    for (int x = 0; x < sz + 2; x++) {
      im.setPixel(x, y, white);
    }
    }
    for (int y = 0; y < sz; y++) {
    for (int x = 0; x < sz; x++) {
      im.setPixel(x + 1, y + 1, qr.getModule(x, y) ? black : white);
    }
    }
    // Integer division to prevent anti-aliasing
    int approx300 = (300 / (sz + 2)) * (sz + 2);
    qr_code->setPixmap(QPixmap::fromImage(im.scaled(approx300, approx300, Qt::KeepAspectRatio, Qt::FastTransformation), Qt::MonoOnly));
    qr_code->setFixedSize(approx300, approx300);

    qr_layout->addWidget(qr_desc);
    qr_layout->addWidget(qr_code);


}


DriveWidget::DriveWidget(QWidget *parent) : QWidget(parent){
    drive_layout = new QVBoxLayout(this);
    QLabel *drive_header = new QLabel("Drive Data");
    QLabel *rem_upl_txt = new QLabel("Remaining Upload");
    rem_upl_val = new QLabel("Loading...");
    QLabel *upl_spd_txt = new QLabel("Upload Speed");
    upl_spd_val = new QLabel("Loading...");

    drive_layout->addWidget(drive_header);
    drive_layout->addWidget(rem_upl_txt);
    drive_layout->addWidget(rem_upl_val);
    drive_layout->addWidget(home_horizontal_line());
    drive_layout->addWidget(upl_spd_txt);
    drive_layout->addWidget(upl_spd_val);

    QFont header_font;
    QFont content_font;
    content_font.setPixelSize(40);
    header_font.setPixelSize(50);
    drive_header -> setFont(header_font);
    rem_upl_txt -> setFont(content_font);
    rem_upl_val -> setFont(content_font);
    upl_spd_txt -> setFont(content_font);
    upl_spd_val -> setFont(content_font);

    rem_upl_val -> setAlignment(Qt::AlignRight);
    upl_spd_val -> setAlignment(Qt::AlignRight);
    drive_layout->setContentsMargins(50,50,50,50);

    setStyleSheet("background-color: rgb(32, 32, 32);border-radius: 25px;");
}

UpdatesWidget::UpdatesWidget(QWidget *parent) : QWidget(parent){
    update_layout = new QVBoxLayout(this);
    setStyleSheet("background-color: rgb(32, 32, 32);border-radius: 25px;");
    QLabel *updates_header = new QLabel("Bukapilot v1.0.0");
    QLabel *updates_content = new QLabel("-Update UI \n-Add Support for Axia, Ativa, Alza, Aruz, Bezza and Proton Cars");


    QFont header_font;
    QFont content_font;

    content_font.setPixelSize(40);
    header_font.setPixelSize(50);

    updates_header->setFont(header_font);
    updates_header->setAlignment(Qt::AlignTop);
    updates_header->setWordWrap(true);
    updates_header->setFixedHeight(65);
    updates_content->setAlignment(Qt::AlignTop|Qt::AlignLeft);
    updates_content->setFont(content_font);
    updates_content->setWordWrap(true);
    updates_content->setFixedHeight(200);

    update_button = new QPushButton("UPDATE");
    update_button->setFixedSize(QSize(300,100));
    update_button->setStyleSheet("background-color: rgb(75, 75, 75);font-size:40px;border-radius: 50px;");
    update_layout->setContentsMargins(50,50,50,50);
    update_layout->addWidget(updates_header);
    update_layout->addWidget(updates_content);
    update_layout->addWidget(update_button,0,Qt::AlignRight|Qt::AlignBottom);

}

