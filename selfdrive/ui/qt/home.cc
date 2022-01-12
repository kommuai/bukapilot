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
#include "selfdrive/ui/qt/widgets/drive_stats.h"
#include "selfdrive/ui/qt/widgets/setup.h"

using qrcodegen::QrCode;


QFrame *home_horizontal_line(QWidget *parent) {
  QFrame *line = new QFrame(parent);
  line->setFrameShape(QFrame::StyledPanel);
  line->setStyleSheet(R"(
    margin-left: 100px;
    margin-right: 100px;
    border-width: 1px;
    border-bottom-style: solid;
    border-color: gray;
  )");
  line->setFixedHeight(1);
  return line;
}

StatusLabel::StatusLabel( const QString &text, const QPixmap &icon, QWidget *parent) : QWidget( parent){

    this->icon = icon;
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

void StatusLabel::setIcon(const QPixmap &icon)
{
    if (this->icon == icon)
        return;
    this->icon = icon;
    update();
}


void StatusLabel::paintEvent(QPaintEvent *e) {
  QWidget::paintEvent(e);
  QTextOption option (Qt::AlignCenter);
  option.setWrapMode(QTextOption:: WordWrap);
  QPainter p(this);
  p.drawPixmap(width()*0.2,width()*0.05,width()*0.6,width()*0.6, icon);
  QFont font=p.font() ;
  int font_size = 25;
  font.setPointSize(font_size);
  p.setFont(font);
  font.setWeight(QFont::DemiBold);
//  p.drawText(QPoint(width()/2,(height()/2)),button_text);
 // p.drawText(QRect(width()/2 - button_text.size()*font_size/2.75,height()/2+icon.height()/1.5),Qt::AlignCenter,button_text);
  p.drawText(QRect(width()*0.32,width()*0.16,width()*0.37,width()*0.38), text, option);

}



// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *main_layout = new QHBoxLayout(this);
  main_layout->setMargin(0);
  main_layout->setSpacing(0);

  sidebar = new Sidebar(this);
  main_layout->addWidget(sidebar);
  QObject::connect(this, &HomeWindow::update, sidebar, &Sidebar::updateState);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  main_layout->addLayout(slayout);

  home = new OffroadHome();
  slayout->addWidget(home);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);

  QObject::connect(this, &HomeWindow::update, onroad, &OnroadWindow::update);
  QObject::connect(this, &HomeWindow::offroadTransitionSignal, onroad, &OnroadWindow::offroadTransitionSignal);

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

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // Handle sidebar collapsing
  // if (onroad->isVisible() && (!sidebar->isVisible() || e->x() > sidebar->width())) {

  //   // TODO: Handle this without exposing pointer to map widget
  //   // Hide map first if visible, then hide sidebar
  //   if (onroad->map != nullptr && onroad->map->isVisible()) {
  //     onroad->map->setVisible(false);
  //   } else if (!sidebar->isVisible()) {
  //     sidebar->setVisible(true);
  //   } else {
  //     sidebar->setVisible(false);

  //     if (onroad->map != nullptr) onroad->map->setVisible(true);
  //   }
  // }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {

  QGridLayout* main_layout = new QGridLayout(this);
  main_layout->setMargin(50);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setSpacing(10);

  date = new QLabel();
  status = new StatusWidget();
  updates = new UpdatesWidget();
  qr = new QrWidget();
  drive = new DriveWidget();
  header_layout->addWidget(date, 1, Qt::AlignHCenter | Qt::AlignLeft);
  // main_layout->addLayout(header_layout);
  status->setMaximumWidth(450);

  main_layout->addWidget(status,0,0,2,1);
  main_layout->addWidget(qr,0,1,1,2);
  main_layout->addWidget(updates,1,1);
  main_layout->addWidget(drive,1,2);
  main_layout->setHorizontalSpacing(15);


  // main content
  // main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QWidget* statsAndSetupWidget = new QWidget(this);
  QHBoxLayout* statsAndSetup = new QHBoxLayout(statsAndSetupWidget);
  statsAndSetup->setMargin(0);

  center_layout->addWidget(statsAndSetupWidget);



  //main_layout->addLayout(center_layout, 1);
  
  // set up refresh timer
  timer = new QTimer(this);
  timer->callOnTimeout(this, &OffroadHome::refresh);

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

  refresh();
}



void StatusWidget::updateState(const UIState &s) {

  auto &sm = *(s.sm);

  auto deviceState = sm["deviceState"].getDeviceState();
  // setProperty("netType", network_type[deviceState.getNetworkType()]);
  // int strength = (int)deviceState.getNetworkStrength();
  // setProperty("netStrength", strength > 0 ? strength + 1 : 0);

  // auto last_ping = deviceState.getLastAthenaPingTime();
  // if (last_ping == 0) {
  //   setProperty("connectStr", "OFFLINE");
  //   setProperty("connectStatus", warning_color);
  // } else {
  //   bool online = nanos_since_boot() - last_ping < 80e9;
  //   setProperty("connectStr",  online ? "ONLINE" : "ERROR");
  //   setProperty("connectStatus", online ? good_color : danger_color);
  // }

  QString temp_pixmap_dir = "../assets/kommu/red_circle.png";
  auto ts = deviceState.getThermalStatus();
  if (ts == cereal::DeviceState::ThermalStatus::GREEN) {
    temp_pixmap_dir = "../assets/kommu/blue_circle.png";
  } else if (ts == cereal::DeviceState::ThermalStatus::YELLOW) {
    temp_pixmap_dir = "../assets/kommu/green_circle.png";
  }
  QPixmap temp_pixmap(temp_pixmap_dir);
  // setProperty("tempStatus", tempStatus);
  // setProperty("tempVal", (int)deviceState.getAmbientTempC());
  temp_txt -> setIcon(temp_pixmap);
  temp_txt -> setText((QString)((int)deviceState.getAmbientTempC()) + " °C");
  // QString pandaStr = "VEHICLE\nONLINE";
  // QColor pandaStatus = good_color;
  // if (s.scene.pandaType == cereal::PandaState::PandaType::UNKNOWN) {
  //   pandaStatus = danger_color;
  //   pandaStr = "NO\nPANDA";
  // } else if (s.scene.started && !sm["liveLocationKalman"].getLiveLocationKalman().getGpsOK()) {
  //   pandaStatus = warning_color;
  //   pandaStr = "GPS\nSEARCHING";
  // }
  // setProperty("pandaStr", pandaStr);
  // setProperty("pandaStatus", pandaStatus);
}


void OffroadHome::showEvent(QShowEvent *event) {
  timer->start(10 * 1000);
}

void OffroadHome::hideEvent(QHideEvent *event) {
  timer->stop();
}

void OffroadHome::refresh() {
  date->setText(QDateTime::currentDateTime().toString("dddd, MMMM d"));

  // bool updateAvailable = update_widget->refresh();
  // int alerts = alerts_widget->refresh();

  // // pop-up new notification
  // int idx = center_layout->currentIndex();
  // // Kommu Edits:
  // //if (!updateAvailable && !alerts) {
  // if (!updateAvailable){
  //   idx = 0;
  // } else if (updateAvailable && (!update_notif->isVisible() || (!alerts && idx == 2))) {
  //   idx = 1;
  // } else if (alerts && (!alert_notif->isVisible() || (!updateAvailable && idx == 1))) {
  //   idx = 2;
  // }
  // center_layout->setCurrentIndex(idx);

  // update_notif->setVisible(updateAvailable);
  // alert_notif->setVisible(alerts);
  // if (alerts) {
  //   alert_notif->setText(QString::number(alerts) + " ALERT" + (alerts > 1 ? "S" : ""));
  // }
}


StatusWidget::StatusWidget(QWidget *parent) : QWidget(parent){
    status_layout = new QVBoxLayout(this);
    QString device_img_dir = "../assets/kommu/green_circle.png";
    QPixmap device_pixmap(device_img_dir);
    // device_img = new QLabel("SYSTEM READY");
    // device_img -> setPixmap(device_pixmap);
    // QString *asdf = new QString("test");
    device_txt = new StatusLabel("SYSTEM READY",device_pixmap);

    QLabel *device_temp_label = new QLabel("DEVICE TEMPERATURE");
    device_temp_label -> setWordWrap(true);
    device_temp_label -> setAlignment( Qt::AlignCenter);
    device_temp_label->setStyleSheet("padding-left:75px;padding-right:100px");
    QFont label_font;
    label_font.setPointSize(25);
    device_temp_label -> setFont(label_font);
    device_temp_label->setFixedWidth(450);
    device_temp_label->setFixedHeight(75);

    QString temp_img_dir = "../assets/kommu/red_circle.png";
    QPixmap temp_pixmap(temp_img_dir);
//    temp_img = new QLabel();
//    temp_img -> setPixmap(temp_img_dir);

    temp_txt = new StatusLabel("0 °C",temp_pixmap);

    //status_layout -> addWidget(device_img);
    status_layout->addSpacing(50);
    status_layout -> addWidget(device_txt);
    device_txt->setStyleSheet("padding-top:300px;");
    device_txt -> setMinimumHeight(250);
    status_layout -> addWidget(home_horizontal_line());
    status_layout -> addWidget(device_temp_label);
    //status_layout -> addWidget(device_img);
    status_layout -> addWidget(temp_txt);
    status_layout -> setSpacing(50);
   // setStyleSheet(R"(*{background-color: rgb(40, 40, 40);border-radius: 25px})");
}


QrWidget::QrWidget(QWidget *parent) : QWidget(parent){
    
    qr_layout = new QHBoxLayout(this);
    setStyleSheet("background-color: rgb(0, 0, 0);");
    QLabel *qr_desc = new QLabel("Pair your device with KommuApp by scanning the QR Code to view drive data.");
    QLabel *qr_code = new QLabel;
    qr_desc->setStyleSheet("font-size: 40px;padding: 15px");
    qr_desc->setWordWrap(true);

    Params params = Params();
    QString qr_text = QString::fromStdString(params.get("DongleId", false));
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
    QLabel *rem_upl_val = new QLabel("0 MB");
    QLabel *upl_spd_txt = new QLabel("Upload Speed");
    QLabel *upl_spd_val = new QLabel("0 kB/s");

    drive_layout->addWidget(drive_header);
    drive_layout->addWidget(rem_upl_txt);
    drive_layout->addWidget(rem_upl_val);
    drive_layout->addWidget(home_horizontal_line());
    drive_layout->addWidget(upl_spd_txt);
    drive_layout->addWidget(upl_spd_val);

    QFont header_font;
    QFont content_font;
    content_font.setPointSize(20);
    header_font.setPointSize(25);

    drive_header -> setFont(content_font);
    rem_upl_txt -> setFont(content_font);
    rem_upl_val -> setFont(content_font);
    upl_spd_txt -> setFont(content_font);
    upl_spd_val -> setFont(content_font);

    rem_upl_val -> setAlignment(Qt::AlignRight);
    upl_spd_val -> setAlignment(Qt::AlignRight);
    drive_layout->setContentsMargins(50,50,50,50);

    setStyleSheet("background-color: rgb(40, 40, 40);border-radius: 25px;");
}

UpdatesWidget::UpdatesWidget(QWidget *parent) : QWidget(parent){
    update_layout = new QVBoxLayout(this);
    setStyleSheet("background-color: rgb(40, 40, 40);border-radius: 25px;");
    QLabel *updates_header = new QLabel("bukapilot 1.0.0");
    QLabel *updates_content = new QLabel("bukapilot 1.0.0\n-To be added in the future");


    QFont header_font;
    QFont content_font;

    content_font.setPointSize(15);
    header_font.setPointSize(25);
   // header_font.thin()

    updates_header->setFont(header_font);
    updates_header->setAlignment(Qt::AlignTop);
    updates_header->setFixedHeight(50);
    updates_content -> setAlignment(Qt::AlignTop|Qt::AlignLeft);
    updates_content->setFont(content_font);
    updates_content->setFixedHeight(250);

    update_button = new QPushButton("UPDATE");
    update_button -> setFixedSize(QSize(300,100));
    update_button -> setStyleSheet("background-color: rgb(100, 100, 100);font-size:40px;border-radius: 50px");
    update_layout->setContentsMargins(50,35,15,15);
    update_layout->addWidget(updates_header);
    update_layout->addWidget(updates_content);
    update_layout->addWidget(update_button,0,Qt::AlignRight|Qt::AlignBottom);

}

