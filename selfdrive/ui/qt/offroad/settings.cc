#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <string>

#include <QDebug>
#include <QPixmap>

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "cereal/gen/cpp/log.capnp.h"
#include "cereal/messaging/messaging.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/qt/widgets/nav_buttons.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"

TogglesPanel::TogglesPanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);

  QList<ParamControl*> toggles;

  toggles.append(new ParamControl("OpenpilotEnabledToggle",
                                  "Enable Bukapilot",
                                  "Use the bukapilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off.",
                                  "../assets/kommu/settings/kommu_steering_wheel_icon.png",
                                  this));
  toggles.append(new ParamControl("IsLdwEnabled",
                                  "Enable Lane Departure Warnings",
                                  "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31mph (50kph).",
                                  "../assets/kommu/settings/kommu_alert_sign.png",
                                  this));
  toggles.append(new ParamControl("IsRHD",
                                  "Enable Right-Hand Drive",
                                  "Allow bukapilot to obey left-hand traffic conventions and perform driver monitoring on right driver seat.",
                                  "../assets/kommu/settings/kommu_steering_wheel_icon_inverted.png",
                                  this));
  toggles.append(new ParamControl("IsMetric",
                                  "Use Metric System",
                                  "Display speed in km/h instead of mp/h.",
                                  "../assets/offroad/icon_metric.png",
                                  this));
  toggles.append(new ParamControl("CommunityFeaturesToggle",
                                  "Enable Community Features",
                                  "Use features from the open source community that are not maintained or supported by kommu.ai and have not been confirmed to meet the standard safety model. These features include community supported cars and community supported hardware. Be extra cautious when using these features",
                                  "../assets/kommu/settings/kommu_developer_icon.png",
                                  this));

  toggles.append(new ParamControl("UploadRaw",
                                  "Upload Raw Logs",
                                  "Upload full logs and full resolution video by default while on WiFi. If not enabled, individual logs can be marked for upload at my.comma.ai/useradmin.",
                                  "../assets/kommu/settings/kommu_upload_icon.png",
                                  this));

  ParamControl *record_toggle = new ParamControl("RecordFront",
                                                 "Record and Upload Driver Camera",
                                                "Upload data from the driver facing camera and help improve the driver monitoring algorithm.",
                                                "../assets/offroad/icon_monitoring.png",
                                                this);
  toggles.append(record_toggle);
  toggles.append(new ParamControl("EndToEndToggle",
                                   "\U0001f96c Disable use of lanelines (Alpha) \U0001f96c",
                                   "In this mode bukapilot will ignore lanelines and just drive how it thinks a human would.",
                                   "../assets/offroad/icon_road.png",
                                   this));

  if (Hardware::TICI()) {
    toggles.append(new ParamControl("EnableWideCamera",
                                    "Enable use of Wide Angle Camera",
                                    "Use wide angle camera for driving and ui.",
                                    "../assets/offroad/icon_openpilot.png",
                                    this));
    QObject::connect(toggles.back(), &ToggleControl::toggleFlipped, [=](bool state) {
      Params().remove("CalibrationParams");
    });
  }

#ifdef ENABLE_MAPS
  toggles.append(new ParamControl("NavSettingTime24h",
                                  "Show ETA in 24h format",
                                  "Use 24h format instead of am/pm",
                                  "../assets/offroad/icon_metric.png",
                                  this));
#endif

  bool record_lock = Params().getBool("RecordFrontLock");
  record_toggle->setEnabled(!record_lock);

  for(ParamControl *toggle : toggles) {
    if(main_layout->count() != 0) {
      main_layout->addWidget(horizontal_line());
    }
    main_layout->addWidget(toggle);
  }
}

DevicePanel::DevicePanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  Params params = Params();

  QString dongle = QString::fromStdString(params.get("DongleId", false));
  main_layout->addWidget(new LabelControl("Dongle ID", dongle));
  main_layout->addWidget(horizontal_line());

  QString serial = QString::fromStdString(params.get("HardwareSerial", false));
  main_layout->addWidget(new LabelControl("Serial", serial));

  // offroad-only buttons

  auto dcamBtn = new ButtonControl("Driver Camera", "PREVIEW",
                                        "Preview the driver facing camera to help optimize device mounting position for best driver monitoring experience. (vehicle must be off)");
  connect(dcamBtn, &ButtonControl::released, [=]() { emit showDriverView(); });

  QString resetCalibDesc = "openpilot requires the device to be mounted within 4° left or right and within 5° up or down. openpilot is continuously calibrating, resetting is rarely required.";
  auto resetCalibBtn = new ButtonControl("Reset Calibration", "RESET", resetCalibDesc);
  connect(resetCalibBtn, &ButtonControl::released, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration?", this)) {
      Params().remove("CalibrationParams");
    }
  });
  connect(resetCalibBtn, &ButtonControl::showDescription, [=]() {
    QString desc = resetCalibDesc;
    std::string calib_bytes = Params().get("CalibrationParams");
    if (!calib_bytes.empty()) {
      try {
        AlignedBuffer aligned_buf;
        capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
        auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
        if (calib.getCalStatus() != 0) {
          double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
          double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
          desc += QString(" Your device is pointed %1° %2 and %3° %4.")
                                .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? "up" : "down",
                                     QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? "right" : "left");
        }
      } catch (kj::Exception) {
        qInfo() << "invalid CalibrationParams";
      }
    }
    resetCalibBtn->setDescription(desc);
  });

  auto uninstallBtn = new ButtonControl("Uninstall " + getBrand(), "UNINSTALL");
  connect(uninstallBtn, &ButtonControl::released, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to uninstall?", this)) {
      Params().putBool("DoUninstall", true);
    }
  });

  auto pm = new PubMaster({"kommuState"});

  auto tuneProfile1Btn = new ButtonControl("AggroLevel = -2", "SET");
  connect(tuneProfile1Btn, &ButtonControl::released, [=]() {
    MessageBuilder msg;
    auto ks = msg.initEvent().initKommuState();
    ks.setAggroLevel(-2);
    pm->send("kommuState", msg);
  });

  auto tuneProfile2Btn = new ButtonControl("AggroLevel = -1", "SET");
  connect(tuneProfile2Btn, &ButtonControl::released, [=]() {
    MessageBuilder msg;
    auto ks = msg.initEvent().initKommuState();
    ks.setAggroLevel(-1);
    pm->send("kommuState", msg);
  });

  auto tuneProfile3Btn = new ButtonControl("AggroLevel = 0", "SET");
  connect(tuneProfile3Btn, &ButtonControl::released, [=]() {
    MessageBuilder msg;
    auto ks = msg.initEvent().initKommuState();
    ks.setAggroLevel(0);
    pm->send("kommuState", msg);
  });

  auto tuneProfile4Btn = new ButtonControl("AggroLevel = 1", "SET");
  connect(tuneProfile4Btn, &ButtonControl::released, [=]() {
    MessageBuilder msg;
    auto ks = msg.initEvent().initKommuState();
    ks.setAggroLevel(1);
    pm->send("kommuState", msg);
  });

  auto tuneProfile5Btn = new ButtonControl("AggroLevel = 2", "SET");
  connect(tuneProfile5Btn, &ButtonControl::released, [=]() {
    MessageBuilder msg;
    auto ks = msg.initEvent().initKommuState();
    ks.setAggroLevel(2);
    pm->send("kommuState", msg);
  });

  for (auto btn : {dcamBtn, resetCalibBtn, uninstallBtn, tuneProfile1Btn, tuneProfile2Btn, tuneProfile3Btn, tuneProfile4Btn, tuneProfile5Btn}) {
    if (btn) {
      main_layout->addWidget(horizontal_line());
      if (btn != resetCalibBtn &&
          btn != tuneProfile1Btn &&
          btn != tuneProfile2Btn &&
          btn != tuneProfile3Btn &&
          btn != tuneProfile4Btn &&
          btn != tuneProfile5Btn) {
        connect(parent, SIGNAL(offroadTransition(bool)), btn, SLOT(setEnabled(bool)));
      }
      main_layout->addWidget(btn);
    }
  }

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *reboot_btn = new QPushButton("Reboot");
  reboot_btn->setStyleSheet("height: 120px;border-radius: 15px; background-color: #393939;");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reboot?", this)) {
      Hardware::reboot();
    }
  });

  QPushButton *poweroff_btn = new QPushButton("Power Off");
  poweroff_btn->setStyleSheet("height: 120px;border-radius: 15px; background-color: #E22C2C;");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to power off?", this)) {
      Hardware::poweroff();
    }
  });

  main_layout->addLayout(power_layout);
}

SoftwarePanel::SoftwarePanel(QWidget* parent) : QWidget(parent) {
  gitBranchLbl = new LabelControl("Git Branch");
  gitCommitLbl = new LabelControl("Git Commit");
  osVersionLbl = new LabelControl("OS Version");
  versionLbl = new LabelControl("Version", "", QString::fromStdString(params.get("ReleaseNotes")).trimmed());
  lastUpdateLbl = new LabelControl("Last Update Check", "", "The last time openpilot successfully checked for an update. The updater only runs while the car is off.");

  updateBtn = new ButtonControl("Check for Update", "");
  connect(updateBtn, &ButtonControl::released, [=]() {
    if (params.getBool("IsOffroad")) {
      const QString paramsPath = QString::fromStdString(params.getParamsPath());
      fs_watch->addPath(paramsPath + "/d/LastUpdateTime");
      fs_watch->addPath(paramsPath + "/d/UpdateFailedCount");
      updateBtn->setText("CHECKING");
      updateBtn->setEnabled(false);
    }
    std::system("pkill -1 -f selfdrive.updated");
  });

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  QWidget *widgets[] = {versionLbl, lastUpdateLbl, updateBtn, gitBranchLbl, gitCommitLbl, osVersionLbl};
  for (int i = 0; i < std::size(widgets); ++i) {
    main_layout->addWidget(widgets[i]);
    if (i < std::size(widgets) - 1) {
      main_layout->addWidget(horizontal_line());
    }
  }

  fs_watch = new QFileSystemWatcher(this);
  QObject::connect(fs_watch, &QFileSystemWatcher::fileChanged, [=](const QString path) {
    int update_failed_count = params.get<int>("UpdateFailedCount").value_or(0);
    if (path.contains("UpdateFailedCount") && update_failed_count > 0) {
      lastUpdateLbl->setText("failed to fetch update");
      updateBtn->setText("CHECK");
      updateBtn->setEnabled(true);
    } else if (path.contains("LastUpdateTime")) {
      updateLabels();
    }
  });
}

void SoftwarePanel::showEvent(QShowEvent *event) {
  updateLabels();
}

void SoftwarePanel::updateLabels() {
  QString lastUpdate = "";
  auto tm = params.get("LastUpdateTime");
  if (!tm.empty()) {
    lastUpdate = timeAgo(QDateTime::fromString(QString::fromStdString(tm + "Z"), Qt::ISODate));
  }

  versionLbl->setText(getBrandVersion());
  lastUpdateLbl->setText(lastUpdate);
  updateBtn->setText("CHECK");
  updateBtn->setEnabled(true);
  gitBranchLbl->setText(QString::fromStdString(params.get("GitBranch")));
  gitCommitLbl->setText(QString::fromStdString(params.get("GitCommit")).left(10));
  osVersionLbl->setText(QString::fromStdString(Hardware::get_os_version()).trimmed());
}

QWidget * network_panel(QWidget * parent) {
#ifdef QCOM
  QWidget *w = new QWidget(parent);
  QVBoxLayout *layout = new QVBoxLayout(w);
  layout->setSpacing(30);

  // wifi + tethering buttons
  auto wifiBtn = new ButtonControl("WiFi Settings", "OPEN");
  QObject::connect(wifiBtn, &ButtonControl::released, [=]() { HardwareEon::launch_wifi(); });
  layout->addWidget(wifiBtn);
  layout->addWidget(horizontal_line());

  auto tetheringBtn = new ButtonControl("Tethering Settings", "OPEN");
  QObject::connect(tetheringBtn, &ButtonControl::released, [=]() { HardwareEon::launch_tethering(); });
  layout->addWidget(tetheringBtn);
  layout->addWidget(horizontal_line());

  // SSH key management
  layout->addWidget(new SshToggle());
  layout->addWidget(horizontal_line());
  layout->addWidget(new SshControl());

  layout->addStretch(1);
#else
  Networking *w = new Networking(parent);
#endif
  return w;
}

void SettingsWindow::showEvent(QShowEvent *event) {
  panel_widget->setCurrentIndex(0);
  header_label->setText("Device");
  nav_btns->buttons()[0]->setChecked(true);
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 1px;
    background-color: #202020;
  )");

  //setup panel header
  header_label = new QLabel;
  header_label -> setStyleSheet(R"(
    border-radius: 0.1px;
    font-size: 70px;
    font-weight:500;
    padding-left: 40px;
    background-color: #303030;
    margin-bottom:-50px;
  )");

  // close button
  QPushButton *close_btn = new QPushButton("x");
  close_btn->setStyleSheet(R"(
    font-size: 90px;
    font-family: Calibri;
    border 0px black solid;
    border-radius: 75px;
    background-color: #000000;
  )");
  close_btn->setFixedSize(100, 100);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignTop);
  QObject::connect(close_btn, &QPushButton::released, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);

  QList<QPair<QString, QWidget *>> panels = {
    {"Device", device},
    {"Network", network_panel(this)},
    {"Toggles", new TogglesPanel(this)},
    {"Software", new SoftwarePanel(this)},
  };

#ifdef ENABLE_MAPS
  if (!Params().get("MapboxToken").empty()) {
    auto map_panel = new MapPanel(this);
    panels.push_back({"Navigation", map_panel});
    QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
  }
#endif
  const int padding = panels.size() > 3 ? 35 : 55;

  nav_btns = new QButtonGroup();
  for (auto &[name, panel] : panels) {

    QString img_dir = "../assets/kommu/";
    img_dir.append(name);
    img_dir.append(".png");

    QPixmap pixmap(img_dir);

    NavButton *btn = new NavButton(name,pixmap);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setFixedHeight(175);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 50px;
        font-weight: 500;
        padding-top: 50px;
        padding-left: 150px;
        padding-bottom: %1px;
      }
      QPushButton:checked {
        color: white;
      }
    )").arg(padding));

    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignLeft);

    panel->setContentsMargins(50, 25, 50, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::released, [=, w = panel_frame, n=name]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
      header_label->setText(n);
    });
  }
  sidebar_layout-> setSpacing(75);
  sidebar_layout->setContentsMargins(25, 25, 100, 100);

  // main settings layout, sidebar + main panel
  QGridLayout *main_layout = new QGridLayout(this);

  sidebar_widget->setFixedWidth(500);
  header_label->setFixedHeight(175);
  main_layout->addWidget(sidebar_widget,0,0,2,1);
  main_layout->addWidget(header_label,0,1);
  main_layout->addWidget(panel_widget,1,1);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
  )");
}

void SettingsWindow::hideEvent(QHideEvent *event) {
#ifdef QCOM
  HardwareEon::close_activities();
#endif
}
