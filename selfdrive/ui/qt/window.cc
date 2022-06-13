#include "selfdrive/ui/qt/window.h"

#include <QFontDatabase>

#include "selfdrive/common/params.h"
#include "selfdrive/hardware/hw.h"

MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
  main_layout = new QStackedLayout(this);
  main_layout->setMargin(0);

  homeWindow = new HomeWindow(this);
  main_layout->addWidget(homeWindow);

  onboardingWindow = new OnboardingWindow(this);
  main_layout->addWidget(onboardingWindow);

  QObject::connect(homeWindow, &HomeWindow::openTraining, [=]() {
    Params().put("CompletedTrainingVersion", "b'0'");
    main_layout->setCurrentWidget(onboardingWindow);
  });
  QObject::connect(homeWindow, &HomeWindow::openTerms, [=]() {
    Params().put("HasAcceptedTerms", "b'0'");
    main_layout->setCurrentWidget(onboardingWindow);
  });
  QObject::connect(onboardingWindow, &OnboardingWindow::onboardingDone, [=]() {
    main_layout->setCurrentWidget(homeWindow);
  });

  QObject::connect(homeWindow, &HomeWindow::openSettings, this, &MainWindow::openSettings);
  QObject::connect(homeWindow, &HomeWindow::closeSettings, this, &MainWindow::closeSettings);
  QObject::connect(&qs, &QUIState::uiUpdate, homeWindow, &HomeWindow::update);
  QObject::connect(&qs, &QUIState::offroadTransition, homeWindow, &HomeWindow::offroadTransition);
  QObject::connect(&qs, &QUIState::offroadTransition, homeWindow, &HomeWindow::offroadTransitionSignal);
  QObject::connect(&device, &Device::displayPowerChanged, homeWindow, &HomeWindow::displayPowerChanged);

  settingsWindow = new SettingsWindow(this);
  main_layout->addWidget(settingsWindow);
  QObject::connect(settingsWindow, &SettingsWindow::closeSettings, this, &MainWindow::closeSettings);
  QObject::connect(&qs, &QUIState::offroadTransition, settingsWindow, &SettingsWindow::offroadTransition);
  QObject::connect(settingsWindow, &SettingsWindow::showDriverView, [=] {
    homeWindow->showDriverView(true);
  });
  QObject::connect(settingsWindow, &SettingsWindow::keepAwakeChanged, &device, &Device::setKeepAwake);

  bool should_onboard = Params().get("HasAcceptedTerms") != Params().get("TermsVersion") &&
    Params().get("CompletedTrainingVersion") != Params().get("TrainingVersion");
  if (should_onboard) {
    main_layout->setCurrentWidget(onboardingWindow);
  }

  device.setAwake(true, true);
  QObject::connect(&qs, &QUIState::uiUpdate, &device, &Device::update);
  QObject::connect(&qs, &QUIState::offroadTransition, [=](bool offroad) {
    if (!offroad) {
      closeSettings();
    }
  });
  QObject::connect(&device, &Device::displayPowerChanged, [=]() {
    if(main_layout->currentWidget() != onboardingWindow) {
      closeSettings();
    }
  });

  // load fonts
  QFontDatabase::addApplicationFont("../assets/fonts/GlacialIndifference-Regular.otf");
  QFontDatabase::addApplicationFont("../assets/fonts/GlacialIndifference-Bold.otf");

  // no outline to prevent the focus rectangle
  setStyleSheet(R"(
    * {
      font-family: Glacial Indifference;
      outline: none;
    }
  )");
}

void MainWindow::openSettings() {
  main_layout->setCurrentWidget(settingsWindow);
}

void MainWindow::closeSettings() {
  main_layout->setCurrentWidget(homeWindow);

  if (QUIState::ui_state.scene.started) {
    emit homeWindow->showSidebar(false);
  }
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
  // wake screen on tap
  if (event->type() == QEvent::MouseButtonPress) {
    device.setAwake(true, true);
  }

#ifdef QCOM
  // filter out touches while in android activity
  const static QSet<QEvent::Type> filter_events({QEvent::MouseButtonPress, QEvent::MouseMove, QEvent::TouchBegin, QEvent::TouchUpdate, QEvent::TouchEnd});
  if (HardwareEon::launched_activity && filter_events.contains(event->type())) {
    HardwareEon::check_activity();
    if (HardwareEon::launched_activity) {
      return true;
    }
  }
#endif
  return false;
}
