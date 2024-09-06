#include "selfdrive/ui/qt/window.h"

#include <QFontDatabase>

#include "selfdrive/hardware/hw.h"

MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
  main_layout = new QStackedLayout(this);
  main_layout->setMargin(0);

  homeWindow = new HomeWindow(this);
  main_layout->addWidget(homeWindow);
  QObject::connect(homeWindow, &HomeWindow::openSettings, this, &MainWindow::openSettings);
  QObject::connect(homeWindow, &HomeWindow::closeSettings, this, &MainWindow::closeSettings);
  connect(homeWindow, &HomeWindow::openTerms, [=] {
    onboardingWindow->showTerms();
    main_layout->setCurrentWidget(onboardingWindow);
  });

  connect(homeWindow, &HomeWindow::openTraining, [=] {
    onboardingWindow->showTrainingGuide();
    main_layout->setCurrentWidget(onboardingWindow);
  });

  settingsWindow = new SettingsWindow(this);
  main_layout->addWidget(settingsWindow);
  QObject::connect(settingsWindow, &SettingsWindow::closeSettings, this, &MainWindow::closeSettings);
  QObject::connect(settingsWindow, &SettingsWindow::showDriverView, [=] {
    homeWindow->showDriverView(true);
  });

  onboardingWindow = new OnboardingWindow(this);
  main_layout->addWidget(onboardingWindow);
  QObject::connect(onboardingWindow, &OnboardingWindow::onboardingDone, [=]() {
    main_layout->setCurrentWidget(homeWindow);
  });
  if (!onboardingWindow->completed()) {
    main_layout->setCurrentWidget(onboardingWindow);
  }

  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    if (!offroad) {
      closeSettings();
    }
  });
  QObject::connect(&device, &Device::interactiveTimout, [=]() {
    if (main_layout->currentWidget() == settingsWindow) {
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
  setAttribute(Qt::WA_NoSystemBackground);
}

void MainWindow::openSettings() {
  main_layout->setCurrentWidget(settingsWindow);
}

void MainWindow::closeSettings() {
  main_layout->setCurrentWidget(homeWindow);

  if (uiState()->scene.started) {
    homeWindow->showSidebar(false);
  }
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
  const static QSet<QEvent::Type> evts({QEvent::MouseButtonPress, QEvent::MouseMove,
                                 QEvent::TouchBegin, QEvent::TouchUpdate, QEvent::TouchEnd});

  if (evts.contains(event->type())) {
    device.resetInteractiveTimout();
#ifdef QCOM
    // filter out touches while in android activity
    if (HardwareEon::launched_activity) {
      HardwareEon::check_activity();
      if (HardwareEon::launched_activity) {
        return true;
      }
    }
#endif
  }
  return false;
}
