#pragma once

#include <QFrame>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QStackedLayout>
#include <QTextOption>
#include <QTimer>
#include <QWidget>

#include <unordered_map>
#include <vector>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/offroad/driverview.h"
#include "selfdrive/ui/qt/onroad.h"
#include "selfdrive/ui/qt/k_sidebar.h"
#include "selfdrive/ui/qt/widgets/k_alerts.h"
#include "selfdrive/ui/ui.h"

class StatusLabel : public QWidget {

Q_OBJECT

public:
  QPixmap icon;
  QString text;

  StatusLabel(const QString& text, QPixmap icon, QWidget *parent);

private:
  QFont font;
  QTextOption opt = QTextOption(Qt::AlignCenter);

  void paintEvent(QPaintEvent *e) override;
};

class AlertsManager; // fwd: k_alerts.h

class QrWidget : public QWidget {

Q_OBJECT

public:
    QrWidget(const char *content, QWidget *parent);
    void paintEvent(QPaintEvent *e) override;
    void setContent(const char *content);

private:
    QPixmap img;
};

class OffroadHome : public QFrame {
  Q_OBJECT

public:
  bool hasSevereAlerts;
  explicit OffroadHome(QWidget* parent = 0);

public slots:
  void updateState(const UIState& s);

private:
  Params params;
  int update_state = -1; // tri-state of -1 (not inited), 0 (no update) and 1 (available)
  StatusLabel *device_text;
  StatusLabel *temperature_text;
  QLabel *remaining_upload;
  QLabel *updates_text;
  QLabel *upload_speed;
  std::unordered_map<cereal::DeviceState::ThermalStatus, QPixmap> status_icons = {
    {cereal::DeviceState::ThermalStatus::GREEN, QPixmap("../assets/kommu/green_circle.png")},
    {cereal::DeviceState::ThermalStatus::YELLOW, QPixmap("../assets/kommu/yellow_circle.png")},
    {cereal::DeviceState::ThermalStatus::RED, QPixmap("../assets/kommu/red_circle.png")},
    {cereal::DeviceState::ThermalStatus::DANGER, QPixmap("../assets/kommu/red_circle.png")},
  };
};

class HomeWindow : public QWidget {
  Q_OBJECT

public:
  explicit HomeWindow(QWidget* parent = 0);

signals:
  void openSettings();
  void openTerms();
  void openTraining();
  void closeSettings();

public slots:
  void offroadTransition(bool offroad);
  void showDriverView(bool show);
  void showSidebar(bool show);

protected:
  void mousePressEvent(QMouseEvent* e) override;

private:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void refresh();

  AlertsManager alerts;
  Sidebar *sidebar;
  OffroadHome *home;
  OnroadWindow *onroad;
  DriverViewWindow *driver_view;
  QStackedLayout *slayout;
  QTimer *timer;
};
