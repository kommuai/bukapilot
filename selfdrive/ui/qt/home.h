#pragma once

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedLayout>
#include <QTimer>
#include <QWidget>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/offroad/driverview.h"
#include "selfdrive/ui/qt/onroad.h"
#include "selfdrive/ui/qt/sidebar.h"
#include "selfdrive/ui/qt/widgets/offroad_alerts.h"
#include "selfdrive/ui/ui.h"


QFrame *home_horizontal_line(QWidget *parent = nullptr);

class StatusLabel: public QWidget {

    Q_OBJECT
    public:
      explicit StatusLabel(const QString &text, const QPixmap &newIcon, QWidget* parent = 0);

      void setText(const QString &text);
      void setIcon(const QPixmap &icon);


    private:
      QString text;
      QPixmap icon;
      void paintEvent(QPaintEvent*) override;
};

class StatusWidget: public QWidget {

    Q_OBJECT
    public slots:
      void updateState(const UIState &s);

    public:
      explicit StatusWidget(QWidget* parent = 0);

    private:
        QVBoxLayout *status_layout;
        // QLabel *device_img;
        StatusLabel *device_txt;
        // QLabel *temp_img;
        StatusLabel *temp_txt;


};

class UpdatesWidget: public QWidget {
    public:
      explicit UpdatesWidget(QWidget* parent = 0);

    private:
        QVBoxLayout *update_layout;
        QPushButton *update_button;
};

class QrWidget: public QWidget {
    public:
      explicit QrWidget(QWidget* parent = 0);

    private:
        QHBoxLayout *qr_layout;
};

class DriveWidget: public QWidget {
    public:
      explicit DriveWidget(QWidget* parent = 0);
    private:
        QVBoxLayout *drive_layout;

};


class OffroadHome : public QFrame {
  Q_OBJECT

public:

  explicit OffroadHome(QWidget* parent = 0);
  StatusWidget* status;

private:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void refresh();

  QTimer* timer;
  QLabel* date;
  QStackedLayout* center_layout;

  UpdatesWidget* updates;
  QrWidget* qr;
  DriveWidget* drive;

};

class HomeWindow : public QWidget {
  Q_OBJECT


public:
  explicit HomeWindow(QWidget* parent = 0);

signals:
  void openSettings();
  void closeSettings();

  // forwarded signals
  void displayPowerChanged(bool on);
  void update(const UIState &s);
  void offroadTransitionSignal(bool offroad);

public slots:
  void offroadTransition(bool offroad);
  void showDriverView(bool show);
  void showSidebar(bool show);

protected:
  void mousePressEvent(QMouseEvent* e) override;

private:
  Sidebar *sidebar;
  OffroadHome *home;
  OnroadWindow *onroad;
  DriverViewWindow *driver_view;
  QStackedLayout *slayout;

};

