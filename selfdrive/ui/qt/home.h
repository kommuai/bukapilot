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
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/offroad_alerts.h"
#include "selfdrive/ui/ui.h"


QFrame *home_horizontal_line(QWidget *parent = nullptr);

class StatusLabel: public QWidget {

    Q_OBJECT
    public:
      explicit StatusLabel(const QString &text, const QString &icon_dir, QWidget* parent = 0);

      void setText(const QString &text);
      void setIconDir(const QString &icon_dir);

    private:
      QString text;
      QString icon_dir;
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

    Q_OBJECT
    public slots:
      void updateState(const UIState &s);

    public:
      explicit UpdatesWidget(QWidget* parent = 0);
      void mouseReleaseEvent(QMouseEvent* ev) override;

    private:
        QVBoxLayout *update_layout;
        QLabel *updates_header;
        Params params;
        int previousState;
};

class QrWidget: public QWidget {

    public:
      explicit QrWidget(QWidget* parent = 0);

    private:
        QHBoxLayout *qr_layout;
};

class DriveWidget: public QWidget {

    Q_OBJECT
    public slots:
      void updateState(const UIState &s);

    public:
      explicit DriveWidget(QWidget* parent = 0);

    private:
        QVBoxLayout *drive_layout;
        QLabel *rem_upl_val;
        QLabel *upl_spd_val;
};


class OffroadHome : public QFrame {
  Q_OBJECT

public:
  explicit OffroadHome(QWidget* parent = 0);

  StatusWidget* status;
  DriveWidget* drive;
  UpdatesWidget* updates;

private:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

  QTimer* timer;

  QrWidget* qr;

};

class HomeWindow : public QWidget {
  Q_OBJECT

public:
  explicit HomeWindow(QWidget* parent = 0);

signals:
  void openSettings();
  void closeSettings();
  void openTraining();
  void openTerms();

  // forwarded signals
  void displayPowerChanged(bool on);
  void update(const UIState &s);
  void offroadTransitionSignal(bool offroad);

public slots:
  void offroadTransition(bool offroad);
  void showDriverView(bool show);
  void showSidebar(bool show);

private:
  Sidebar *sidebar;
  OffroadHome *home;
  OnroadWindow *onroad;
  DriverViewWindow *driver_view;
  QStackedLayout *slayout;
};

class NotesPopup : public QDialogBase {
  Q_OBJECT

public:
  explicit NotesPopup(const QString &prompt_text, QWidget *parent);

public slots:
  int exec();
};

