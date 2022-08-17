#pragma once

#include <QButtonGroup>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>

#include "selfdrive/common/features.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"

// ********** settings window + top-level panels **********
class SettingsWindow : public QFrame {
  Q_OBJECT

public:
  explicit SettingsWindow(QWidget *parent = 0);

protected:
  void hideEvent(QHideEvent *event) override;
  void showEvent(QShowEvent *event) override;

signals:
  void closeSettings();
  void showDriverView();

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;
};

class DevicePanel : public ListWidget {
  Q_OBJECT
public:
  explicit DevicePanel(SettingsWindow *parent);
signals:
  void showDriverView();

private slots:
  void poweroff();
  void reboot();
  void updateCalibDescription();

private:
  ButtonControl *resetCalibBtn;
  ButtonControl *serialBtn;
  ButtonControl *testBtn;

  int dev_tab_counter = 0;
  Params params;
};

class TogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TogglesPanel(SettingsWindow *parent);
};

class SoftwarePanel : public ListWidget {
  Q_OBJECT
public:
  explicit SoftwarePanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  void updateLabels();

  LabelControl *gitCommitLbl;
  LabelControl *osVersionLbl;
  LabelControl *versionLbl;
  LabelControl *lastUpdateLbl;
  ButtonControl *updateBtn;

  Params params;
  QFileSystemWatcher *fs_watch;
};

class C2NetworkPanel: public ListWidget {
  Q_OBJECT
public:
  explicit C2NetworkPanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  QString getIPAddress();
  LabelControl *ipaddress;
};

class FeaturesControl : public ButtonControl {
  Q_OBJECT

public:
  FeaturesControl() : ButtonControl("Features Package", "SET", "Warning: Only use under guidance of a support staff.") {
    package_label = new QLabel();
    package_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    package_label->setStyleSheet("color: #aaaaaa");
    package_label->setText(Params().get("FeaturesPackage").c_str());
    hlayout->insertWidget(1, package_label);
    connect(this, &ButtonControl::clicked, [=] {
      QString package = InputDialog::getText("Enter feature package name", this);
      if (package.length() > 0) {
        Features().set_package(package.toStdString());
        package_label->setText(Params().get("FeaturesPackage").c_str());
      }
    });
  }

private:
  QLabel *package_label;
};
