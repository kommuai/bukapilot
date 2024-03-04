#pragma once

#include <QFrame>
#include <QLabel>

#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/ui.h"

class Sidebar : public QFrame {
  Q_OBJECT

public:
  explicit Sidebar(QWidget* parent = 0);
  void setAlertCount(int count);

signals:
  void openAlerts();
  void openSettings();
  void openTerms();
  void openTraining();

private:
  QPixmap alert_empty_icon, alert_unread_icon;
  QLabel *alert_icon;
};

class SidebarItem : public ClickableWidget {
  Q_OBJECT

public:
  SidebarItem(const QString& label, const QString &iconPath, QWidget *parent);

  QLabel *icon;
  QLabel *text;
};
