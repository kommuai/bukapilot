#include "selfdrive/ui/qt/k_sidebar.h"

#include <QVBoxLayout>
#include <QWidget>

#include "selfdrive/ui/qt/util.h"

Sidebar::Sidebar(QWidget *parent) : QFrame(parent) {
  auto layout = new QVBoxLayout(this);
  layout->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

  auto logo = new QLabel();
  logo->setPixmap(loadPixmap("../assets/kommu/logo.png", {200, 200}));
  logo->setAlignment(Qt::AlignHCenter);
  layout->addWidget(logo);

  alert_empty_icon = loadPixmap("../assets/kommu/alerts.png", {64, 64});
  alert_unread_icon = loadPixmap("../assets/kommu/alerts_red.png", {64, 64});
  auto alerts = new SidebarItem("Alerts", "../assets/kommu/alerts.png", this);
  alert_icon = alerts->icon;
  connect(alerts, &SidebarItem::clicked, this, &Sidebar::openAlerts);
  layout->addWidget(alerts);

  auto training = new SidebarItem("Tutorial", "../assets/kommu/training.png", this);
  connect(training, &SidebarItem::clicked, this, &Sidebar::openTraining);
  layout->addWidget(training);

  auto terms = new SidebarItem("T&C", "../assets/kommu/terms.png", this);
  connect(terms, &SidebarItem::clicked, this, &Sidebar::openTerms);
  layout->addWidget(terms);

  auto settings = new SidebarItem("Settings", "../assets/kommu/settings.png", this);
  connect(settings, &SidebarItem::clicked, this, &Sidebar::openSettings);
  layout->addWidget(settings);

  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
  setFixedWidth(250);
  setStyleSheet("color: white; font-size: 30px;");
}

void Sidebar::setAlertCount(int count) {
  alert_icon->setPixmap((count > 0) ? alert_unread_icon : alert_empty_icon);
}

SidebarItem::SidebarItem(const QString& label, const QString &iconPath, QWidget *parent) : ClickableWidget(parent) {
  auto layout = new QVBoxLayout(this);
  layout->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

  icon = new QLabel();
  icon->setPixmap(loadPixmap(iconPath, {64, 64}));
  icon->setAlignment(Qt::AlignHCenter);
  layout->addWidget(icon);

  text = new QLabel(label);
  layout->addWidget(text);
}

