#include "selfdrive/ui/qt/sidebar.h"

#include <QMouseEvent>
#include <QPushButton>
#include <QPixmap>
#include <QLabel>

#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/util.h"

#include "selfdrive/ui/qt/widgets/main_sidebar_buttons.h"


void Sidebar::drawMetric(QPainter &p, const QString &label, const QString &val, QColor c, int y) {
  const QRect rect = {30, y, 240, val.isEmpty() ? (label.contains("\n") ? 124 : 100) : 148};

  p.setPen(Qt::NoPen);
  p.setBrush(QBrush(c));
  p.setClipRect(rect.x() + 6, rect.y(), 18, rect.height(), Qt::ClipOperation::ReplaceClip);
  p.drawRoundedRect(QRect(rect.x() + 6, rect.y() + 6, 100, rect.height() - 12), 10, 10);
  p.setClipping(false);

  QPen pen = QPen(QColor(0xff, 0xff, 0xff, 0x55));
  pen.setWidth(2);
  p.setPen(pen);
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(rect, 20, 20);

  p.setPen(QColor(0xff, 0xff, 0xff));
  if (val.isEmpty()) {
    configFont(p, "Open Sans", 35, "Bold");
    const QRect r = QRect(rect.x() + 30, rect.y(), rect.width() - 40, rect.height());
    p.drawText(r, Qt::AlignCenter, label);
  } else {
    configFont(p, "Open Sans", 58, "Bold");
    p.drawText(rect.x() + 50, rect.y() + 71, val);
    configFont(p, "Open Sans", 35, "Regular");
    p.drawText(rect.x() + 50, rect.y() + 50 + 77, label);
  }
}

Sidebar::Sidebar(QWidget *parent) : QFrame(parent) {

  QVBoxLayout *new_sidebar_layout = new QVBoxLayout(this);
  QPixmap logo_pix("../assets/kommu/Logo.png");
  QLabel *logo_lbl = new QLabel();

  logo_lbl->resize(200,200);
  logo_pix = logo_pix.scaled(logo_lbl->size(),Qt::KeepAspectRatio);
  logo_lbl->setPixmap(logo_pix);
  new_sidebar_layout->addWidget(logo_lbl);
  new_sidebar_layout->setAlignment(logo_lbl,Qt::AlignHCenter);
  QList<QString> btns = {
    "Tutorial",
    "T&C",
    "Settings",
  };

  sidebar_btns = new QButtonGroup();

  for (QString &btn_name:btns) {
      QString img_dir = "../assets/kommu/";
      img_dir.append(btn_name);
      img_dir.append(".png");
      const int padding = 75;
      QPixmap pixmap(img_dir);
      MainSidebarButton *main_sidebar_btn = new MainSidebarButton(btn_name,pixmap);
      main_sidebar_btn->setCheckable(false);
      main_sidebar_btn->setChecked(sidebar_btns->buttons().size() == 0);
      main_sidebar_btn->setFixedHeight(250);
      main_sidebar_btn->setStyleSheet(QString(R"(
        QPushButton {
          color: white;
          border: none;
          background: black;
          font-size: 35px;
          font-weight: 100;
          padding-top: 50px;
          padding-left: 150px;
          padding-bottom: %1px;
        }
        QPushButton:checked {
          background-color:rgb(30,30,30) ;
        }
      )").arg(padding));


    sidebar_btns->addButton(main_sidebar_btn);
    connect(sidebar_btns->button(-2),  &MainSidebarButton::released, [=]() {
        emit openTraining();
    });
    connect(sidebar_btns->button(-3),  &MainSidebarButton::released, [=]() {
        emit openTerms();
    });
    connect(sidebar_btns->button(-4),  &MainSidebarButton::released, [=]() {
        emit openSettings();
    });
    new_sidebar_layout->addWidget(main_sidebar_btn);
    new_sidebar_layout->setAlignment(main_sidebar_btn,Qt::AlignHCenter);

  }

  connect(this, &Sidebar::valueChanged, [=] { update(); });

  setFixedWidth(250);
  setMinimumHeight(vwp_h-225*3);
  setStyleSheet("background-color: rgb(0, 0, 0);");
}

void Sidebar::mousePressEvent(QMouseEvent *event) {
  if (settings_btn.contains(event->pos())) {
    emit openSettings();
  }
}

void Sidebar::updateState(const UIState &s) {

}

