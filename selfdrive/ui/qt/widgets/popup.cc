#include "selfdrive/ui/qt/widgets/popup.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/hardware/hw.h"

Popup::Popup(const QString &title_text, const QString &content_text, int buttons, QWidget *parent) : QDialog(parent) {
  auto main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(45, 35, 45, 45);
  main_layout->setSpacing(0);

  auto title = new QLabel(title_text);
  title->setStyleSheet("font-size: 90px; font-weight: 600;");
  main_layout->addWidget(title);

  main_layout->addSpacing(30);

  content = new QLabel(content_text);
  content->setAlignment(Qt::AlignLeft);
  content->setWordWrap(true);
  content->setStyleSheet("font-size: 50px; background-color: #1b1b1b; color: white;");
  content->setMargin(35);
  auto scroll = new ScrollView(content);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  main_layout->addWidget(scroll);

  main_layout->addSpacing(30);

  auto btn_layout = new QHBoxLayout();
  main_layout->addLayout(btn_layout);

  if (RESET & buttons) {
    auto rbt_btn = new QPushButton("Reboot");
    rbt_btn->setStyleSheet("background-color: #B73D3D");
    connect(rbt_btn, &QPushButton::released, [=]() { Hardware::reboot(); });
    btn_layout->setSpacing(35);
    btn_layout->addWidget(rbt_btn);
  }

  if (OK & buttons) {
    auto close_btn = new QPushButton("Close");
    connect(close_btn, &QPushButton::released, this, &Popup::accept);
    btn_layout->addWidget(close_btn);
  }

  setStyleSheet(R"(
    * {
    color: white;
    background-color: black;
   }
   QPushButton {
     height: 120px;
     font-size: 55px;
     font-weight: 400;
     border-radius: 10px;
     background-color: #00FA9A;
     color: black;
   }
  )");
}

int Popup::exec() {
  show();
  return QDialog::exec();
}

void Popup::show() {
  setMainWindow(this);
}
