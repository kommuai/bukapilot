#include <cstdlib>
#include <fstream>
#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QScrollBar>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

int main(int argc, char *argv[]) {
  initApp();
  QApplication a(argc, argv);
  QWidget window;
  setMainWindow(&window);

  QGridLayout *main_layout = new QGridLayout(&window);
  main_layout->setMargin(50);

  std::string mainLabel = "Sorry, we fucked up.  Please contact support and press \"Click Me!\" when instructed.\n\n";

  QLabel *label = new QLabel((mainLabel + argv[1]).c_str());
  label->setWordWrap(true);
  label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  ScrollView *scroll = new ScrollView(label);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  main_layout->addWidget(scroll, 0, 0, Qt::AlignTop);

  QHBoxLayout *btns_bar = new QHBoxLayout();

  QPushButton *btn3 = new QPushButton();
  btn3->setText("Click Me!");
  QObject::connect(btn3, &QPushButton::released, [=]() {
      std::system("git fetch");

      std::ofstream report;
      report.open("_report", std::ofstream::out | std::ofstream::app);
      report << argv[1] << std::endl;
      report << "\n-------------------------------------\n" << std::endl;
      report << exec("git status") << std::endl;
      report.close();

      label->setText(QString(exec("cat _report | nc termbin.com 9999").c_str()));
      std::system("rm _report");
  });
  btns_bar->addWidget(btn3);

  QPushButton *btn2 = new QPushButton();
  btn2->setText("GitReset");
  QObject::connect(btn2, &QPushButton::released, [=]() {
    std::system("git fetch && git reset --hard @{u}");
    std::system("git clean -fdx");
    label->setText(QString("You may try to reboot now"));
  });
  btns_bar->addWidget(btn2);

  QPushButton *btn = new QPushButton();
#ifdef __aarch64__
  btn->setText("Reboot");
  QObject::connect(btn, &QPushButton::released, [=]() {
    Hardware::reboot();
  });
#else
  btn->setText("Exit");
  QObject::connect(btn, &QPushButton::released, &a, &QApplication::quit);
#endif
  btns_bar->addWidget(btn);

  main_layout->addLayout(btns_bar, 0, 0, Qt::AlignRight | Qt::AlignBottom);
  window.setStyleSheet(R"(
    * {
      outline: none;
      color: white;
      background-color: black;
      font-size: 60px;
    }
    QPushButton {
      padding: 50px;
      padding-right: 100px;
      padding-left: 100px;
      border: 2px solid white;
      border-radius: 20px;
      margin-right: 40px;
    }
  )");

  return a.exec();
}
