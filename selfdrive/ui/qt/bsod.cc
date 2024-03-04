/*
 * Amazing font [Mx437_DOS-V_TWN19.ttf] by VileR
 *   from https://int10h.org/oldschool-pc-fonts/
 */

#include <cstdlib>
#include <fstream>

#include <QApplication>
#include <QFontDatabase>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QString>
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

  auto fontFam = QFontDatabase::applicationFontFamilies(
        QFontDatabase::addApplicationFont("/data/openpilot/selfdrive/assets/kommu/Mx437_DOS-V_TWN19.ttf")
      ).at(0);
  a.setFont(QFont(fontFam));

  auto main_layout = new QGridLayout(&window);
  main_layout->setMargin(50);

  std::string mainLabel = "Sorry, we fucked up.  Please contact support and press \"Click Me!\" when instructed.\n\n";
  auto label = new QLabel((mainLabel + argv[1]).c_str());
  label->setWordWrap(true);
  label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  auto scroll = new ScrollView(label);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  main_layout->addWidget(scroll, 0, 0, Qt::AlignTop);

  auto btns_bar = new QHBoxLayout();
  main_layout->addLayout(btns_bar, 0, 0, Qt::AlignRight | Qt::AlignBottom);

  auto btn_gen_report = new QPushButton();
  btn_gen_report->setText("Click Me!");
  QObject::connect(btn_gen_report, &QPushButton::released, [=]() {
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
  btns_bar->addWidget(btn_gen_report);

  auto btn_git_reset = new QPushButton();
  btn_git_reset->setText("GitReset");
  QObject::connect(btn_git_reset, &QPushButton::released, [=]() {
    std::system("git fetch && git reset --hard @{u}");
    std::system("git clean -fdx");
    label->setText(QString("You may try to reboot now"));
  });
  btns_bar->addWidget(btn_git_reset);

  auto btn_reboot = new QPushButton();
  btn_reboot->setText("Reboot");
  QObject::connect(btn_reboot, &QPushButton::released, [=] {
    Hardware::reboot();
  });
  btns_bar->addWidget(btn_reboot);

  window.setStyleSheet(R"(
    * {
      outline: none;
      color: white;
      background-color: #0000aa;
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
