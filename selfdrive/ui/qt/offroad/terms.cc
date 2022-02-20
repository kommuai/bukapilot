#include "selfdrive/ui/qt/offroad/terms.h"

#include <QLabel>
#include <QPainter>
#include <QQmlContext>
#include <QQuickWidget>
#include <QVBoxLayout>

#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/input.h"

void TermsWindow::showEvent(QShowEvent *event) {
  // late init, building QML widget takes 200ms
  if (layout()) {
    return;
  }

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(45, 35, 45, 45);
  main_layout->setSpacing(0);

  QLabel *title = new QLabel("Terms & Conditions");
  title->setStyleSheet("color: white; font-size: 90px; font-weight: 600;");
  main_layout->addWidget(title);
  this ->setStyleSheet("background-color:black");
  main_layout->addSpacing(30);

  QQuickWidget *text = new QQuickWidget(this);
  text->setResizeMode(QQuickWidget::SizeRootObjectToView);
  text->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  text->setAttribute(Qt::WA_AlwaysStackOnTop);
  text->setClearColor(QColor("#1B1B1B"));

  QString text_view = util::read_file("../assets/offroad/tc.html").c_str();
  text->rootContext()->setContextProperty("text_view", text_view);

  text->setSource(QUrl::fromLocalFile("qt/offroad/text_view.qml"));

  main_layout->addWidget(text, 1);
  main_layout->addSpacing(50);

  QObject *obj = (QObject*)text->rootObject();
  QObject::connect(obj, SIGNAL(scroll()), SLOT(enableAccept()));

  QHBoxLayout* buttons = new QHBoxLayout;
  buttons->setMargin(0);
  buttons->setSpacing(45);
  main_layout->addLayout(buttons);

  accept_btn = new QPushButton("Scroll to Proceed");
  accept_btn->setEnabled(false);
  accept_btn->setStyleSheet(R"(
    QPushButton {
      border-radius: 5px;
      background-color: #00FA9A;
    }
    QPushButton:disabled {
      background-color: #4F4F4F;
    }
  )");
  buttons->addWidget(accept_btn);
  QObject::connect(accept_btn, &QPushButton::released, this, &TermsWindow::closeTerms);
}

void TermsWindow::enableAccept() {
  accept_btn->setText("OK");
  accept_btn->setEnabled(true);
}
