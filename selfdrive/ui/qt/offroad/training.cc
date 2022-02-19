#include "selfdrive/ui/qt/offroad/training.h"


#include <QFrame>
#include <QLabel>
#include <QPainter>
#include <QQmlContext>
#include <QQuickWidget>
#include <QVBoxLayout>

#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/widgets/input.h"

void TrainingWindow::mouseReleaseEvent(QMouseEvent *e) {
  if (boundingRect[currentIndex].contains(e->x(), e->y())) {
    currentIndex += 1;
  } else if (currentIndex == (boundingRect.size() - 2) && boundingRect.last().contains(e->x(), e->y())) {
    currentIndex = 0;
  }

  if (currentIndex >= (boundingRect.size() - 1)) {
    emit closeTraining();
  } else {
    image.load(IMG_PATH + "step" + QString::number(currentIndex) + ".png");
    update();
  }
}

void TrainingWindow::showEvent(QShowEvent *event) {
  currentIndex = 0;
  image.load(IMG_PATH + "step0.png");
}

void TrainingWindow::paintEvent(QPaintEvent *event) {
  QPainter painter(this);

  QRect bg(0, 0, painter.device()->width(), painter.device()->height());
  QBrush bgBrush("#000000");
  painter.fillRect(bg, bgBrush);

  QRect rect(image.rect());
  rect.moveCenter(bg.center());
  painter.drawImage(rect.topLeft(), image);
}
