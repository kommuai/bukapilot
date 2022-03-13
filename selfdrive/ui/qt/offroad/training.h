#pragma once

#include <QFrame>
#include <QImage>
#include <QMouseEvent>
#include <QPushButton>
#include <QWidget>

#include "selfdrive/ui/qt/qt_window.h"

class TrainingWindow : public QFrame {
  Q_OBJECT

public:
  explicit TrainingWindow(QWidget *parent = 0) : QFrame(parent) {};

private:
  void showEvent(QShowEvent *event) override;
  void paintEvent(QPaintEvent *event) override;
  void mouseReleaseEvent(QMouseEvent* e) override;

  QImage image;
  int currentIndex = 0;

  // Bounding boxes for each training guide step
  const QRect continueBtnStandard = {1610, 0, 310, 1080};
  QVector<QRect> boundingRectStandard {
    QRect(650, 710, 720, 190),
    continueBtnStandard,
    continueBtnStandard,
    QRect(1470, 515, 235, 565),
    QRect(1580, 630, 215, 130),
    continueBtnStandard,
    QRect(1580, 630, 215, 130),
    QRect(1210, 0, 485, 590),
    QRect(1460, 400, 375, 210),
    QRect(1460, 210, 300, 310),
    continueBtnStandard,
    QRect(1375, 80, 545, 1000),
    continueBtnStandard,
    QRect(1610, 130, 280, 800),
    QRect(1385, 485, 400, 270),
    continueBtnStandard,
    continueBtnStandard,
    QRect(1036, 769, 718, 189),
    QRect(201, 769, 718, 189),
  };

  const QRect continueBtnWide = {1850, 0, 310, 1080};
  QVector<QRect> boundingRectWide {
    QRect(654, 721, 718, 189),
    continueBtnWide,
    continueBtnWide,
    QRect(1589, 530, 345, 555),
    QRect(1660, 630, 195, 125),
    continueBtnWide,
    QRect(1820, 630, 180, 155),
    QRect(1360, 0, 460, 620),
    QRect(1570, 400, 375, 215),
    QRect(1610, 210, 295, 310),
    continueBtnWide,
    QRect(1555, 90, 610, 990),
    continueBtnWide,
    QRect(1600, 140, 280, 790),
    QRect(1385, 490, 750, 270),
    continueBtnWide,
    continueBtnWide,
    QRect(1138, 755, 718, 189),
    QRect(303, 755, 718, 189),
  };

  const QString IMG_PATH = vwp_w == 2160 ? "../assets/training_wide/" : "../assets/training/";
  const QVector<QRect> boundingRect = vwp_w == 2160 ? boundingRectWide : boundingRectStandard;

signals:
  void closeTraining();
};