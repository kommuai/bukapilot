#include "selfdrive/ui/qt/widgets/nav_buttons.h"
#include <QPainter>

NavButton::NavButton( const QString &text, const QPixmap &newIcon, QWidget *parent) : QPushButton(text,parent){
    update();
    int size = 40;
    icon = newIcon.scaled(size,size,Qt::KeepAspectRatio);

}


void NavButton::paintEvent(QPaintEvent *e) {
  QPushButton::paintEvent(e);
  QPainter p(this);
  p.drawPixmap(50,(height()-40/2)/2, icon);

}

