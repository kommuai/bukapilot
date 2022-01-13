#include "selfdrive/ui/qt/widgets/nav_buttons.h"
#include <QPainter>

NavButton::NavButton( const QString &text, const QPixmap &newIcon, QWidget *parent) : QPushButton(text,parent){
    update();
    int size = 60;
    icon = newIcon.scaled(size,size,Qt::KeepAspectRatio);

}


void NavButton::paintEvent(QPaintEvent *e) {
  QPushButton::paintEvent(e);
  QPainter p(this);
  p.drawPixmap(25,(height()-60/2)/2, icon);

}

