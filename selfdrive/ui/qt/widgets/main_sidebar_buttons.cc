#include "widgets/main_sidebar_buttons.h"
#include <QPainter>

MainSidebarButton::MainSidebarButton( const QString &text, const QPixmap &newIcon, QWidget *parent) : QPushButton(text,parent){
    update();
    int size = 65;
    icon = newIcon.scaled(size,size,Qt::KeepAspectRatio);
    button_text = text;

}


void MainSidebarButton::paintEvent(QPaintEvent *e) {
  this->setText("");

  QPushButton::paintEvent(e);
  QPainter p(this);
  p.drawPixmap(width()/2-icon.width()/2,height()/2-icon.height(), icon);
  QFont font=p.font() ;
  int font_size = 40;
  font.setPixelSize(font_size);
  p.setFont(font);
  font.setWeight(QFont::DemiBold);
//  p.drawText(QPoint(width()/2,(height()/2)),button_text);
 // p.drawText(QRect(width()/2 - button_text.size()*font_size/2.75,height()/2+icon.height()/1.5),Qt::AlignCenter,button_text);
  p.drawText(QRect(0,height()/2,width(),height()-(height()/2+icon.height()/1.5)),Qt::AlignCenter ,button_text);

}

