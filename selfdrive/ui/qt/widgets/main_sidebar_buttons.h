#ifndef MAIN_SIDEBAR_BUTTONS_H
#define MAIN_SIDEBAR_BUTTONS_H


#include <QPushButton>
#include <QPixmap>
#include <QWidget>

#include <QPainter>
#include <QFrame>

class MainSidebarButton : public QPushButton
{

Q_OBJECT

public:
    MainSidebarButton( const QString &text, const QPixmap &newIcon, QWidget *parent = nullptr );

protected:
    QPixmap icon;
    QString button_text;
    void paintEvent(QPaintEvent*) override;

};


#endif // MAIN_SIDEBAR_BUTTONS_H
