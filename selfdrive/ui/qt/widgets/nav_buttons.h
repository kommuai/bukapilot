#ifndef NAVBUTTON_HEADER
#define NAVBUTTON_HEADER

#include <QPushButton>
#include <QPixmap>
#include <QWidget>

#include <QPainter>
#include <QFrame>

class NavButton : public QPushButton
{

Q_OBJECT

public:
    NavButton( const QString &text, const QPixmap &newIcon, QWidget *parent = nullptr );

protected:
    QPixmap icon;
    void paintEvent(QPaintEvent*) override;

};

#endif
