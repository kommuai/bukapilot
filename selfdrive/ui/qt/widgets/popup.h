#pragma once

#include <QDialog>
#include <QLabel>

class Popup : public QDialog {
  Q_OBJECT

public:
  enum Buttons {
    EMPTY   = 0,
    OK      = 2 << 0,
    RESET   = 2 << 1,
  };

  Popup(const QString& title_text, const QString& content_text, int buttons, QWidget *parent);
  void show();

  QLabel *content;
public slots:
  int exec();
};

