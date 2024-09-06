#include "selfdrive/ui/qt/widgets/controls.h"

#include <QPainter>
#include <QStyleOption>
#include <QLineEdit>
#include <QDoubleSpinBox>

QFrame *horizontal_line(QWidget *parent) {
  QFrame *line = new QFrame(parent);
  line->setFrameShape(QFrame::StyledPanel);
  line->setStyleSheet(R"(
    margin-left: 40px;
    margin-right: 40px;
    border-width: 1px;
    border-bottom-style: solid;
    border-color: gray;
  )");
  line->setFixedHeight(2);
  return line;
}

AbstractControl::AbstractControl(const QString &title, const QString &desc, const QString &icon, QWidget *parent) : QFrame(parent) {
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setMargin(0);

  hlayout = new QHBoxLayout;
  hlayout->setMargin(0);
  hlayout->setSpacing(20);

  // left icon
  if (!icon.isEmpty()) {
    QPixmap pix(icon);
    QLabel *icon_label = new QLabel();
    icon_label->setPixmap(pix.scaledToWidth(80, Qt::SmoothTransformation));
    icon_label->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
    hlayout->addWidget(icon_label);
  }

  // title
  title_label = new QPushButton(title);
  title_label->setFixedHeight(120);
  title_label->setStyleSheet("font-size: 50px; font-weight: 400; text-align: left");
  hlayout->addWidget(title_label);

  main_layout->addLayout(hlayout);

  // description
  if (!desc.isEmpty()) {
    description = new QLabel(desc);
    description->setContentsMargins(40, 20, 40, 20);
    description->setStyleSheet("font-size: 40px; color: grey");
    description->setWordWrap(true);
    description->setVisible(false);
    main_layout->addWidget(description);

    connect(title_label, &QPushButton::clicked, [=]() {
      if (!description->isVisible()) {
        emit showDescription();
      }
      description->setVisible(!description->isVisible());
    });
  }
  main_layout->addStretch();
}

void AbstractControl::hideEvent(QHideEvent *e) {
  if(description != nullptr) {
    description->hide();
  }
}

// controls
SpinboxControl::SpinboxControl(const QString &param, const QString &title, const QString &desc, const QString &unit, double range[], bool reboot_req, QWidget *parent) : AbstractControl(title, desc, "", parent) {

  reboot_required = reboot_req;
  spinbox.setRange(range[0], range[1]);
  spinbox.setSingleStep(range[2]);
  spinbox.setDecimals(1);
  spinbox.setObjectName("spinbox");

  // set default value, must set after setRange
  if (params.get(param.toStdString()) == "") {
    spinbox.setValue(0);
  }
  else {
    spinbox.setValue(std::stod(params.get(param.toStdString())));
  }

  spinbox.setSuffix(unit);
  spinbox.setAlignment(Qt::AlignHCenter);

  spinbox.setStyleSheet(R"(
    QDoubleSpinBox#spinbox {
      color: lightgray;
      border-style: none;
      width: 24px;
    }

    QDoubleSpinBox:disabled#spinbox {
      color: gray;
    }

    QDoubleSpinBox::down-button#spinbox, QDoubleSpinBox::up-button#spinbox {
      subcontrol-origin: margin;
      background: #393939;
      width: 72px;
      height: 72px;
    }

    QDoubleSpinBox::down-button#spinbox {
      subcontrol-position: center left;
    }

    QDoubleSpinBox::up-button#spinbox {
      subcontrol-position: center right;
    }

    QDoubleSpinBox::down-button#spinbox,
    QDoubleSpinBox::up-button#spinbox {
      border-radius: 12px;
    }

    QDoubleSpinBox::down-button:pressed#spinbox,
    QDoubleSpinBox::up-button:pressed#spinbox {
      background-color: #4a4a4a;
    }

    QDoubleSpinBox::up-arrow#spinbox, QDoubleSpinBox::down-arrow#spinbox {
      subcontrol-origin: content;
      width: 48px;
      height: 48px;
    }

    QDoubleSpinBox::up-arrow#spinbox{
      image: url("/data/openpilot/selfdrive/assets/kommu/plus.png");
    }

    QDoubleSpinBox::down-arrow#spinbox{
      image: url("/data/openpilot/selfdrive/assets/kommu/minus.png");
    }
  )");

  spinbox.setFixedSize(400, 100);
  hlayout->addWidget(&spinbox);

  // remove the highlighting effect
  spinbox.findChildren<QLineEdit*> ().at(0)->setReadOnly(true);
  QObject::connect(&spinbox, SIGNAL(valueChanged(double)), this, SLOT(deselectTextEdit()), Qt::QueuedConnection);
  connect(spinbox.findChild<QLineEdit*> (), SIGNAL(cursorPositionChanged(int,int)), this, SLOT(deselectTextEdit()),Qt::QueuedConnection);

  // alert reboot required
  QObject::connect(&spinbox, SIGNAL(valueChanged(double)), this, SLOT(alertRebootRequired()), Qt::QueuedConnection);

  // set params
  key = param.toStdString();
  QObject::connect(&spinbox, SIGNAL(valueChanged(double)), this, SLOT(setParams(double)), Qt::QueuedConnection);
}

ButtonControl::ButtonControl(const QString &title, const QString &text, const QString &desc, bool no_style, QWidget *parent) : AbstractControl(title, desc, "", parent) {
  btn.setText(text);
  btn.setStyleSheet(R"(
    QPushButton {
      padding: 0;
      border-radius: 50px;
      font-size: 35px;
      font-weight: 500;
      color: #E4E4E4;
      background-color: #393939;
    }
    QPushButton:pressed {
      background-color: #4a4a4a;
    }
    QPushButton:disabled {
      color: #33E4E4E4;
    }
  )");

  if (no_style) {
    btn.setStyleSheet(R"(
      QPushButton {
        border: none;
      }
    )");
  }

  btn.setFixedSize(250, 100);
  QObject::connect(&btn, &QPushButton::clicked, this, &ButtonControl::clicked);
  hlayout->addWidget(&btn);
}

// ElidedLabel

ElidedLabel::ElidedLabel(QWidget *parent) : ElidedLabel({}, parent) {}

ElidedLabel::ElidedLabel(const QString &text, QWidget *parent) : QLabel(text.trimmed(), parent) {
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  setMinimumWidth(1);
}

void ElidedLabel::resizeEvent(QResizeEvent* event) {
  QLabel::resizeEvent(event);
  lastText_ = elidedText_ = "";
}

void ElidedLabel::paintEvent(QPaintEvent *event) {
  const QString curText = text();
  if (curText != lastText_) {
    elidedText_ = fontMetrics().elidedText(curText, Qt::ElideRight, contentsRect().width());
    lastText_ = curText;
  }

  QPainter painter(this);
  drawFrame(&painter);
  QStyleOption opt;
  opt.initFrom(this);
  style()->drawItemText(&painter, contentsRect(), alignment(), opt.palette, isEnabled(), elidedText_, foregroundRole());
}

ClickableWidget::ClickableWidget(QWidget *parent) : QWidget(parent) { }

void ClickableWidget::mouseReleaseEvent(QMouseEvent *event) {
  emit clicked();
}

// Fix stylesheets
void ClickableWidget::paintEvent(QPaintEvent *) {
  QStyleOption opt;
  opt.init(this);
  QPainter p(this);
  style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
}
