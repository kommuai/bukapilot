#pragma once

#include <QPushButton>

#include "selfdrive/ui/qt/widgets/controls.h"

class BranchControl : public ButtonControl {
  Q_OBJECT

public:
  BranchControl();

  void refresh();

private:
  Params params;
  QLabel branch_label;

  std::string getRealBranch();
  std::string readFile(const std::string filename);
  void switchToBranch(const QString &branch);
};
