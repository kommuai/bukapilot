#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "selfdrive/ui/qt/widgets/branch_control.h"

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/widgets/input.h"

BranchControl::BranchControl() : ButtonControl("Git Branch", "", "Warning: Only switch under advice from Kommu's staff.  Unauthorised switching will impose danger to you and other road users.") {
  branch_label.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  branch_label.setStyleSheet("color: #aaaaaa");
  hlayout->insertWidget(1, &branch_label);

  QObject::connect(this, &ButtonControl::released, [=]() {
    QString branch = InputDialog::getText("Enter branch name", this);
    if (branch.length() > 0) {
      switchToBranch(branch);
    }
  });

  setText("SWITCH");
  setEnabled(true);

  refresh();
}

std::string BranchControl::readFile(const std::string filename) {
  std::stringstream buf;
  std::ifstream f(filename);
  if (!f) {
    return "_ERROR_";
  }
  buf << f.rdbuf();
  return buf.str();
}

std::string BranchControl::getRealBranch() {
  if (std::system("git rev-parse --abbrev-ref HEAD > _realbranch") == 0) {
    return readFile("_realbranch");
  }
  return "_ERROR_";
}

void BranchControl::refresh() {
  std::string label;
  auto real = getRealBranch();
  auto target = params.get("GitBranch");
  if (real == "_ERROR_") {
    label = real;
    setEnabled(false);
  } else {
    if (target != real) {
      label = target + " (pending)";
    } else {
      label = target;
    }
    setEnabled(true);
  }
  branch_label.setText(QString::fromStdString(label));
}

void BranchControl::switchToBranch(const QString &branch) {
  params.put("GitBranch", branch.toStdString());
  refresh();
}

