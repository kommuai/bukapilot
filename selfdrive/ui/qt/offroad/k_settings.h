#pragma once

#include <QButtonGroup>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>

#include "selfdrive/common/features.h"
#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"

#include <iostream>
#include <cstdarg>
#include <string>
#include <fstream>
#include <memory>
#include <cstdio>

// ********** settings window + top-level panels **********
class SettingsWindow : public QFrame {
  Q_OBJECT

public:
  explicit SettingsWindow(QWidget *parent = 0);

protected:
  void hideEvent(QHideEvent *event) override;
  void showEvent(QShowEvent *event) override;

signals:
  void closeSettings();
  void showDriverView();

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;
};

class DevicePanel : public ListWidget {
  Q_OBJECT
public:
  explicit DevicePanel(SettingsWindow *parent);
signals:
  void showDriverView();

private slots:
  void poweroff();
  void reboot();
  void updateCalibDescription();

private:
  ButtonControl *resetCalibBtn;
  ButtonControl *serialBtn;
  ButtonControl *testBtn;
  ButtonControl *replaceSplashBtn;
  ButtonControl *dumpTmuxBtn;
  SpinboxControl *stopDistanceOffsetSb;
  SpinboxControl *drivePathOffsetSb;
  SpinboxControl *fanPwmOverrideSb;
  SpinboxControl *powerSaverEntryDurationSb;

  int dev_tab_counter = 0;
  Params params;
};

class TogglesPanel : public ListWidget {
  Q_OBJECT
public:
  explicit TogglesPanel(SettingsWindow *parent);
};

class PersonalisedPanel : public ListWidget {
  Q_OBJECT
public:
  explicit PersonalisedPanel(QWidget* parent = nullptr);
private:
  SpinboxControl *stopDistanceOffsetSb;
  SpinboxControl *drivePathOffsetSb;
  SpinboxControl *fanPwmOverrideSb;
  SpinboxControl *powerSaverEntryDurationSb;

};

class FeaturesControl : public ButtonControl {
  Q_OBJECT

public:
  FeaturesControl() : ButtonControl("Features Package", "SET", "Warning: Only use under guidance of a support staff.") {
    package_label = new QLabel();
    package_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    package_label->setStyleSheet("color: #aaaaaa");
    package_label->setText(Params().get("FeaturesPackage").c_str());
    hlayout->insertWidget(1, package_label);
    connect(this, &ButtonControl::clicked, [=] {
      QString package = InputDialog::getText("Enter Feature Package Name", this);
      if (package.length() > 0) {
        Features().set_package(package.toStdString());
        package_label->setText(Params().get("FeaturesPackage").c_str());
      }
    });
  }

private:
  QLabel *package_label;
};

class FixFingerprintSelect : public ButtonControl {
  Q_OBJECT

public:
  FixFingerprintSelect() : ButtonControl("Fix Fingerprint", "SET", "Warning: Selecting the wrong car fingerprint can be dangerous!") {
    selection_label = new QLabel();
    selection_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    selection_label->setStyleSheet("color: #aaaaaa");
    selection_label->setText(Params().get("FixFingerprint").c_str());
    hlayout->insertWidget(1, selection_label);
    connect(this, &ButtonControl::clicked, [=] {
      QString package = InputDialog::getText("Enter Car Model", this);
      if (package.length() > 0) {
        Params().put("FixFingerprint",package.toStdString());
      }
      else {
        Params().put("FixFingerprint", "");
      }
      selection_label->setText(Params().get("FixFingerprint").c_str());
    });
  }

private:
  QLabel *selection_label;
};

class ChangeBranchSelect : public ButtonControl {
  Q_OBJECT

// Function to use C++ to execute a Bash command
// Modified from https://gist.github.com/meritozh/f0351894a2a4aa92871746bf45879157
std::string exec(const char* cmd) {
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    result.pop_back(); // Remove the last character (line feed)
    return result;
}

public:
  ChangeBranchSelect() : ButtonControl("Change Branch", "SET", "Warning: Changing the branch can be dangerous!") {
    selection_label = new QLabel();
    selection_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    selection_label->setStyleSheet("color: #aaaaaa");
    std::string currentBranchName = exec("git symbolic-ref --short HEAD"); // Display current branch name
    selection_label->setText(QString::fromStdString(currentBranchName));
    hlayout->insertWidget(1, selection_label);
    connect(this, &ButtonControl::clicked, [=] {
      QString package = InputDialog::getText("Enter Branch Name", this);
      std::string branchName = package.toStdString();
      if (branchName.back() == ' ') {branchName.pop_back();} // Remove extra space if it exists at end of input
      if (branchName.length() > 0) {
        if (branchName == currentBranchName) {
          QString currentPrompt = QString::fromStdString("You are already using the branch\n" + currentBranchName);
          ConfirmationDialog::alert(currentPrompt, this);
        } else if (ConfirmationDialog::confirm("Are you sure to Change Branch?\nAny unsaved changes will be lost.", this)) {
          // Delete branch, fetch and checkout to the branch. (Ignoring changes not committed/not pushed.)
          std::string changeBranchCommand = "git branch -D " + branchName + "; git fetch origin " + branchName + ":" + branchName + " && git checkout " + branchName + " --force";

          // After change, update config fetch, set upstream (for update), then reboot.
          std::string changeBranchConfigReboot = changeBranchCommand + " && git config remote.origin.fetch '+refs/heads/" + branchName + ":refs/remotes/origin/" + branchName + "' && git branch -u origin/" + branchName + " && reboot";
          int branchChanged = system(changeBranchConfigReboot.c_str());
		      if (branchChanged != 0) { // If branch not found (error/fatal)
			      QString failedPrompt = QString::fromStdString("Branch " + branchName + " not found.\n\nPlease make sure the branch name is correct and the device is connected to the Internet.");
		        (ConfirmationDialog::alert(failedPrompt, this));
          }
        }
                }
    });
  }

private:
  QLabel *selection_label;
};

class SoftwarePanel : public ListWidget {
  Q_OBJECT
public:
  explicit SoftwarePanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  void updateLabels();

  LabelControl *gitCommitLbl;
  LabelControl *osVersionLbl;
  LabelControl *versionLbl;
  LabelControl *lastUpdateLbl;
  ButtonControl *updateBtn;
  FixFingerprintSelect *fingerprintInput;
  ChangeBranchSelect *branchInput;
  FeaturesControl *featuresInput;

  Params params;
  QFileSystemWatcher *fs_watch;
};

class C2NetworkPanel: public ListWidget {
  Q_OBJECT
public:
  explicit C2NetworkPanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  QString getIPAddress();
  LabelControl *ipaddress;
};

