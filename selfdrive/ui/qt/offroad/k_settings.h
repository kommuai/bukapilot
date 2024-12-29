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
#include "selfdrive/common/util.h"

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
  FeaturesControl() : ButtonControl("Features Package", "EDIT", "Warning: Only use under guidance of a support staff.") {
    package_label = new QLabel();
    package_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    package_label->setStyleSheet("color: #aaaaaa");
    setElidedText(package_label, Params().get("FeaturesPackage").c_str());
    hlayout->insertWidget(1, package_label);
    connect(this, &ButtonControl::clicked, [=] {
      InputDialog dialog("Enter Feature Package Names", this,
      "Feature package names are separated by commas ( \u2009<b>,</b>\u2009 ).<br>Empty to use the default feature package.");
      dialog.setMinLength(0);
      QString currentFeatures = Params().get("FeaturesPackage").c_str();
      if (currentFeatures != "default") dialog.updateDefaultText(currentFeatures + ", ");
      if (dialog.exec() == QDialog::Accepted) {
        QString packageText = dialog.text().trimmed();
        int result = Features().set_package(packageText.toStdString());
        setElidedText(package_label, Params().get("FeaturesPackage").c_str());
        if (result == -1) ConfirmationDialog::alert("\nSome feature package names\nare invalid and were not added.", this);
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
    setElidedText(selection_label, Params().get("FixFingerprint").c_str());
    hlayout->insertWidget(1, selection_label);
    connect(this, &ButtonControl::clicked, [=] {
      QString package = InputDialog::getText("Enter Car Model", this).trimmed();
      Params().put("FixFingerprint", package.isEmpty() ? "" : package.toStdString());
      setElidedText(selection_label, Params().get("FixFingerprint").c_str());
    });
  }

private:
  QLabel *selection_label;
};

class ChangeBranchSelect : public ButtonControl {
  Q_OBJECT

public:
  ChangeBranchSelect() : ButtonControl("Change Branch", "SET", "Warning: Untested branches may cause unexpected behaviours.") {
    selection_label = new QLabel();
    selection_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    selection_label->setStyleSheet("color: #aaaaaa");
    // Get the current branch name
    std::string currentBranchName = util::check_output("git symbolic-ref --short HEAD");
    currentBranchName.erase(currentBranchName.find_last_not_of("\n") + 1); // Remove line feed
    // Display the current branch name even if it's invalid or empty
    setElidedText(selection_label, QString::fromStdString(currentBranchName));

    // Only use fallback if branch is empty or not found remotely
    if (currentBranchName.empty() || !isBranchInRemote(currentBranchName)) currentBranchName = "snapshot";

    system(setUpstream(currentBranchName).c_str());
    hlayout->insertWidget(1, selection_label);
    connect(this, &ButtonControl::clicked, [=] {
      QString branchName = InputDialog::getText("Enter Branch Name", this).trimmed();
      if (branchName.isEmpty()) return;
      // Check if the branch is already the current branch
      if (branchName == QString::fromStdString(currentBranchName)) {
        ConfirmationDialog::alert("You are already using the branch\n" + QString::fromStdString(currentBranchName), this);
      } else { confirmAndChangeBranch(branchName.toStdString()); }
    });
  }

private:
  // Check if the branch exists in the remote repository
  bool isBranchInRemote(const std::string& branch) {
    return (std::system(("git ls-remote --heads origin " + branch + " | grep -q refs/heads/" + branch).c_str()) == 0);
  }

  // Set upstream for the current branch
  std::string setUpstream(const std::string& branch) {
    return "git config remote.origin.fetch '+refs/heads/" + branch + ":refs/remotes/origin/" + branch + "'; "
           "git fetch origin '" + branch + "'; git branch -u origin/" + branch + "; "
           "git config branch." + branch + ".merge refs/heads/" + branch;
  }

  // Confirm the branch change and perform the action
  void confirmAndChangeBranch(const std::string& branchName) {
    if (ConfirmationDialog::confirm("Are you sure to Change Branch?\nAny unsaved changes will be lost.\n\nReboot required, please wait.", this)) {
      QString errorMessage = "Branch " + QString::fromStdString(branchName) + " not found.\n\nPlease make sure the branch name is correct and the device is connected to the Internet.";
      // Check if the branch exists remotely and attempt the branch change and upstream setup
      std::string changeBranchCommand = "git branch -D " + branchName + "; git fetch origin " + branchName + ":" + branchName + " && git checkout " + branchName + " --force";
      std::string changeBranchConfigReboot = changeBranchCommand + " && " + setUpstream(branchName) + " && reboot";
      // If the branch doesn't exist remotely or the change command fails, show the error message
      if (!isBranchInRemote(branchName) || system(changeBranchConfigReboot.c_str()) != 0) ConfirmationDialog::alert(errorMessage, this);
    }
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
  void hideEvent(QHideEvent *event) override;
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
  QTimer *timer;
};

class C2NetworkPanel: public ListWidget {
  Q_OBJECT
public:
  explicit C2NetworkPanel(QWidget* parent = nullptr);
private:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void updateLabels();
  QString getIPAddress();
  QString getNetworkType();
  LabelControl *ipaddress;
  LabelControl *networkType;
  QTimer *timer;
};

