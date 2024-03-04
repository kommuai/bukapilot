/*
 * included_by: home.cc
 *
 * WHEN IN DOUBT, DO NOT INCLUDE, DO NOT USE.
 */

#include <string>
#include <vector>

#include <QString>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/widgets/popup.h"

class AlertsManager {

  std::vector<std::pair<std::string, int>> alerts;
  Params params;

public:
  bool hasSevere = false;
  QString text;
  int unread = 0;

  AlertsManager();
  void refresh();
};

#ifdef K_IMPL

#include <QJsonDocument>
#include <QJsonObject>

AlertsManager::AlertsManager() {
  QString json = util::read_file("../controls/lib/alerts_offroad.json").c_str();
  auto obj = QJsonDocument::fromJson(json.toUtf8()).object();

  // descending sort labels by severity
  for (auto it = obj.constBegin(); it != obj.constEnd(); ++it) {
    alerts.push_back({it.key().toStdString(), it.value()["severity"].toInt()});
  }
  std::sort(alerts.begin(), alerts.end(), [=](auto &l, auto &r) { return l.second > r.second; });
}

void AlertsManager::refresh() {
  hasSevere = false;
  text = "";
  unread = 0;

  for (const auto& [key, severity] : alerts) {
    auto bytes = params.get(key);
    if (bytes.size()) {
      auto doc_par = QJsonDocument::fromJson(bytes.c_str());
      text += doc_par["text"].toString();
      text += "\n\n";
      hasSevere |= (severity > 0);
      unread++;
    }
  }
}

#endif
