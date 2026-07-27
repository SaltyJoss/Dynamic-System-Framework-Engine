// DSFE_GUI Workspace.cpp
#include "Workspace/Workspace.h"
#include "EngineLib/LogMacros.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>

namespace gui {
    QJsonObject WorkspaceData::toJson() const {
        QJsonObject o;
        o["version"] = version;
        o["name"] = name;

        QJsonObject content;
        content["rigid_body"] = rigidBodyName;
        content["script_text"] = scriptText;
        content["script_path"] = scriptPath;
        o["content"] = content;

        QJsonObject sim;
        sim["integration_method"] = integrationMethod;
        sim["ad_integration_method"] = adIntegrationMethod;
        sim["sim_dt"] = simDt;
        sim["telemetry_dt"] = telemetryDt;
        sim["auto_diff"] = autoDiff;
        sim["gravity"] = QJsonArray{ gravity.x, gravity.y, gravity.z };
        o["simulation"] = sim;

        QJsonObject cam;
        cam["pos"] = QJsonArray{ cameraPos.x, cameraPos.y, cameraPos.z };
        cam["yaw"] = cameraYaw;
        cam["pitch"] = cameraPitch;
        o["camera"] = cam;

        return o;
    }

    WorkspaceData WorkspaceData::fromJson(const QJsonObject& o) {
        WorkspaceData w;
        w.version = o["version"].toInt(1);
        w.name = o["name"].toString();

        const QJsonObject content = o["content"].toObject();
        w.rigidBodyName = content["rigid_body"].toString();
        w.scriptText = content["script_text"].toString();
        w.scriptPath = content["script_path"].toString();

        const QJsonObject sim = o["simulation"].toObject();
        w.integrationMethod = sim["integration_method"].toInt(0);
        w.adIntegrationMethod = sim["ad_integration_method"].toInt(0);
        w.simDt = sim["sim_dt"].toDouble(1.0 / 180.0);
        w.telemetryDt = sim["telemetry_dt"].toDouble(1.0 / 180.0);
        w.autoDiff = sim["auto_diff"].toBool(false);
        const QJsonArray gravity = sim["gravity"].toArray();
        if (gravity.size() == 3) {
            w.gravity = { (float)gravity[0].toDouble(), (float)gravity[1].toDouble(), (float)gravity[2].toDouble() };
        }

        const QJsonObject cam = o["camera"].toObject();
        const QJsonArray pos = cam["pos"].toArray();
        if (pos.size() == 3) {
            w.cameraPos = { (float)pos[0].toDouble(), (float)pos[1].toDouble(), (float)pos[2].toDouble() };
        }
        w.cameraYaw = (float)cam["yaw"].toDouble(-1.5708);
        w.cameraPitch = (float)cam["pitch"].toDouble(0.0);

        return w;
    }

    bool WorkspaceData::saveToFile(const QString& path) const {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            LOG_ERROR("Workspace save failed, cannot open: %s", path.toUtf8().constData());
            return false;
        }
        file.write(QJsonDocument(toJson()).toJson(QJsonDocument::Indented));
        LOG_INFO("Workspace saved: %s", path.toUtf8().constData());
        return true;
    }

    bool WorkspaceData::loadFromFile(const QString& path, WorkspaceData& out) {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly)) {
            LOG_ERROR("Workspace load failed, cannot open: %s", path.toUtf8().constData());
            return false;
        }
        QJsonParseError err{};
        const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &err);
        if (err.error != QJsonParseError::NoError || !doc.isObject()) {
            LOG_ERROR("Workspace parse failed (%s): %s", err.errorString().toUtf8().constData(), path.toUtf8().constData());
            return false;
        }
        out = fromJson(doc.object());
        LOG_INFO("Workspace loaded: %s", path.toUtf8().constData());
        return true;
    }

} // namespace gui