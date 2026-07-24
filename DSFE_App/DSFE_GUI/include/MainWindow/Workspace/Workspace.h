// DSFE_GUI Workspace.h
#pragma once

#include <QString>
#include <QJsonObject>
#include <glm/glm.hpp>

#include "Platform/Logger.h"

namespace gui {
    // Everything a saved workspace contains. Version field for forward migration.
    struct WorkspaceData {
        int version = 1;
        QString name;

        QString robotName;      // empty = no robot
        QString scriptText;     // DSL script embedded — file is self-contained
        QString scriptPath;     // original script file if one was opened (informational)

        // Simulation properties
        int integrationMethod = 0;      // integration::eIntegrationMethod as int
        int adIntegrationMethod = 0;    // integration::eAutoDiffIntegrationMethod as int
        double  simDt = 1.0 / 180.0;
        double telemetryDt = 1.0 / 180.0;
        bool autoDiff = false;

        // Camera
        glm::vec3 cameraPos{ 0.0f, 1.5f, 4.0f };
        float cameraYaw = -1.5708f;   // radians
        float cameraPitch = 0.0f;

        QJsonObject toJson() const;
        static WorkspaceData fromJson(const QJsonObject& obj);

        bool saveToFile(const QString& path) const;
        static bool loadFromFile(const QString& path, WorkspaceData& out);
    };

} // namespace gui