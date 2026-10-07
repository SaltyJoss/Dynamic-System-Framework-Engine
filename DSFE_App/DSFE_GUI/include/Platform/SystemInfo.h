#pragma once

#include <QString>
#include <QStringList>

namespace gui {
    // SystemInfo struct holds information about the system's hardware and software environment.
    struct SystemInfo {
        QString os;             // OS name + version
        QString processor;      // CPU brand string
        QString ram;            // e.g. "32 GB"
        QString storage;        // free / total on the app drive
        QStringList gpus;       // one entry per Vulkan physical device
        QString vulkanVersion;  // API version string
        static SystemInfo query();
    };

} // namespace gui