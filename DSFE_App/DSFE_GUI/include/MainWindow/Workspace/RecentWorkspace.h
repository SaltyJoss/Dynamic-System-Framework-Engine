// DSFE_GUI RecentWorkspace.h
#pragma once

#include <QString>
#include <QStringList>

namespace gui {
    // Recent workspace list, persisted via QSettings (registry on Windows, ini on Linux).
    class RecentWorkspaces {
    public:
        static constexpr int MAX_RECENTS = 10;

        static QStringList list(); // most recent first
        static void add(const QString& path);// inserts/promotes to front
        static void remove(const QString& path);
        static void clear();

        static QString workspaceDir();
        static void setWorkspaceDir(const QString& dir);
    };

} // namespace gui