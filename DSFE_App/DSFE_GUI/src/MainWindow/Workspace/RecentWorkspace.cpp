// DSFE_GUI Workspace.cpp
#include "Workspace/RecentWorkspace.h"

#include <QSettings>
#include <QFileInfo>
#include <QStandardPaths>
#include <QDir>

namespace gui {
    static constexpr const char* KEY = "workspaces/recent";

    // Returns the list of recent workspace paths, most recent first. Prunes entries whose files no longer exist.
    QStringList RecentWorkspaces::list() {
        QSettings settings;
        QStringList paths = settings.value(KEY).toStringList();
        // Prune entries whose files no longer exist.
        QStringList alive;
        for (const QString& p : paths) {
            if (QFileInfo::exists(p)) { alive << p; }
        }
        if (alive.size() != paths.size()) { settings.setValue(KEY, alive); }
        return alive;
    }

    // Inserts a path at the front of the list, or promotes it to the front if it already exists.
    void RecentWorkspaces::add(const QString& path) {
        QSettings settings;
        QStringList paths = settings.value(KEY).toStringList();
        paths.removeAll(path);
        paths.prepend(path);
        while (paths.size() > MAX_RECENTS) { paths.removeLast(); }
        settings.setValue(KEY, paths);
    }

    // Removes a path from the recent list.
    void RecentWorkspaces::remove(const QString& path) {
        QSettings settings;
        QStringList paths = settings.value(KEY).toStringList();
        paths.removeAll(path);
        settings.setValue(KEY, paths);
    }

    // Clears the entire recent workspace list.
    void RecentWorkspaces::clear() {
        QSettings settings;
        settings.remove(KEY);
    }

    // Returns the directory where workspaces are stored. Defaults to the user's home directory.
    QString RecentWorkspaces::workspaceDir() {
        QSettings settings;
        const QString fallback = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/DSFE";
        return settings.value("workspaces/dir", fallback).toString();
    }

    // Sets the directory where workspaces are stored.
    void RecentWorkspaces::setWorkspaceDir(const QString& dir) {
        QSettings settings;
        settings.setValue("workspaces/dir", dir);
        QDir().mkpath(dir); // Ensure the directory exists.
    }

} // namespace gui