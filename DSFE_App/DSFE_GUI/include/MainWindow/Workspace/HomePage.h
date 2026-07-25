// DSFE_GUI HomePage.h
#pragma once

#include <QWidget>
#include <functional>

class QListWidget;

namespace Workspace {
    class HomePage : public QWidget {
    public:
        explicit HomePage(QWidget* parent = nullptr);

        // Wired by DSFE_MainWindow
        std::function<void()> onNewProject;
        std::function<void()> onOpenProject;
        std::function<void(const QString&)> onOpenRecent;

        void refreshRecents(); // Repopulate the recents list

    private:
        QListWidget* _recents_list = nullptr;
    };
} // namespace Workspace