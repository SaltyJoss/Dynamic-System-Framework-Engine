// DSFE_GUI HomePage.h
#pragma once

#include <QWidget>
#include <functional>

#include "Platform/Logger.h"

class QListWidget;
class QVBoxLayout;

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
        void buildHeader(QVBoxLayout* into);
        void buildTemplateArea(QVBoxLayout* into);
        void buildDiagnosticsFooter(QVBoxLayout* into);

        QListWidget* _recents_list = nullptr;
    };
} // namespace Workspace