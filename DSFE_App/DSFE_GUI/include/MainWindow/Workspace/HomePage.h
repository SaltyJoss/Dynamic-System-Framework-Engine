// DSFE_GUI HomePage.h
#pragma once

#include <QWidget>
#include <functional>

#include "Platform/Logger.h"

class QListWidget;
class QVBoxLayout;
class QHBoxLayout;

namespace Workspace {
    class HomePage : public QWidget {
    public:
        explicit HomePage(QWidget* parent = nullptr);

        // Wired by DSFE_MainWindow
        std::function<void()> onNewProject;
        std::function<void()> onOpenProject;
        std::function<void(const QString&)> onOpenRecent;
        std::function<void(const QString&)> onOpenTemplate;

        void refreshRecents(); // Repopulate the recents list

    private:
        void buildHeader(QVBoxLayout* into);
        void buildTemplatesArea(QHBoxLayout* into);
        void buildDiagnosticsPanel(QHBoxLayout* into);
        void populateDiagnostics();

        QWidget* _diag_content = nullptr;
        QWidget* _footer_content = nullptr;
        QListWidget* _recents_list = nullptr;
    };
} // namespace Workspace