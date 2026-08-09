// DSFE_GUI DSFE_MainWindow.h
#pragma once

#include "GUIExports.h"

#include <QMainWindow>
#include <QMenu>
#include <QCloseEvent>

class QStackedWidget;
namespace gui { class SimulationManager; struct WorkspaceData; }
namespace widgets { class DSLEditorWidget; class ControlPanelWidget; }
namespace Workspace { class ProjectPage; class HomePage; }

namespace render {
	enum class ResolutionPreset;
	enum class QualityPreset;
}

namespace window {
	class DSFE_MainWindow : public QMainWindow {
	public:
		explicit DSFE_MainWindow(gui::SimulationManager* sim, QWidget* parent = nullptr);

	protected:
		void closeEvent(QCloseEvent* event) override;

	private:
		void buildMenuBar();
		void buildSceneMenu(QMenu* sceneMenu);
		void buildRobotMenu(QMenu* projectMenu);
		void buildFreeBodyMenu(QMenu* projectMenu);
		void resetRigidBody();
		void onLoadMesh();

		bool _dirty = false;
		void mark_dirty() { _dirty = true; updateTitle(); }
		void mark_clean() { _dirty = false; updateTitle(); }

		// --- Workspaces ---
		void newWorkspace();
        void openWorkspaceDialog();
        void openWorkspacePath(const QString& path);
        bool saveWorkspace();       // to current path, or Save As if none
        bool saveWorkspaceAs();
		void setWorkspaceDir();
        void gatherFullWorkspace(gui::WorkspaceData& w);
        void applyFullWorkspace(const gui::WorkspaceData& w);
        void rebuildRecentsMenu();
        void updateTitle();
		void showHomePage();
        void showProjectPage();

		void openTemplate(const QString& template_path);
		bool confirmDiscard();

		render::ResolutionPreset r;
		render::QualityPreset q;

		gui::SimulationManager* _sim;
        widgets::DSLEditorWidget* _dslEditor = nullptr;
        widgets::ControlPanelWidget* _controlPanel = nullptr;
        QMenu* _recentsMenu = nullptr;
        QString _currentWorkspacePath;
		QStackedWidget* _stack = nullptr;
        Workspace::HomePage* _homePage = nullptr;
        Workspace::ProjectPage* _projectPage = nullptr;
	};
}
