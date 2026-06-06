//DSFE_GUI DSFE_MainWindow.cpp
#include "MainWindow/DSFE_MainWindow.h"
#include "Scene/SimulationManager.h"
#include "Workspace/ProjectPage.h"

#include <QAction>
#include <QApplication>
#include <QMenu>
#include <QMenuBar>

namespace window {
	DSFE_MainWindow::DSFE_MainWindow(gui::SimManager* sim, QWidget* parent) : QMainWindow(parent), _sim(sim) {
		setWindowTitle("DSFE");
		resize(1280, 720);

		buildMenuBar();

		auto* page = new Workspace::ProjectPage(sim, this);
		setCentralWidget(page);
	}

	void DSFE_MainWindow::buildMenuBar() {
		auto* fileMenu = menuBar()->addMenu("&File");
		auto* editMenu = menuBar()->addMenu("&Project");
		auto* viewMenu = menuBar()->addMenu("&View");
		auto* ToolsMenu = menuBar()->addMenu("&Tools");
		auto* helpMenu = menuBar()->addMenu("&Help");

		// File menu
		{
			auto* openAction = new QAction("Open", this);
			connect(openAction, &QAction::triggered, this, []() { 
				LOG_INFO("Menu clicked: File -> Open");
			});
			auto* saveAction = new QAction("Save", this);
			connect(saveAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: File -> Save");
			});
			fileMenu->addSeparator();
			auto* exitAction = new QAction("Exit", this);
			connect(exitAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: File -> Exit");
				QApplication::quit();
			});
		}
		// Project menu
		{
			auto* newProjectAction = new QAction("New Project", this);
			connect(newProjectAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Project -> New Project");
			});
			auto* loadProjectAction = new QAction("Load Project", this);
			connect(loadProjectAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Project -> Load Project");
			});
			auto* saveProjectAction = new QAction("Save Project", this);
			connect(saveProjectAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Project -> Save Project");
			});
			fileMenu->addSeparator();
			auto* loadRobotAction = new QAction("Load Robot", this);
			connect(loadRobotAction, &QAction::triggered, this, [this]() {
				LOG_INFO("Menu clicked: Project -> Load Robot");
				_sim->loadRobot("panda");
			});
			auto* loadMeshAction = new QAction("Load Mesh", this);
			connect(loadMeshAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Project -> Load Mesh");
			});
			auto* loadHDRAction = new QAction("Load HDRI", this);
			connect(loadHDRAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Project -> Load HDRI");
			});
		}
		// View menu
		{
			auto* resetCameraAction = new QAction("Reset Camera", this);
			connect(resetCameraAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: View -> Reset Camera");
			});
			auto* toggleGridAction = new QAction("Toggle Grid", this);
			toggleGridAction->setCheckable(true);
			toggleGridAction->setChecked(true);
			connect(toggleGridAction, &QAction::toggled, this, [](bool checked) {
				LOG_INFO("Menu toggled: View -> Toggle Grid -> %s", checked ? "On" : "Off");
			});
		}
		// Tools menu
		{
			auto* physicsDebugAction = new QAction("Toggle Physics Debug", this);
			physicsDebugAction->setCheckable(true);
			physicsDebugAction->setChecked(false);
			connect(physicsDebugAction, &QAction::toggled, this, [](bool checked) {
				LOG_INFO("Menu toggled: Tools -> Toggle Physics Debug -> %s", checked ? "On" : "Off");
			});
			auto* reloadShadersAction = new QAction("Reload Shaders", this);
			connect(reloadShadersAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Tools -> Reload Shaders");
			});
			auto* diagnosticsAction = new QAction("Run Diagnostics", this);
		}
		// Help menu
		{
			auto* aboutAction = new QAction("About", this);
			connect(aboutAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Help -> About");
			});
			auto* docsAction = new QAction("Documentation", this);
			connect(docsAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Help -> Documentation");
			});
		}
	}

} // namespace window