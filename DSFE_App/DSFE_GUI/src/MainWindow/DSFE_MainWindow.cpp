//DSFE_GUI DSFE_MainWindow.cpp
#include "MainWindow/DSFE_MainWindow.h"
#include "Scene/SimulationManager.h"
#include "Workspace/ProjectPage.h"
#include "Widgets/DSLEditorWidget.h"

#include "Platform/SystemMap.h"

#include <QAction>
#include <QApplication>
#include <QMenuBar>
#include <QFileDialog>

#include "Platform/Paths.h"

namespace window {
	DSFE_MainWindow::DSFE_MainWindow(gui::SimManager* sim, QWidget* parent)
		: QMainWindow(parent), _sim(sim), _dslEditor(nullptr)
	{
		setWindowTitle("DSFE");
		resize(1920, 1080);

		buildMenuBar();

		auto* page = new Workspace::ProjectPage(sim, this);
		_dslEditor = page->editor();
		setCentralWidget(page);
	}

	void DSFE_MainWindow::buildMenuBar() {
		auto* fileMenu = menuBar()->addMenu("&File");
		auto* projectMenu = menuBar()->addMenu("&Project");
		auto* viewMenu = menuBar()->addMenu("&View");
		auto* toolsMenu = menuBar()->addMenu("&Tools");
		auto* helpMenu = menuBar()->addMenu("&Help");

		// File menu
		{
			auto* newMenu = fileMenu->addMenu("New");
			connect(newMenu, &QMenu::aboutToShow, this, [this, newMenu]() {
				LOG_INFO("Menu clicked: File -> New");
				auto* newProjectAction = newMenu->addAction("Project");
				connect(newProjectAction, &QAction::triggered, this, []() {
					LOG_INFO("Menu clicked: File -> New -> New Project");
				});
				newMenu->addSeparator();
				auto* newScriptAction = newMenu->addAction("Script");
				connect(newScriptAction, &QAction::triggered, this, []() {
					LOG_INFO("Menu clicked: File -> New -> New Script");
				});
			});
			auto* openMenu = fileMenu->addMenu("Open");
			connect(openMenu, &QMenu::aboutToShow, this, [this, openMenu]() {
				LOG_INFO("Menu clicked: File -> Open");
				auto* openProjectAction = openMenu->addAction("Project");
				connect(openProjectAction, &QAction::triggered, this, []() {
					LOG_INFO("Menu clicked: File -> Open -> Project");
				});
				openMenu->addSeparator();
				auto* openScriptAction = openMenu->addAction("Script (*.dsl *.txt)");
				connect(openScriptAction, &QAction::triggered, this, [this]() {
					QString fileName = QFileDialog::getOpenFileName(nullptr, "Open Script", QString::fromStdString((paths::assets() / "DSLScripts").string()), "DSL Script Files (*.dsl);;Text Files (*.txt)");
					if (fileName.isEmpty()) { return; }
					if (!_dslEditor) { LOG_ERROR("DSL Editor not found!"); return; }
					_dslEditor->loadScript(fileName);
				});
			});
			fileMenu->addSeparator();
			auto* saveMenu = fileMenu->addMenu("Save");
			connect(saveMenu, &QMenu::aboutToShow, this, [this, saveMenu]() {
				LOG_INFO("Menu clicked: File -> Save");
				auto* saveProjectAction = saveMenu->addAction("Project");
				connect(saveProjectAction, &QAction::triggered, this, []() {
					LOG_INFO("Menu clicked: File -> Save -> Project");
				});
				auto* saveScriptAction = saveMenu->addAction("Script");
				connect(saveScriptAction, &QAction::triggered, this, [this]() {
					LOG_INFO("Menu clicked: File -> Save -> Script");
					QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Script", QString::fromStdString((paths::assets() / "DSLScripts").string()), "DSL Script Files (*.dsl);;Text Files (*.txt)"); // ARGS are 
					if (fileName.isEmpty()) { return; }
					if (!_dslEditor) { LOG_ERROR("DSL Editor not found!"); return; }
					_dslEditor->saveScript(fileName);
				});
			});
			auto* saveAsMenu = fileMenu->addMenu("Save As");
			connect(saveAsMenu, &QMenu::aboutToShow, this, [this, saveAsMenu]() {
				LOG_INFO("Menu clicked: File -> Save As");
				auto* saveProjectAsAction = saveAsMenu->addAction("Project");
				connect(saveProjectAsAction, &QAction::triggered, this, []() {
					LOG_INFO("Menu clicked: File -> Save As -> Project");
				});
				auto* saveScriptAsAction = saveAsMenu->addAction("Script");
				connect(saveScriptAsAction, &QAction::triggered, this, [this]() {
					LOG_INFO("Menu clicked: File -> Save As -> Script");
					QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Script As", QString::fromStdString((paths::assets() / "DSLScripts").string()), "DSL Script Files (*.dsl);;Text Files (*.txt)");
					if (fileName.isEmpty()) { return; }
					if (!_dslEditor) { LOG_ERROR("DSL Editor not found!"); return; }
					_dslEditor->saveScript(fileName);
				});
			});
			fileMenu->addSeparator();
			auto* exitAction = fileMenu->addAction("Exit");
			connect(exitAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: File -> Exit");
				QApplication::quit();
			});
		}
		// Project menu
		{
			auto* newProjectAction = projectMenu->addAction("New Project");
			connect(newProjectAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Project -> New Project");
			});
			auto* loadProjectAction = projectMenu->addAction("Load Project");
			connect(loadProjectAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Project -> Load Project");
			});
			auto* saveProjectAction = projectMenu->addAction("Save Project");
			connect(saveProjectAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Project -> Save Project");
			});
			projectMenu->addSeparator();
			auto* robotMenu = projectMenu->addMenu("Load Robot");
			connect(robotMenu, &QMenu::aboutToShow, this, [this, robotMenu]() {
				robotMenu->clear();
				buildRobotMenu(robotMenu);
			});
			auto* loadMeshAction = projectMenu->addAction("Load Mesh");
			connect(loadMeshAction, &QAction::triggered, this, [this]() {
				LOG_INFO("Menu clicked: Project -> Load Mesh");
				QString path = QFileDialog::getOpenFileName(nullptr, "Select Mesh File", QString::fromStdString((paths::assets() / "objects" / "Shapes").string()), "Mesh Files (*.obj *.fbx *.gltf *.dae *.stl");
				if (path.isEmpty()) { return; }
				_sim->loadMesh(path.toStdString());
			});
			auto* loadHDRAction = projectMenu->addAction("Load HDRI");
			connect(loadHDRAction, &QAction::triggered, this, [this]() {
				LOG_INFO("Menu clicked: Project -> Load HDRI");
				QString path = QFileDialog::getOpenFileName(nullptr, "Select HDRI File", QString::fromStdString((paths::assets() / "hdr").string()), "HDRI Files (*.hdr *.exr)");
				if (path.isEmpty()) { return; }
				_sim->loadNewHDR_UI(path.toStdString());
			});
		}
		// View menu
		{
			auto* resetCameraAction = viewMenu->addAction("Reset Camera");
			connect(resetCameraAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: View -> Reset Camera");
			});
			auto* toggleGridAction = viewMenu->addAction("Toggle Grid");
			toggleGridAction->setCheckable(true);
			toggleGridAction->setChecked(true);
			connect(toggleGridAction, &QAction::toggled, this, [](bool checked) {
				LOG_INFO("Menu toggled: View -> Toggle Grid -> %s", checked ? "On" : "Off");
			});
		}
		// Tools menu
		{
			auto* physicsDebugAction = toolsMenu->addAction("Toggle Physics Debug");
			physicsDebugAction->setCheckable(true);
			physicsDebugAction->setChecked(false);
			connect(physicsDebugAction, &QAction::toggled, this, [](bool checked) {
				LOG_INFO("Menu toggled: Tools -> Toggle Physics Debug -> %s", checked ? "On" : "Off");
			});
			auto* reloadShadersAction = toolsMenu->addAction("Reload Shaders");
			connect(reloadShadersAction, &QAction::triggered, this, [this]() {
				LOG_INFO("Menu clicked: Tools -> Reload Shaders");
				_sim->reloadAllShaders();
			});
			auto* diagnosticsAction = toolsMenu->addAction("Run Diagnostics");
			connect(diagnosticsAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Tools -> Run Diagnostics");
			});
		}
		// Help menu
		{
			auto* aboutAction = helpMenu->addAction("About");
			connect(aboutAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Help -> About");
			});
			auto* docsAction = helpMenu->addAction("Documentation");
			connect(docsAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: Help -> Documentation");
			});
		}
	}

	void DSFE_MainWindow::buildRobotMenu(QMenu* projectMenu) {
		const auto& robotMap = platform::getRobotSystemMap();
		std::unordered_map<platform::eRoboticSystemFamilies, QMenu*> familyMenus;
		for (const auto& [sys, family] : robotMap) {
			if (!familyMenus.contains(family)) {
				QString familyName = QString::fromStdString(platform::RoboticSystems().toString(family));
				familyMenus[family] = projectMenu->addMenu(familyName);
			}
			QString robotName = QString::fromStdString(platform::RoboticSystems().toString(sys));
			QAction* robotAction = familyMenus[family]->addAction(robotName);
			connect(robotAction, &QAction::triggered, this, [this, robotName]() {
				LOG_INFO("Menu clicked: Project -> Load Robot -> %s", robotName.toStdString().c_str());
				_sim->loadRobot(robotName.toStdString());
			});
		}
	}

} // namespace window