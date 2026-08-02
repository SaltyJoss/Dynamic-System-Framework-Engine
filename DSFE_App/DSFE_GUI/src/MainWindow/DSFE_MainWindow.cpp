//DSFE_GUI DSFE_MainWindow.cpp
#include "MainWindow/DSFE_MainWindow.h"
#include "Simulation/SimulationManager.h"
#include "Scene/SimulationCore.h"

#include "Workspace/ProjectPage.h"
#include "Widgets/DSLEditorWidget.h"
#include "Widgets/ControlPanelWidget.h"
#include "Workspace/Workspace.h"
#include "Workspace/RecentWorkspace.h"
#include "Workspace/HomePage.h"

#include "ui/RenderPreset.h"

#include "Platform/SystemMap.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QMenuBar>
#include <QFileDialog>
#include <QFileInfo>
#include <QStandardPaths>
#include <QStackedWidget>
#include <QMessageBox>

#include "Platform/Paths.h"

namespace window {
	DSFE_MainWindow::DSFE_MainWindow(gui::SimulationManager* sim, QWidget* parent)
		: QMainWindow(parent), _sim(sim), _dslEditor(nullptr)
	{
		setWindowTitle("DSFE");
		resize(1920, 1080);

		buildMenuBar();

		_stack = new QStackedWidget(this);
		_homePage = new Workspace::HomePage(_stack);
		_projectPage = new Workspace::ProjectPage(sim, _stack);
		_dslEditor = _projectPage->editor();
		if (_dslEditor) {
			_dslEditor->onContentChanged = [this]() { mark_dirty(); };
		}
		_controlPanel = _projectPage->controlPanel();
		_stack->addWidget(_homePage);       // index 0
		_stack->addWidget(_projectPage);    // index 1
		setCentralWidget(_stack);

		// IMPORTANT: ge switching happens BEFORE ANY renderer-touching call -> the viewport's renderer initialises in its showEvent, which fires on first switch
		_homePage->onNewProject   = [this]() { showProjectPage(); newWorkspace(); };
		_homePage->onOpenProject  = [this]() { openWorkspaceDialog(); };
		_homePage->onOpenRecent   = [this](const QString& p) { openWorkspacePath(p); };
		_homePage->onOpenTemplate = [this](const QString& p) { openTemplate(p); };

		showHomePage();
		updateTitle();
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
			auto* newProjectAction = newMenu->addAction("Project");
			connect(newProjectAction, &QAction::triggered, this, [this]() { newWorkspace(); });
			newMenu->addSeparator();
			auto* newScriptAction = newMenu->addAction("Script");
			connect(newScriptAction, &QAction::triggered, this, [this]() {
				if (_dslEditor) { _dslEditor->setScriptText(QString()); }
			});

			auto* openMenu = fileMenu->addMenu("Open");
			auto* openProjectAction = openMenu->addAction("Project");
			connect(openProjectAction, &QAction::triggered, this, [this]() { openWorkspaceDialog(); });
			openMenu->addSeparator();
			auto* openScriptAction = openMenu->addAction("Script");
			connect(openScriptAction, &QAction::triggered, this, [this]() {
				QString fileName = QFileDialog::getOpenFileName(nullptr, "Open Script", QString::fromStdString((paths::assets() / "DSLScripts").string()), "DSL Script Files (*.dsl);;Text Files (*.txt)");
				if (fileName.isEmpty()) { return; }
				if (!_dslEditor) { LOG_ERROR("DSL Editor not found!"); return; }
				_dslEditor->loadScript(fileName);
			});

			_recentsMenu = fileMenu->addMenu("Recent Projects");
			rebuildRecentsMenu();

			fileMenu->addSeparator();
			auto* setFolderAction = fileMenu->addAction("Set Workspace Folder…");
			connect(setFolderAction, &QAction::triggered, this, [this]() { setWorkspaceDir(); });

			fileMenu->addSeparator();
			auto* saveMenu = fileMenu->addMenu("Save");
			auto* saveProjectAction = saveMenu->addAction("Project");
			connect(saveProjectAction, &QAction::triggered, this, [this]() { saveWorkspace(); });
			auto* saveScriptAction = saveMenu->addAction("Script");
			connect(saveScriptAction, &QAction::triggered, this, [this]() {
				QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Script", QString::fromStdString((paths::assets() / "DSLScripts").string()), "DSL Script Files (*.dsl);;Text Files (*.txt)");
				if (fileName.isEmpty()) { return; }
				if (!_dslEditor) { LOG_ERROR("DSL Editor not found!"); return; }
				_dslEditor->saveScript(fileName);
			});
			auto* saveAsMenu = fileMenu->addMenu("Save As");
			auto* saveProjectAsAction = saveAsMenu->addAction("Project");
			connect(saveProjectAsAction, &QAction::triggered, this, [this]() { saveWorkspaceAs(); });
			auto* saveScriptAsAction = saveAsMenu->addAction("Script");
			connect(saveScriptAsAction, &QAction::triggered, this, [this]() {
				QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Script As", QString::fromStdString((paths::assets() / "DSLScripts").string()), "DSL Script Files (*.dsl);;Text Files (*.txt)");
				if (fileName.isEmpty()) { return; }
				if (!_dslEditor) { LOG_ERROR("DSL Editor not found!"); return; }
				_dslEditor->saveScript(fileName);
			});
			fileMenu->addSeparator();
			auto* closeProjectAction = fileMenu->addAction("Close Project");
			connect(closeProjectAction, &QAction::triggered, this, [this]() {
				newWorkspace();      // stop script, tear down, clear editor/panel/path
				showHomePage();
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
			connect(newProjectAction, &QAction::triggered, this, [this]() { showProjectPage(); newWorkspace(); });
			auto* loadProjectAction = projectMenu->addAction("Load Project");
			connect(loadProjectAction, &QAction::triggered, this, [this]() { openWorkspaceDialog(); });
			auto* saveProjectAction = projectMenu->addAction("Save Project");
			connect(saveProjectAction, &QAction::triggered, this, [this]() { saveWorkspace(); });
			projectMenu->addSeparator();
			auto* robotMenu = projectMenu->addMenu("Load Robot");
			connect(robotMenu, &QMenu::aboutToShow, this, [this, robotMenu]() {
				robotMenu->clear();
				buildRobotMenu(robotMenu);
			});
			auto* loadMeshAction = projectMenu->addAction("Load Mesh");
			connect(loadMeshAction, &QAction::triggered, this, [this]() {
				LOG_INFO("Menu clicked: Project -> Load Mesh");
				onLoadMesh();
			});
		}
		// View menu
		{
			auto* sceneMenu = viewMenu->addMenu("Scene");
			buildSceneMenu(sceneMenu);
			viewMenu->addSeparator();
			auto* resetRigidBody = viewMenu->addAction("Reset RigidBody");
			connect(resetRigidBody, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: View -> Reset RigidBody");

			});
			auto* resetCameraAction = viewMenu->addAction("Reset Camera");
			connect(resetCameraAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: View -> Reset Camera");
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

	void DSFE_MainWindow::buildSceneMenu(QMenu* sceneMenu) {
		auto* toggleOrientatorAction = sceneMenu->addAction("Toggle Orientator");
		toggleOrientatorAction->setCheckable(true);
		//toggleOrientatorAction->setChecked(_sim->isOrientastorEnabled());
		connect(toggleOrientatorAction, &QAction::toggled, this, [this](bool checked) {
			LOG_INFO("Menu toggled: Scene -> Toggle Orientator -> %s", checked ? "On" : "Off");
			//_sim->enableOrientator(checked);
		});
	}

	void DSFE_MainWindow::resetRigidBody() {
		if (!_sim) { LOG_ERROR("Simulation Manager not found!"); return; }
		_sim->resetRigidBody();
	}

	// Build the robot menu dynamically based on the available robotic systems, using the general RigidBody System interface
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
				std::string n = robotName.toStdString();
				std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c){ return std::tolower(c); });
				const std::string path = "rigidbody_models/" + n + "/" + n + ".urdf";
				LOG_INFO("Menu clicked: Project -> Load Robot -> %s", path.c_str());
				showProjectPage(); _sim->load_rigidBody(path);
			});
		}
	}

	void DSFE_MainWindow::onLoadMesh() {
		const QString path = QFileDialog::getOpenFileName(nullptr, "Select Mesh File", QString::fromStdString((paths::assets() / "objects" / "Shapes").string()), "Mesh Files(*.obj * .fbx * .gltf * .dae * .stl)");
		if (path.isEmpty()) { return; }
		std::string bodyName = QFileInfo(path).baseName().toStdString();
		//_sim->simCore()->loadSingleBody(bodyName);
		_sim->load_mesh(path.toStdString());
	}

	// ---- Workspaces ----

	static QString workspacesDir() { return QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/DSFE"; }

	void DSFE_MainWindow::newWorkspace() {
		if (!confirmDiscard()) { return; }
		if (_sim->isScriptRunning() && _dslEditor) { _dslEditor->stopScript(); }
		_sim->closeWorkspace();
		if (_dslEditor) { _dslEditor->setScriptText(QString()); }
		if (_controlPanel) {
			_controlPanel->refreshFromSim();
			_controlPanel->refreshSelectorTree();
		}
		_currentWorkspacePath.clear();
		mark_clean();
		updateTitle();
	}

	void DSFE_MainWindow::openWorkspaceDialog() {
		const QString path = QFileDialog::getOpenFileName(this, "Open Project", gui::RecentWorkspaces::workspaceDir(), "DSFE Projects (*.dsfe)");
		if (path.isEmpty()) { return; }
		openWorkspacePath(path);
	}

	void DSFE_MainWindow::openWorkspacePath(const QString& path) {
		if (!confirmDiscard()) { return; }
		gui::WorkspaceData w;
		if (!gui::WorkspaceData::loadFromFile(path, w)) {
			gui::RecentWorkspaces::remove(path);
			rebuildRecentsMenu();
			return;
		}
		showProjectPage(); // IMPORTANT: renderer must be initialised before applyWorkspace loads the robot
		applyFullWorkspace(w);
		_currentWorkspacePath = path;
		gui::RecentWorkspaces::add(path);
		rebuildRecentsMenu();
		updateTitle();
	}

	bool DSFE_MainWindow::saveWorkspace() {
		if (_currentWorkspacePath.isEmpty()) { return saveWorkspaceAs(); }
		gui::WorkspaceData w;
		gatherFullWorkspace(w);
		if (!w.saveToFile(_currentWorkspacePath)) { return false; }
		gui::RecentWorkspaces::add(_currentWorkspacePath);
		rebuildRecentsMenu();
		mark_clean();
		return true;
	}

	bool DSFE_MainWindow::saveWorkspaceAs() {
		const QString path = QFileDialog::getSaveFileName(this, "Save Project As", gui::RecentWorkspaces::workspaceDir() + "/untitled.dsfe", "DSFE Projects (*.dsfe)");
		if (path.isEmpty()) { return false; }
		_currentWorkspacePath = path;
		updateTitle();
		return saveWorkspace();
	}

	void DSFE_MainWindow::setWorkspaceDir() {
		const QString dir = QFileDialog::getExistingDirectory(this, "Set Workspace Directory", gui::RecentWorkspaces::workspaceDir());
		if (dir.isEmpty()) { return; }
		gui::RecentWorkspaces::setWorkspaceDir(dir);
		LOG_INFO("Workspace directory set to: %s", dir.toUtf8().constData());
	}

	void DSFE_MainWindow::gatherFullWorkspace(gui::WorkspaceData& w) {
		_sim->gatherWorkspace(w);
		if (_dslEditor) {
			w.scriptText = _dslEditor->scriptText();
			w.scriptPath = _dslEditor->currentScriptPath();
		}
		w.name = QFileInfo(_currentWorkspacePath).baseName();
	}

	void DSFE_MainWindow::applyFullWorkspace(const gui::WorkspaceData& w) {
		if (_sim->isScriptRunning() && _dslEditor) { _dslEditor->stopScript(); }
		_sim->closeWorkspace();
		_sim->applyWorkspace(w);
		if (_dslEditor) { _dslEditor->setScriptText(w.scriptText); }
		if (_controlPanel) {
			_controlPanel->refreshFromSim();
			_controlPanel->refreshSelectorTree();
		}
		mark_clean();
	}

	void DSFE_MainWindow::rebuildRecentsMenu() {
		if (!_recentsMenu) { return; }
		_recentsMenu->clear();
		const QStringList recents = gui::RecentWorkspaces::list();
		if (recents.isEmpty()) { _recentsMenu->addAction("(none)")->setEnabled(false); return; }
		for (const QString& path : recents) {
			QAction* a = _recentsMenu->addAction(QFileInfo(path).baseName());
			a->setToolTip(path);
			connect(a, &QAction::triggered, this, [this, path]() { openWorkspacePath(path); });
		}
		_recentsMenu->addSeparator();
		QAction* clear = _recentsMenu->addAction("Clear List");
		connect(clear, &QAction::triggered, this, [this]() { gui::RecentWorkspaces::clear(); rebuildRecentsMenu(); });
	}

	void DSFE_MainWindow::updateTitle() {
		const QString name = _currentWorkspacePath.isEmpty() ? QStringLiteral("Untitled") : QFileInfo(_currentWorkspacePath).baseName();
		setWindowTitle("DSFE: " + name);
	}

	// showHomePage switches to the home page, which will refresh the recent projects list
	void DSFE_MainWindow::showHomePage() {
		if (_homePage) { _homePage->refreshRecents(); }
		if (_stack && _homePage) { _stack->setCurrentWidget(_homePage); }
		menuBar()->setVisible(false);
	}
	// showProjectPage switches to the project page, which will initialise the renderer if we're still on the home page
	void DSFE_MainWindow::showProjectPage() { 
		if (_stack && _projectPage) { _stack->setCurrentWidget(_projectPage); }
		menuBar()->setVisible(true);
	}

	// openTemplate loads a workspace template from a file, applies it, and clears the current workspace path
	void DSFE_MainWindow::openTemplate(const QString& template_path) {
		if (!confirmDiscard()) { return; }
		gui::WorkspaceData w;
		if (!gui::WorkspaceData::loadFromFile(template_path, w)) {
			LOG_ERROR("Failed to load template: %s", template_path.toUtf8().constData());
			return;
		}
		showProjectPage(); // IMPORTANT: renderer must be initialised before applyWorkspace loads the robot
		applyFullWorkspace(w);
		_currentWorkspacePath.clear();
		mark_dirty();
		updateTitle();
	}

	bool DSFE_MainWindow::confirmDiscard() {
		const bool running = _sim && _sim->isSimRunning();
		if (!_dirty && !running) { return true; }
		QString msg;
		if (_dirty && running) {
			msg = "The current project has unsaved changes and the simulation is running.";
		} else if (running) {
			msg = "The simulation is currently running.";
		} else {
			msg = "The current project has unsaved changes.";
		}

		QMessageBox box(this);
		box.setWindowTitle("DSFE");
		box.setIcon(QMessageBox::Warning);
		box.setText(msg);
		box.setInformativeText("Do you want to save before continuing?");
		box.setStandardButtons(QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
		box.setDefaultButton(QMessageBox::Save);

		const int choice = box.exec();
		if (choice == QMessageBox::Cancel) { return false; }
		if (choice == QMessageBox::Save) {
			if (running && _dslEditor && _sim->isScriptRunning()) {
				_dslEditor->stopScript();
			}
			if (!saveWorkspace()) {
				LOG_ERROR("Failed to save workspace.");
				return false;
			}
		}
		if (running && _dslEditor && _sim->isScriptRunning()) {
			_dslEditor->stopScript();
		}
		return true;
	}

	void DSFE_MainWindow::closeEvent(QCloseEvent* event) {
		if (confirmDiscard()) {
			event->accept();
		} else {
			event->ignore(); // user hit Cancel
		}
	}

} // namespace window