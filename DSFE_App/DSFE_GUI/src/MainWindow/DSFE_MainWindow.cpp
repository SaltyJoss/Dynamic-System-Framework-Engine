//DSFE_GUI DSFE_MainWindow.cpp
#include "MainWindow/DSFE_MainWindow.h"
#include "Simulation/SimulationManager.h"
#include "Scene/SimulationCore.h"
#include "Workspace/ProjectPage.h"
#include "Widgets/DSLEditorWidget.h"

#include "ui/RenderPreset.h"

#include "Platform/SystemMap.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QMenuBar>
#include <QFileDialog>

#include "Platform/Paths.h"

namespace window {
	DSFE_MainWindow::DSFE_MainWindow(gui::SimulationManager* sim, QWidget* parent)
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
			auto* openProjectAction = openMenu->addAction("Project");
			connect(openProjectAction, &QAction::triggered, this, []() {
				LOG_INFO("Menu clicked: File -> Open -> Project");
			});
			openMenu->addSeparator();
			auto* openScriptAction = openMenu->addAction("Script");
			connect(openScriptAction, &QAction::triggered, this, [this]() {
				QString fileName = QFileDialog::getOpenFileName(nullptr, "Open Script", QString::fromStdString((paths::assets() / "DSLScripts").string()), "DSL Script Files (*.dsl);;Text Files (*.txt)");
				if (fileName.isEmpty()) { return; }
				if (!_dslEditor) { LOG_ERROR("DSL Editor not found!"); return; }
				_dslEditor->loadScript(fileName);
			});
			fileMenu->addSeparator();
			auto* saveMenu = fileMenu->addMenu("Save");
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
				//buildRobotMenu(robotMenu);
			});
			auto* loadMeshAction = projectMenu->addAction("Load Mesh");
			connect(loadMeshAction, &QAction::triggered, this, [this]() {
				LOG_INFO("Menu clicked: Project -> Load Mesh");
				QString path = QFileDialog::getOpenFileName(nullptr, "Select Mesh File", QString::fromStdString((paths::assets() / "objects" / "Shapes").string()), "Mesh Files(*.obj * .fbx * .gltf * .dae * .stl)");
				if (path.isEmpty()) { return; }
				// Covert path name to just the file name without extension or path
				std::string bodyName = QFileInfo(path).baseName().toStdString();
				//_sim->simCore()->loadSingleBody(bodyName);
				//_sim->loadMesh(path.toStdString());
			});
			auto* loadHDRAction = projectMenu->addAction("Load HDRI");
			connect(loadHDRAction, &QAction::triggered, this, [this]() {
				LOG_INFO("Menu clicked: Project -> Load HDRI");
				QString path = QFileDialog::getOpenFileName(nullptr, "Select HDRI File", QString::fromStdString((paths::assets() / "scene_hdr").string()), "HDRI Files (*.hdr *.exr)");
				if (path.isEmpty()) { return; }
				//_sim->loadNewHDR_UI(path.toStdString());
			});
		}
		// View menu
		{
			auto* graphicsMenu = viewMenu->addMenu("Graphics Options");
			buildGraphicsMenu(graphicsMenu);
			auto* sceneMenu = viewMenu->addMenu("SceneOptions");
			buildSceneMenu(sceneMenu);
			viewMenu->addSeparator();
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

	void DSFE_MainWindow::buildGraphicsMenu(QMenu* graphicsMenu) {
		// Quality submenu
		auto* qualityMenu = graphicsMenu->addMenu("Quality");
		auto* lowQualityAction = qualityMenu->addAction("Low");
		auto* mediumQualityAction = qualityMenu->addAction("Medium");
		auto* highQualityAction = qualityMenu->addAction("High");
		auto* ultraQualityAction = qualityMenu->addAction("Ultra");
		lowQualityAction->setCheckable(true);
		mediumQualityAction->setCheckable(true);
		highQualityAction->setCheckable(true);
		ultraQualityAction->setCheckable(true);
		auto* qualityGroup = new QActionGroup(this);
		qualityGroup->setExclusive(true);
		qualityGroup->addAction(lowQualityAction);
		qualityGroup->addAction(mediumQualityAction);
		qualityGroup->addAction(highQualityAction);
		qualityGroup->addAction(ultraQualityAction);
		mediumQualityAction->setChecked(true);
		// LOW
		connect(lowQualityAction, &QAction::triggered, this, [this]() {
			q = render::QualityPreset::Low;
			auto s = render::MakeSettings(r, q);
			_sim->applyRenderProfile(s, r);
			LOG_INFO("Graphics Quality -> Low");
		});
		// MEDIUM
		connect(mediumQualityAction, &QAction::triggered, this, [this]() {
			q = render::QualityPreset::Medium;
			auto s = render::MakeSettings(r, q);
			_sim->applyRenderProfile(s, r);
			LOG_INFO("Graphics Quality -> Medium");
		});
		// HIGH
		connect(highQualityAction, &QAction::triggered, this, [this]() {
			q = render::QualityPreset::High;
			auto s = render::MakeSettings(r, q);
			_sim->applyRenderProfile(s, r);
			LOG_INFO("Graphics Quality -> High");
		});
		// ULTRA
		connect(ultraQualityAction, &QAction::triggered, this, [this]() {
			q = render::QualityPreset::Ultra;
			auto s = render::MakeSettings(r, q);
			_sim->applyRenderProfile(s, r);
			LOG_INFO("Graphics Quality -> Ultra");
		});

		// Resolution submenu
		auto* resolutionMenu = graphicsMenu->addMenu("Resolution");
		auto* r720Action = resolutionMenu->addAction("720p");
		auto* r1080Action = resolutionMenu->addAction("1080p");
		auto* r1440Action = resolutionMenu->addAction("1440p");
		auto* r4kAction = resolutionMenu->addAction("4K");
		r720Action->setCheckable(true);
		r1080Action->setCheckable(true);
		r1440Action->setCheckable(true);
		r4kAction->setCheckable(true);
		auto* resolutionGroup = new QActionGroup(this);
		resolutionGroup->setExclusive(true);
		resolutionGroup->addAction(r720Action);
		resolutionGroup->addAction(r1080Action);
		resolutionGroup->addAction(r1440Action);
		resolutionGroup->addAction(r4kAction);
		r1080Action->setChecked(true);
		// 720p
		connect(r720Action, &QAction::triggered, this, [this]() {
			r = render::ResolutionPreset::R_720p;
			auto s = render::MakeSettings(r, q);
			_sim->applyRenderProfile(s, r);
			LOG_INFO("Resolution -> 720p");
		});
		// 1080p
		connect(r1080Action, &QAction::triggered, this, [this]() {
			r = render::ResolutionPreset::R_1080p;
			auto s = render::MakeSettings(r, q);
			_sim->applyRenderProfile(s, r);
			LOG_INFO("Resolution -> 1080p");
		});
		// 1440p
		connect(r1440Action, &QAction::triggered, this, [this]() {
			r = render::ResolutionPreset::R_1440p;
			auto s = render::MakeSettings(r, q);
			_sim->applyRenderProfile(s, r);
			LOG_INFO("Resolution -> 1440p");
		});
		// 4K
		connect(r4kAction, &QAction::triggered, this, [this]() {
			r = render::ResolutionPreset::R_4K;
			auto s = render::MakeSettings(r, q);
			_sim->applyRenderProfile(s, r);
			LOG_INFO("Resolution -> 4K");
		});

		// // Shader submenu
		// auto* shaderMenu = graphicsMenu->addMenu("Shaders");
		// auto* basicShaderAction = shaderMenu->addAction("Basic");
		// auto* litShaderAction = shaderMenu->addAction("Lit");
		// auto* pbrShaderAction = shaderMenu->addAction("PBR");
		// basicShaderAction->setCheckable(true);
		// litShaderAction->setCheckable(true);
		// pbrShaderAction->setCheckable(true);
		// auto* shaderGroup = new QActionGroup(this);
		// shaderGroup->setExclusive(true);
		// shaderGroup->addAction(basicShaderAction);
		// shaderGroup->addAction(litShaderAction);
		// shaderGroup->addAction(pbrShaderAction);
		// pbrShaderAction->setChecked(true);
		// // BASIC
		// connect(basicShaderAction, &QAction::triggered, this, [this]() {
		// 	_sim->currentShaderMode = gui::SimulationManager::ShaderMode::Basic;
		// 	LOG_INFO("Shader Mode -> Basic");
		// });
		// // LIT
		// connect(litShaderAction, &QAction::triggered, this, [this]() {
		// 	_sim->currentShaderMode = gui::SimulationManager::ShaderMode::Lit;
		// 	LOG_INFO("Shader Mode -> Lit");
		// });
		// // PBR
		// connect(pbrShaderAction, &QAction::triggered, this, [this]() {
		// 	_sim->currentShaderMode = gui::SimulationManager::ShaderMode::PBR;
		// 	LOG_INFO("Shader Mode -> PBR");
		// });

		// shaderMenu->addSeparator();

		// auto* reloadShadersAction = shaderMenu->addAction("Reload Shaders");
		// connect(reloadShadersAction, &QAction::triggered, this, [this]() {
		// 	LOG_INFO("Menu clicked: Reload Shaders");
		// 	_sim->reloadAllShaders();
		// });
	}

	void DSFE_MainWindow::buildSceneMenu(QMenu* sceneMenu) {
		auto* toggleGridAction = sceneMenu->addAction("Toggle Grid");
		toggleGridAction->setCheckable(true);
		//toggleGridAction->setChecked(_sim->isGridEnabled());
		connect(toggleGridAction, &QAction::toggled, this, [this](bool checked) {
			LOG_INFO("Menu toggled: Scene -> Toggle Grid -> %s", checked ? "On" : "Off");
			//_sim->enableGrid(checked);
		});

		auto* toggleFloorAction = sceneMenu->addAction("Toggle Floor");
		toggleFloorAction->setCheckable(true);
		//toggleFloorAction->setChecked(_sim->isFloorEnabled());
		connect(toggleFloorAction, &QAction::toggled, this, [this](bool checked) {
			LOG_INFO("Menu toggled: Scene -> Toggle Floor -> %s", checked ? "On" : "Off");
			//_sim->enableFloor(checked);
		});

		auto* toggleSkyboxAction = sceneMenu->addAction("Toggle Skybox");
		toggleSkyboxAction->setCheckable(true);
		//toggleSkyboxAction->setChecked(_sim->isSkyboxEnabled());
		connect(toggleSkyboxAction, &QAction::toggled, this, [this](bool checked) {
			LOG_INFO("Menu toggled: Scene -> Toggle Skybox -> %s", checked ? "On" : "Off");
			//_sim->enableSkybox(checked);
		});

		auto* toggleOrientatorAction = sceneMenu->addAction("Toggle Orientator");
		toggleOrientatorAction->setCheckable(true);
		//toggleOrientatorAction->setChecked(_sim->isOrientastorEnabled());
		connect(toggleOrientatorAction, &QAction::toggled, this, [this](bool checked) {
			LOG_INFO("Menu toggled: Scene -> Toggle Orientator -> %s", checked ? "On" : "Off");
			//_sim->enableOrientator(checked);
		});
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
				//_sim->loadRobot(robotName.toStdString());
			});
		}
	}

} // namespace window