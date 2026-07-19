// DSFE_GUI DSFE_MainWindow.h
#pragma once

#include "GUIExports.h"

#include <QMainWindow>
#include <QMenu>

namespace gui { class SimulationManager; }
namespace widgets { class DSLEditorWidget; }

namespace render {
	enum class ResolutionPreset;
	enum class QualityPreset;
}

namespace window {
	class DSFE_MainWindow : public QMainWindow {
	public:
		explicit DSFE_MainWindow(gui::SimulationManager* sim, QWidget* parent = nullptr);

	private:
		void buildMenuBar();
		void buildGraphicsMenu(QMenu* graphicsMenu);
		void buildSceneMenu(QMenu* sceneMenu);
		void buildRobotMenu(QMenu* projectMenu);
		void onLoadMesh();

		render::ResolutionPreset r;
		render::QualityPreset q;

		gui::SimulationManager* _sim;
		widgets::DSLEditorWidget* _dslEditor = nullptr;
	};
}
