//DSFE_GUI DSFE_MainWindow.cpp
#include "MainWindow/DSFE_MainWindow.h"
#include "Scene/SimulationManager.h"
#include "Workspace/ProjectPage.h"

namespace window {
	DSFE_MainWindow::DSFE_MainWindow(gui::SimManager* sim, QWidget* parent) : QMainWindow(parent) {
		setWindowTitle("DSFE");
		resize(1280, 720);

		auto* page = new Workspace::ProjectPage(sim, this);
		setCentralWidget(page);
	}

} // namespace window