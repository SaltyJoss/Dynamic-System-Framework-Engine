//DSFE_GUI DSFE_MainWindow.cpp
#include "MainWindow/DSFE_MainWindow.h"
#include "Workspace/ProjectPage.h"

namespace window {
	DSFE_MainWindow::DSFE_MainWindow(QWidget* parent) : QMainWindow(parent) {
		setWindowTitle("DSFE");
		resize(800, 600);

		auto* page = new Workspace::ProjectPage(this);
		setCentralWidget(page);
		page->show();
	}
} // namespace window