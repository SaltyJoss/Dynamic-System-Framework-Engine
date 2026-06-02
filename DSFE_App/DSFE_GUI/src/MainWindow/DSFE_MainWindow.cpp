//DSFE_GUI DSFE_MainWindow.cpp
#include "MainWindow/DSFE_MainWindow.h"

namespace window {
	DSFE_MainWindow::DSFE_MainWindow(QWidget* parent) : QMainWindow(parent) {
		setWindowTitle("DSFE Simulator");
		resize(800, 600);
	}
}