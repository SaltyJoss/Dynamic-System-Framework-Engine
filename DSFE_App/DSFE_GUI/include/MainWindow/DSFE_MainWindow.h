// DSFE_GUI DSFE_MainWindow.h
#pragma once

#include <QMainWindow>

namespace gui { class SimManager; }

namespace window {
	class DSFE_MainWindow : public QMainWindow {
	public:
		explicit DSFE_MainWindow(gui::SimManager* sim, QWidget* parent = nullptr);

	private:
		void buildMenuBar();
		gui::SimManager* _sim;
	};
}
