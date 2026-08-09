/*
 * File: DSFE_GUI/src/MainWindow/Widgets/RobotSelectorWidget.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include <QWidget>

namespace gui {class SimulationManager; }
class QPushButton;
namespace widgets {
	class RobotSelectorWidget : public QWidget {
	public:
		explicit RobotSelectorWidget(gui::SimulationManager* sim, QWidget* parent = nullptr);
	private:
		gui::SimulationManager* _sim;
		void addRobotButton(const QString& robotName, QString company);
	};
} // namespace widgets