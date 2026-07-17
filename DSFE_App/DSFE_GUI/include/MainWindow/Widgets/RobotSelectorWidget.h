// DSFE_GUI RobotSelectorWidget.h
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