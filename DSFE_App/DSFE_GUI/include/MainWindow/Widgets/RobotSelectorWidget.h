// DSFE_GUI RobotSelectorWidget.h
#pragma once

#include <QWidget>

namespace gui {class SimManager; }
class QPushButton;
namespace widgets {
	class RobotSelectorWidget : public QWidget {
	public:
		explicit RobotSelectorWidget(gui::SimManager* sim, QWidget* parent = nullptr);
	private:
		gui::SimManager* _sim;
		void addRobotButton(const QString& robotName, QString company);
	};
} // namespace widgets