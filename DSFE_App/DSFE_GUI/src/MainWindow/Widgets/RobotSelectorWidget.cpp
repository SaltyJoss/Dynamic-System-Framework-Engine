// DSFE_GUI RobotSelectorWidget.cpp
#include "Widgets/RobotSelectorWidget.h"
#include "Scene/SimulationManager.h"

#include <unordered_map>
#include "Platform/SystemMap.h"

#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

namespace widgets {
	RobotSelectorWidget::RobotSelectorWidget(gui::SimManager* sim, QWidget* parent) : QWidget(parent), _sim(sim) {
		setWindowTitle("Choose Robotic Arm");
		setMinimumWidth(300);
		auto* layout = new QVBoxLayout(this);
		if (!_sim) {
			layout->addWidget(new QLabel("No simulation manager available", this));
			return;
		}

		const std::unordered_map<platform::eRoboticArms, platform::eRoboticArmFamilies>& robotMap = platform::getRoboticArmMap();

		if (robotMap.empty()) {
			layout->addWidget(new QLabel("No robotic arms available", this));
			return;
		}

		for (const auto& [arm, family] : robotMap) {
			const QString robotName = QString::fromStdString(platform::RoboticArms().toString(arm));
			const QString familyName = QString::fromStdString(platform::RoboticArms().toString(family));
			addRobotButton(robotName, familyName);
		}
	}
	void RobotSelectorWidget::addRobotButton(const QString& robotName, QString company) {
		auto* button = new QPushButton(robotName + "\n" + company);
		layout()->addWidget(button);
		connect(button, &QPushButton::clicked, this, [this, robotName]() {
			LOG_INFO("Selected robot: %s", robotName.toStdString().c_str());
			if (_sim) { _sim->loadRobot(robotName.toStdString()); }
		});
	}
} // namespace widgets