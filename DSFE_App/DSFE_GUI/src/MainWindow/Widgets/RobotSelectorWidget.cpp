/*
 * File: DSFE_GUI/src/MainWindow/Widgets/RobotSelectorWidget.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "Widgets/RobotSelectorWidget.h"
#include "Simulation/SimulationManager.h"

#include "Platform/SystemMap.h"

#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>

namespace widgets {
	RobotSelectorWidget::RobotSelectorWidget(gui::SimulationManager* sim, QWidget* parent) : QWidget(parent), _sim(sim) {
		setWindowTitle("Choose Robotic Arm");
		setMinimumWidth(300);
		auto* layout = new QVBoxLayout(this);
		if (!_sim) {
			layout->addWidget(new QLabel("No simulation manager available", this));
			return;
		}

		const std::unordered_map<platform::eRoboticSystems, platform::eRoboticSystemFamilies>& robotMap = platform::getRobotSystemMap();

		if (robotMap.empty()) {
			layout->addWidget(new QLabel("No robotic arms available", this));
			return;
		}

		for (const auto& [sys, family] : robotMap) {
			const QString robotName = QString::fromStdString(platform::RoboticSystems().toString(sys));
			const QString familyName = QString::fromStdString(platform::RoboticSystems().toString(family));
			addRobotButton(robotName, familyName);
		}
	}
	void RobotSelectorWidget::addRobotButton(const QString& robotName, QString company) {
		auto* button = new QPushButton(robotName + "\n" + company);
		layout()->addWidget(button);
		connect(button, &QPushButton::clicked, this, [this, robotName]() {
			std::string n = robotName.toStdString();
			std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c){ return std::tolower(c); });
			const std::string path = "rigidbody_models/" + n + "/" + n + ".urdf";
			LOG_INFO("Selected robot: %s -> %s", n.c_str(), path.c_str());
			if (_sim) { _sim->load_rigidBody(path); }
		});
	}
} // namespace widgets