// DSFE_GUI ControlPanelWidget.cpp
#include "Widgets/ControlPanelWidget.h"

#include <QVBoxLayout>
#include <QScrollArea>
#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>

#include "Scene/Mesh.h"
#include "Scene/Object.h"
#include "Scene/Light.h"
#include "Scene/Camera.h"

#include "Scene/SimulationManager.h"
#include "Robots/RobotSystem.h"

#include "Analysis/Telemetry.h"
#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"

namespace widgets {
	ControlPanelWidget::ControlPanelWidget(gui::SimManager* sim, QWidget* parent)
		: QWidget(parent), _sim(sim)
	{
		auto* rootLayout = new QVBoxLayout(this);
		rootLayout->setContentsMargins(4, 4, 4, 4);
		auto* scrollArea = new QScrollArea(this);
		scrollArea->setWidgetResizable(true);
		auto* content = new QWidget(scrollArea);
		_contentLayout = new QVBoxLayout(content);
		content->setLayout(_contentLayout);
		scrollArea->setWidget(content);
		rootLayout->addWidget(scrollArea);

		simPropertiesPanel();

		_contentLayout->addStretch();
	}

	// SimSetupPanel for 
	void ControlPanelWidget::simPropertiesPanel() {
		auto* robot = _sim->robotSystem();
		_simPropertiesGroup = new QGroupBox("Simulation Properties");
		auto* layout = new QVBoxLayout(_simPropertiesGroup);
		_useAutoDiffCheck = new QCheckBox("Enable Automatic Differentiable Integrators");
		layout->addWidget(_useAutoDiffCheck);
		_integratorCombo = new QComboBox();
		layout->addWidget(_integratorCombo);
		_contentLayout->addWidget(_simPropertiesGroup);

		buildIntegratorCombos();
		connect(_useAutoDiffCheck, &QCheckBox::toggled, this, [this, robot](bool checked) {
			_useAutoDiff = checked;
			robot->enableAutoDiff(checked);
			buildIntegratorCombos();
		});

		connect(_integratorCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
			if (_useAutoDiff) {
				auto selectedMethod = static_cast<integration::eAutoDiffIntegrationMethod>(_integratorCombo->currentData().toInt());
				_sim->setADIntegrationMethod(selectedMethod);
			}
			else {
				auto selectedMethod = static_cast<integration::eIntegrationMethod>(_integratorCombo->currentData().toInt());
				_sim->setIntegrationMethod(selectedMethod);
			}
		});
	}

	void ControlPanelWidget::buildIntegratorCombos() {
		QSignalBlocker blocker(_integratorCombo);
		_integratorCombo->clear();
		if (_useAutoDiff) {
			for (const auto& entry : adIntegrators) {
				_integratorCombo->addItem(entry.name, static_cast<int>(entry.method));
			}
			auto currentMethod = _sim->autoDiffIntegrationMethod();
			int index = _integratorCombo->findData(static_cast<int>(currentMethod));
			if (index != -1) {
				_integratorCombo->setCurrentIndex(index);
			}
		}
		else {
			for (const auto& entry : integrators) {
				_integratorCombo->addItem(entry.name, static_cast<int>(entry.method));
			}
			auto currentMethod = _sim->integrationMethod();
			int index = _integratorCombo->findData(static_cast<int>(currentMethod));
			if (index != -1) {
				_integratorCombo->setCurrentIndex(index);
			}
		}
	}

	void ControlPanelWidget::jointInfoPanel() {
	}

	void ControlPanelWidget::displayPanel() {
	}

	void ControlPanelWidget::selectJointAndFollow(int jointIdx) {
		if (!_sim || !_sim->hasRobot()) { return; }

		robots::RobotSystem* robot = _sim->robotSystem();
		if (!robot) { return; }

		auto& joints = robot->joints();
		auto& links = robot->links();
		if (joints.empty()) { return; }

		jointIdx = std::clamp(jointIdx, 0, (int)joints.size() - 1);

		const auto& joint = joints[jointIdx];
		_currentJointName = joint.name;

		_selection.type = SelectionType::JOINT;
		_selection.index = jointIdx;
		_selection.source = SelectionSource::CONTROL_PANEL;

		_sim->followRobotJoint(_currentJointName, glm::vec3(0.0f, 0.2f, 0.6f));
	}
} // namespace widgets