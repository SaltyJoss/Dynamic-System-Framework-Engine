// DSFE_GUI ControlPanelWidget.cpp
#include "Widgets/ControlPanelWidget.h"

#include <QVBoxLayout>
#include <QScrollArea>
#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QLabel>
#include <QSlider>
#include <QSignalBlocker>
#include <QFont>

#include "Widgets/FractionSelectorWidget.h"

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
		jointInfoPanel();

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

		_currentIntegratorLabel = new QLabel();
		layout->addWidget(_currentIntegratorLabel);

		layout->addSpacing(5);

		_simDtSelector = new FractionSelectorWidget(false);
		layout->addWidget(new QLabel("Simulation Time Step (dt):"));
		layout->addWidget(_simDtSelector);
		_telemetryDtSelector = new FractionSelectorWidget(true);
		layout->addWidget(new QLabel("Telemetry Time Step (dt):"));
		layout->addWidget(_telemetryDtSelector);

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
				_currentIntegratorLabel->setWordWrap(true);
				_currentIntegratorLabel->setText(QString("Current Integrator: ") + _integratorCombo->currentText());
			}
			else {
				auto selectedMethod = static_cast<integration::eIntegrationMethod>(_integratorCombo->currentData().toInt());
				_sim->setIntegrationMethod(selectedMethod);
				_currentIntegratorLabel->setWordWrap(true);
				_currentIntegratorLabel->setText(QString("Current Integrator: ") + _integratorCombo->currentText());
			}
		});

		connect(_simDtSelector, &FractionSelectorWidget::valueChanged, this, [this](double dt) { _sim->setFixedDt(dt); });
		connect(_telemetryDtSelector, &FractionSelectorWidget::valueChanged, this, [this](double dt) { _sim->setTelemetryHz(1.0/dt); });
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
		_jointInfoGroup = new QGroupBox("Robot Joint Information");
		auto* layout = new QVBoxLayout(_jointInfoGroup);
		_contentLayout->addWidget(_jointInfoGroup);

		if (!_sim->hasRobot()) {
			layout->addWidget(new QLabel("No Robot Loaded."));
			return;
		}
		robots::RobotSystem* robot = _sim->robotSystem();
		if (!robot) {
			layout->addWidget(new QLabel("Robotic system unavailable"));
			return;
		}
		auto& joints = robot->joints();
		auto& links = robot->links();


		_jointIdxSlider = new QSlider(Qt::Orientation::Horizontal);
		layout->addWidget(_jointIdxSlider);

		static int currentJointIndex = 0;
		currentJointIndex = std::clamp(currentJointIndex, 0, (int)joints.size() - 1);

		auto& j = joints[currentJointIndex];
		int linkIndex = currentJointIndex;
		linkIndex = std::clamp(linkIndex, 0, (int)links.size() - 1);
		auto& l = links[linkIndex];

		const auto& rec = _sim->telemetry();
		displayJointInfo(rec, currentJointIndex, layout);
		layout->addSpacing(5);
		layout->addWidget(new QLabel("Selected Joint: " + QString::fromStdString(j.name) + " - Child Link: " + QString::fromStdString(l.name)));
	}

	void ControlPanelWidget::displayJointInfo(const diagnostics::TelemetryRecorder& rec, int& selectedJoint, QVBoxLayout* layout) {
		const auto& ring = rec.ring;
		if (ring.size() < 1) { return; }
		const diagnostics::TelemetrySample& s = ring.at(ring.size() - 1); // Get the most recent sample
		if (selectedJoint < 0) { selectedJoint = 0; }
		if (selectedJoint >= (int)s.j.size()) { selectedJoint = (int)s.j.size() - 1; }

		_currentSimTimeJointLabel = new QLabel(QString("Current Simulation Time: ") + QString::number(s.timeSec) + " s");
		layout->addWidget(_currentSimTimeJointLabel);

		int currentJ = selectedJoint + 1;
		if (!_jointIdxSlider) {
			selectedJoint = currentJ - 1;
			
			_jointIdxSlider->setMinimum(1);
			_jointIdxSlider->setMaximum((int)s.j.size());
			_jointIdxSlider->setValue(currentJ);
			connect(_jointIdxSlider, &QSlider::valueChanged, this, [this](int value) {
				int jointIdx = value - 1;
				selectJointAndFollow(jointIdx);
			});
		}
		else {
			selectedJoint = currentJ - 1;
			_jointIdxSlider->setMaximum((int)s.j.size());
			_jointIdxSlider->setValue(currentJ);
		}

		const diagnostics::JointTelemetry& j = s.j[selectedJoint];
		const float e = static_cast<float>(j.q_ref - j.q);

		layout->addSpacing(10);

		auto headerFont = [](QLabel* label) {
			QFont font = label->font();
			font.setBold(true);
			font.setPointSize(font.pointSize() + 2);
			label->setFont(font);
		};

		
		auto* stateLabel = new QLabel(QString("State: "));
		headerFont(stateLabel);
		auto* refLabel = new QLabel(QString("Reference: "));
		headerFont(refLabel);
		auto* trajLabel = new QLabel(QString("Trajectory: "));
		headerFont(trajLabel);
		auto* clampLabel = new QLabel(QString("Clamped: "));
		headerFont(clampLabel);
		auto* constLabel = new QLabel(QString("Constants: "));
		headerFont(constLabel);

		// Joint State Telemetry
		layout->addWidget(stateLabel);
		layout->addWidget(new QLabel(QString("pos:	") + QString::number(j.q) + " rad"));
		layout->addWidget(new QLabel(QString("vel:	") + QString::number(j.qd) + " rad/s"));
		layout->addWidget(new QLabel(QString("torque: ") + QString::number(j.torqueNm) + " Nm"));

		layout->addSpacing(5);

		// Joint Reference Telemetry
		layout->addWidget(refLabel);
		layout->addWidget(new QLabel(QString("pos_ref: ") + QString::number(j.q_ref) + " rad"));
		layout->addWidget(new QLabel(QString("vel_ref: ") + QString::number(j.qd_ref) + " rad/s"));
		layout->addWidget(new QLabel(QString("acc_ref: ") + QString::number(j.qdd_ref) + " rad/s²"));
		layout->addWidget(new QLabel(QString("error:	") + QString::number(e) + " rad"));

		layout->addSpacing(5);

		// Joint Trajectory Telemetry
		layout->addWidget(trajLabel);
		layout->addWidget(new QLabel(QString("pos_traj: ") + QString::number(j.traj_q) + " rad"));
		layout->addWidget(new QLabel(QString("vel_traj: ") + QString::number(j.traj_qd) + " rad/s"));
		layout->addWidget(new QLabel(QString("acc_traj: ") + QString::number(j.traj_qdd) + " rad/s²"));

		layout->addSpacing(5);

		// Joint Clamping Telemetry
		layout->addWidget(clampLabel);
		layout->addWidget(new QLabel(QString("pos_clamped: ") + QString(j.clampTheta ? "true" : "false")));
		layout->addWidget(new QLabel(QString("vel_clamped: ") + QString(j.clampOmega ? "true" : "false")));

		layout->addSpacing(5);

		// Joint Constants Telemetry
		layout->addWidget(constLabel);
		layout->addWidget(new QLabel(QString("damping: ") + QString::number(j.damping) + " kg·m²/s"));
		layout->addWidget(new QLabel(QString("friction: ") + QString::number(j.friction) + " N·m"));
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