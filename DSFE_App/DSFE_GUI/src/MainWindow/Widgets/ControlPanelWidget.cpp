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
#include <QTimer>

#include <cmath>

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

		auto* timer = new QTimer(this);
		connect(timer, &QTimer::timeout, this, [this]() {
			updateSimClock();
			jointInfoPanel();
			updateTelemetryDisplay();
		});
		timer->start(7);

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
		_simTimeLabel = new QLabel();
		_simTimeLabel->setTextFormat(Qt::RichText);
		_simTimeLabel->setWordWrap(true);
		layout->addWidget(new QLabel("<b>Simulation Time:</b>"));
		layout->addWidget(_simTimeLabel);

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
		if (!_jointInfoGroup) {
			_jointInfoGroup = new QGroupBox("Robot Joint Information");
			auto* layout = new QVBoxLayout(_jointInfoGroup);
			_jointInfoGroup->setLayout(layout);
			_currentSimTimeJointLabel = new QLabel(_jointInfoGroup);
			layout->addWidget(_currentSimTimeJointLabel);

			_jointIdxSlider = new QSlider(Qt::Horizontal, _jointInfoGroup);
			_jointIdxSlider->setMinimum(1);
			_jointIdxSlider->setValue(1);
			connect(_jointIdxSlider, &QSlider::valueChanged, this, [this](int value) { selectJointAndFollow(value - 1); });
			layout->addWidget(_jointIdxSlider);

			buildTelemetryWidgets(layout);

			_contentLayout->addWidget(_jointInfoGroup);
		}

		if (!_sim || !_sim->hasRobot()) {
			_jointInfoGroup->setVisible(false);
			return;
		}
		robots::RobotSystem* robot = _sim->robotSystem();
		if (!robot) {
			_jointInfoGroup->setVisible(false);
			return;
		}
		auto& joints = robot->joints();
		auto& links = robot->links();

		_jointInfoGroup->setVisible(!joints.empty() && !links.empty());
		if (joints.empty() || links.empty()) { _jointInfoGroup->setVisible(false); return; }
		_jointInfoGroup->setVisible(true);
		_jointIdxSlider->setMaximum(static_cast<int>(joints.size()));

		static int currentJointIndex = 0;
		currentJointIndex = std::clamp(currentJointIndex, 0, (int)joints.size() - 1);
	}

	void ControlPanelWidget::updateTelemetryInfo(const diagnostics::JointTelemetry& j) {
		auto& t = _telemetryLabels;
		const float e = static_cast<float>(j.q_ref - j.q);

		t.q->setText(QString("pos:\t%1 rad").arg(j.q));
		t.qd->setText(QString("vel:\t%1 rad/s").arg(j.qd));
		t.tau->setText(QString("torque:\t%1 Nm").arg(j.torqueNm));

		t.qRef->setText(QString("pos_ref:\t%1 rad").arg(j.q_ref));
		t.qdRef->setText(QString("vel_ref:\t%1 rad/s").arg(j.qd_ref));
		t.qddRef->setText(QString("acc_ref:\t%1 rad/s²").arg(j.qdd_ref));
		t.err->setText(QString("error:\t%1 rad").arg(e));

		t.qTraj->setText(QString("pos_traj:\t%1 rad").arg(j.traj_q));
		t.qdTraj->setText(QString("vel_traj:\t%1 rad/s").arg(j.traj_qd));
		t.qddTraj->setText(QString("acc_traj:\t%1 rad/s²").arg(j.traj_qdd));

		t.qClamped->setText(QString("pos_clamped:\t%1").arg(j.clampTheta ? "true" : "false"));
		t.qdClamped->setText(QString("vel_clamped:\t%1").arg(j.clampOmega ? "true" : "false"));

		t.damping->setText(QString("damping:\t%1 kg·m²/s").arg(j.damping));
		t.friction->setText(QString("friction:\t%1 N·m").arg(j.friction));
	}

	void ControlPanelWidget::buildTelemetryWidgets(QVBoxLayout* layout) {
		auto headerFont = [](QLabel* label) {
			QFont font = label->font();
			font.setBold(true);
			font.setPointSize(font.pointSize() + 2);
			label->setFont(font);
		};

		auto& t = _telemetryLabels;

		t.stateHeader = new QLabel("State:");
		t.referenceHeader = new QLabel("Reference:");
		t.trajectoryHeader = new QLabel("Trajectory:");
		t.clampedHeader = new QLabel("Clamped:");
		t.constantsHeader = new QLabel("Constants:");

		headerFont(t.stateHeader);
		headerFont(t.referenceHeader);
		headerFont(t.trajectoryHeader);
		headerFont(t.clampedHeader);
		headerFont(t.constantsHeader);

		t.q = new QLabel();
		t.qd= new QLabel();
		t.tau = new QLabel();

		t.qRef = new QLabel();
		t.qdRef = new QLabel();
		t.qddRef = new QLabel();
		t.err = new QLabel();

		t.qTraj = new QLabel();
		t.qdTraj = new QLabel();
		t.qddTraj = new QLabel();

		t.qClamped = new QLabel();
		t.qdClamped = new QLabel();

		t.damping = new QLabel();
		t.friction = new QLabel();

		layout->addSpacing(5);

		layout->addWidget(t.stateHeader);
		layout->addWidget(t.q);
		layout->addWidget(t.qd);
		layout->addWidget(t.tau);

		layout->addSpacing(5);

		layout->addWidget(t.referenceHeader);
		layout->addWidget(t.qRef);
		layout->addWidget(t.qdRef);
		layout->addWidget(t.qddRef);
		layout->addWidget(t.err);

		layout->addSpacing(5);

		layout->addWidget(t.trajectoryHeader);
		layout->addWidget(t.qTraj);
		layout->addWidget(t.qdTraj);
		layout->addWidget(t.qddTraj);

		layout->addSpacing(5);

		layout->addWidget(t.clampedHeader);
		layout->addWidget(t.qClamped);
		layout->addWidget(t.qdClamped);

		layout->addSpacing(5);

		layout->addWidget(t.constantsHeader);
		layout->addWidget(t.damping);
		layout->addWidget(t.friction);
	}

	void ControlPanelWidget::updateTelemetryDisplay() {
		if (!_sim) { return; }
		const auto& rec = _sim->telemetry();
		const auto& ring = rec.ring;
		if (ring.size() < 1) { return; }
		const auto& s = ring.at(ring.size() - 1);
		if (s.j.empty()) { return; }
		int jointIdx = _selection.type == SelectionType::JOINT ? _selection.index : 0;
		jointIdx = std::clamp(jointIdx, 0, static_cast<int>(s.j.size()) - 1);
		if (_jointIdxSlider) {
			QSignalBlocker blocker(_jointIdxSlider);
			_jointIdxSlider->setMaximum(static_cast<int>(s.j.size()));
			_jointIdxSlider->setValue(jointIdx + 1);
		}
		updateTelemetryInfo(s.j[jointIdx]);
		if (_currentSimTimeJointLabel) { _currentSimTimeJointLabel->setText(QString("Current Simulation Time: %1 s").arg(simTime, 0, 'f', 3)); }
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

	void ControlPanelWidget::updateSimClock() {
		if (!_sim || !_simTimeLabel) { return; }

		const double elapsed = _sim->simTime();
		const bool running = _sim->isSimRunning();

		if (!running && elapsed <= 0.0) {
			_simTimeLabel->clear();
			_simTimeLabel->setVisible(false);
			return;
		}

		_simTimeLabel->setVisible(true);
		_simTimeLabel->setTextFormat(Qt::RichText);
		_simTimeLabel->setWordWrap(true);

		if (_sim->isSimRunning()) {
			simTime = _sim->simTime();
			_simTimeLabel->setText(QString("<b>Elapsed Time:</b> %1 s").arg(_sim->simTime(), 0, 'f', 3));
		}
		else {
			_simTimeLabel->setText(QString("<b>Elapsed Time:</b> %1 s").arg(_sim->simTime(), 0, 'f', 3));
		}
	}
} // namespace widgets