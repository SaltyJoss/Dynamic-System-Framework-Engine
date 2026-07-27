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
#include <QGridLayout>
#include <QFormLayout>
#include <QPushButton>

#include "Widgets/FractionSelectorWidget.h"
#include "Widgets/GravityVectorWidget.h"

#include "Scene/Camera.h"

#include "Simulation/SimulationManager.h"
#include "Systems/RigidBodySystem.h"

#include "Analysis/Telemetry.h"
#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"

namespace widgets {
	ControlPanelWidget::ControlPanelWidget(gui::SimulationManager* sim, QWidget* parent)
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
		worldPropertiesPanel();
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
		_simPropertiesGroup = new QGroupBox("Simulation Properties");
		auto* layout = new QVBoxLayout(_simPropertiesGroup);

		_useAutoDiffCheck = new QCheckBox();
		_useAutoDiff = _sim->autoDiffEnabled();
		_useAutoDiffCheck->setChecked(_useAutoDiff);
		_integratorCombo = new QComboBox();
		_integratorCombo->setMaximumWidth(175);

		_simDtSelector = new FractionSelectorWidget(false);
		_telDtSelector = new FractionSelectorWidget(true);
		_simDtLabel = new QLabel();
		_telDtLabel = new QLabel();

		_simTimeLabel = new QLabel();
		_simTimeLabel->setWordWrap(true);

		auto* form = new QFormLayout();

		form->addRow("Auto Diff", _useAutoDiffCheck);
		form->addRow("Integrator", _integratorCombo);
		layout->addLayout(form);

		layout->addSpacing(8);   

		buildTimestepSelectors(layout);

		layout->addSpacing(8);

		layout->addWidget(_simTimeLabel);
		_contentLayout->addWidget(_simPropertiesGroup);

		buildIntegratorCombos();

		connect(_useAutoDiffCheck, &QCheckBox::toggled, this, [this](bool checked) {
			_useAutoDiff = checked;
			_sim->enableAutoDiff(checked);
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

		connect(_simDtSelector, &FractionSelectorWidget::valueChanged, this, [this](double dt) { 
			_sim->setFixedDt(dt);
			_simDtValue = _simDtSelector->setDtVarDecValue(dt);
		});
		connect(_telDtSelector, &FractionSelectorWidget::valueChanged, this, [this](double dt) { 
			_sim->setTelemetryHz(1.0 / dt);
			_telDtValue = _telDtSelector->setDtVarDecValue(1.0/dt);
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
			if (index != -1) { _integratorCombo->setCurrentIndex(index); }
		}
		else {
			for (const auto& entry : integrators) {
				_integratorCombo->addItem(entry.name, static_cast<int>(entry.method));
			}
			auto currentMethod = _sim->integrationMethod();
			int index = _integratorCombo->findData(static_cast<int>(currentMethod));
			if (index != -1) { _integratorCombo->setCurrentIndex(index); }
		}
	}

	void ControlPanelWidget::buildTimestepSelectors(QVBoxLayout* layout) {
		_simDtLabel = _simDtSelector->setDtVarName("sim");
		_telDtLabel = _telDtSelector->setDtVarName("tel");
		
		double simDt = _sim->fixedDt();
		_simDtSelector->setDt(simDt);
		_simDtSelector->setFixedWidth(50);
		_simDtValue = _simDtSelector->setDtVarDecValue(simDt);

		double telDt = 1.0 / _sim->telemetryHz();
		_telDtSelector->setDt(telDt);
		_telDtSelector->setFixedWidth(50);
		_telDtValue = _telDtSelector->setDtVarDecValue(telDt);

		auto* dtGrid = new QGridLayout();
        dtGrid->setHorizontalSpacing(2);   // label hugs fraction
        dtGrid->setVerticalSpacing(8);

		dtGrid->setColumnStretch(0, 0);
        dtGrid->setColumnStretch(1, 0);
        dtGrid->setColumnStretch(2, 1);

        dtGrid->addWidget(_simDtLabel,    0, 0, Qt::AlignRight | Qt::AlignVCenter);
        dtGrid->addWidget(_simDtSelector, 0, 1, Qt::AlignLeft);
		dtGrid->addWidget(_simDtValue,    0, 2, Qt::AlignLeft  | Qt::AlignVCenter);
        dtGrid->addWidget(_telDtLabel, 	  1, 0, Qt::AlignRight | Qt::AlignVCenter);
        dtGrid->addWidget(_telDtSelector, 1, 1, Qt::AlignLeft);
		dtGrid->addWidget(_telDtValue,	  1, 2, Qt::AlignLeft  | Qt::AlignVCenter);

		dtGrid->setColumnStretch(0, 0);
        dtGrid->setColumnStretch(1, 0);
        dtGrid->setColumnStretch(2, 1);

		layout->addLayout(dtGrid);
	}

	void ControlPanelWidget::worldPropertiesPanel() {
		_worldPropertiesGroup = new QGroupBox("World Properties");
		auto* layout = new QVBoxLayout(_worldPropertiesGroup);
		_worldPropertiesGroup->setLayout(layout);

		auto* gravityLabel = new QLabel(this);
		gravityLabel->setTextFormat(Qt::RichText);
		gravityLabel->setText("<b><u>Gravity</u></b>");
		gravityLabel->setAlignment(Qt::AlignLeft);
		auto* grav = new GravityVectorWidget(_worldPropertiesGroup);
		grav->setValue(_sim->gravity());
		grav->onChanged = [this](const glm::vec3& g) { _sim->setGravity(g); };

		layout->addWidget(gravityLabel);
		layout->addWidget(grav);
		
		layout->addSpacing(8);

		_contentLayout->addWidget(_worldPropertiesGroup);
	}

	void ControlPanelWidget::jointInfoPanel() {
		if (!_jointInfoGroup) {
			_jointInfoGroup = new QGroupBox("RigidBody Joint Information");
			auto* layout = new QVBoxLayout(_jointInfoGroup);
			_jointInfoGroup->setLayout(layout);
			layout->addWidget(new QLabel("Joint"));

			_jointIdxSlider = new QSlider(Qt::Horizontal, _jointInfoGroup);
			_jointIdxSlider->setMinimum(1);
			_jointIdxSlider->setValue(1);

			connect(_jointIdxSlider, &QSlider::valueChanged, this, [this](int value) { selectJointAndFollow(value - 1); });

			layout->addWidget(_jointIdxSlider);

			buildTelemetryWidgets(layout);

			_contentLayout->addWidget(_jointInfoGroup);
		}

		if (!_sim || !_sim->hasRigidBody()) {
			_jointInfoGroup->setVisible(false);
			return;
		}
		auto& body = _sim->rigidBodySystem();
		auto& joints = body.joints();
		auto& links = body.links();

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

		t.q->setText(QString("%1 rad").arg(j.q));
		t.qd->setText(QString("%1 rad/s").arg(j.qd));
		t.tau->setText(QString("%1 Nm").arg(j.torqueNm));

		t.qRef->setText(QString("%1 rad").arg(j.q_ref));
		t.qdRef->setText(QString("%1 rad/s").arg(j.qd_ref));
		t.qddRef->setText(QString("%1 rad/s²").arg(j.qdd_ref));
		t.err->setText(QString("%1 rad").arg(e));

		t.qTraj->setText(QString("%1 rad").arg(j.traj_q));
		t.qdTraj->setText(QString("%1 rad/s").arg(j.traj_qd));
		t.qddTraj->setText(QString("%1 rad/s²").arg(j.traj_qdd));

		t.qClamped->setText(j.clampTheta ? "On" : "Off");
		t.qdClamped->setText(j.clampOmega ? "On" : "Off");

		t.damping->setText(QString("%1 kg·m²/s").arg(j.damping));
		t.friction->setText(QString("%1 N·m").arg(j.friction));
	}

	void ControlPanelWidget::buildTelemetryWidgets(QVBoxLayout* layout) {
		auto headerFont = [](QLabel* label) {
			QFont font = label->font();
			font.setBold(true);
			font.setPointSize(font.pointSize() + 1);
			label->setFont(font);
			label->setStyleSheet("color: rgb(220,220,220);");
		};

		auto& t = _telemetryLabels;

		t.stateHeader = new QLabel("STATE");
		t.referenceHeader = new QLabel("REFERENCE");
		t.trajectoryHeader = new QLabel("TRAJECTORY");
		t.clampedHeader = new QLabel("LIMITS");
		t.constantsHeader = new QLabel("PHYSICAL");

		headerFont(t.stateHeader);
		headerFont(t.referenceHeader);
		headerFont(t.trajectoryHeader);
		headerFont(t.clampedHeader);
		headerFont(t.constantsHeader);

		t.q = new QLabel();
		t.qd = new QLabel();
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

		auto* stateGrid = new QGridLayout();
		stateGrid->addWidget(new QLabel("Position"), 0, 0);
		stateGrid->addWidget(t.q, 0, 1);
		stateGrid->addWidget(new QLabel("Velocity"), 1, 0);
		stateGrid->addWidget(t.qd, 1, 1);
		stateGrid->addWidget(new QLabel("Torque"), 2, 0);
		stateGrid->addWidget(t.tau, 2, 1);

		stateGrid->setHorizontalSpacing(12);
		stateGrid->setColumnStretch(0, 0);
		stateGrid->setColumnStretch(1, 1);

		auto* refGrid = new QGridLayout();
		refGrid->addWidget(new QLabel("Target Pos"), 0, 0);
		refGrid->addWidget(t.qRef, 0, 1);
		refGrid->addWidget(new QLabel("Target Vel"), 1, 0);
		refGrid->addWidget(t.qdRef, 1, 1);
		refGrid->addWidget(new QLabel("Target Acc"), 2, 0);
		refGrid->addWidget(t.qddRef, 2, 1);
		refGrid->addWidget(new QLabel("Error"), 3, 0);
		refGrid->addWidget(t.err, 3, 1);

		refGrid->setHorizontalSpacing(12);
		refGrid->setColumnStretch(0, 0);
		refGrid->setColumnStretch(1, 1);

		auto* trajGrid = new QGridLayout();
		trajGrid->addWidget(new QLabel("Position"), 0, 0);
		trajGrid->addWidget(t.qTraj, 0, 1);
		trajGrid->addWidget(new QLabel("Velocity"), 1, 0);
		trajGrid->addWidget(t.qdTraj, 1, 1);
		trajGrid->addWidget(new QLabel("Acceleration"), 2, 0);
		trajGrid->addWidget(t.qddTraj, 2, 1);

		trajGrid->setHorizontalSpacing(12);
		trajGrid->setColumnStretch(0, 0);
		trajGrid->setColumnStretch(1, 1);

		auto* limitGrid = new QGridLayout();
		limitGrid->addWidget(new QLabel("Position Clamp"), 0, 0);
		limitGrid->addWidget(t.qClamped, 0, 1);
		limitGrid->addWidget(new QLabel("Velocity Clamp"), 1, 0);
		limitGrid->addWidget(t.qdClamped, 1, 1);

		limitGrid->setHorizontalSpacing(12);
		limitGrid->setColumnStretch(0, 0);
		limitGrid->setColumnStretch(1, 1);

		auto* physicalGrid = new QGridLayout();
		physicalGrid->addWidget(new QLabel("Damping"), 0, 0);
		physicalGrid->addWidget(t.damping, 0, 1);
		physicalGrid->addWidget(new QLabel("Friction"), 1, 0);
		physicalGrid->addWidget(t.friction, 1, 1);

		physicalGrid->setHorizontalSpacing(12);
		physicalGrid->setColumnStretch(0, 0);
		physicalGrid->setColumnStretch(1, 1);

		layout->addSpacing(5);
		layout->addWidget(t.stateHeader);
		layout->addLayout(stateGrid);

		layout->addSpacing(8);
		layout->addWidget(t.referenceHeader);
		layout->addLayout(refGrid);

		layout->addSpacing(8);
		layout->addWidget(t.trajectoryHeader);
		layout->addLayout(trajGrid);

		layout->addSpacing(8);
		layout->addWidget(t.clampedHeader);
		layout->addLayout(limitGrid);

		layout->addSpacing(8);
		layout->addWidget(t.constantsHeader);
		layout->addLayout(physicalGrid);
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
	}

	void ControlPanelWidget::displayPanel() {
	}

	void ControlPanelWidget::selectJointAndFollow(int jointIdx) {
		if (!_sim || !_sim->hasRigidBody()) { return; }

		auto& body = _sim->rigidBodySystem();
		auto& joints = body.joints();
		auto& links = body.links();
		if (joints.empty()) { return; }

		jointIdx = std::clamp(jointIdx, 0, (int)joints.size() - 1);

		const auto& joint = joints[jointIdx];
		_currentJointName = joint.name;

		_selection.type = SelectionType::JOINT;
		_selection.index = jointIdx;
		_selection.source = SelectionSource::CONTROL_PANEL;

		_sim->followRigidBodyJoint(_currentJointName, glm::vec3(0.0f, 0.2f, 0.6f));
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

	void ControlPanelWidget::refreshFromSim() {
		if (!_sim) { return; }
		_useAutoDiff = _sim->autoDiffEnabled();
		{
			QSignalBlocker b(_useAutoDiffCheck);
			_useAutoDiffCheck->setChecked(_useAutoDiff);
		}
		buildIntegratorCombos(); // already reads back from _sim
		double simDt = _sim->fixedDt();
		double telDt = 1.0 / _sim->telemetryHz();
		_simDtSelector->setDt(simDt);
		_simDtValue = _simDtSelector->setDtVarDecValue(simDt);
		_telDtSelector->setDt(telDt);
		_telDtValue = _telDtSelector->setDtVarDecValue(telDt);
	}
} // namespace widgets