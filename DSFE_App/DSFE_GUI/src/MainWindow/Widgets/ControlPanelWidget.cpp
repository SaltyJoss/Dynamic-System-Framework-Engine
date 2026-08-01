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
		freeBodyInfoPanel();

		auto* timer = new QTimer(this);
		connect(timer, &QTimer::timeout, this, [this]() {
			updateSimClock();
			jointInfoPanel();
			freeBodyInfoPanel();
			updateJointTelemetryDisplay();
			updateFreeBodyTelemetryDisplay();
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

		_gravityLabel = new QLabel(this);
		_gravityLabel->setTextFormat(Qt::RichText);
		_gravityLabel->setText("<b><u>Gravity</u></b>");
		_gravityLabel->setAlignment(Qt::AlignLeft);
		_grav = new GravityVectorWidget(_worldPropertiesGroup);
		_grav->setValue(_sim->gravity());
		_grav->onChanged = [this](const glm::vec3& g) { _sim->setGravity(g); };

		layout->addWidget(_gravityLabel);
		layout->addWidget(_grav);
		
		layout->addSpacing(8);

		_contentLayout->addWidget(_worldPropertiesGroup);
	}

	/*
	 * Joint Telemetry Display
	 */
	// Builds the joint telemetry display widgets and adds them to the given layout.
	void ControlPanelWidget::jointInfoPanel() {
		if (!_jointInfoGroup) {
			_jointInfoGroup = new QGroupBox("RigidBody Joint Information");
			auto* layout = new QVBoxLayout(_jointInfoGroup);
			_jointInfoGroup->setLayout(layout);

			// --- Joint chain context: parent  ->  [joint]  ->  child ---
			_jointChainLabel = new QLabel();
			_jointChainLabel->setTextFormat(Qt::RichText);
			_jointChainLabel->setAlignment(Qt::AlignCenter);
			_jointChainLabel->setWordWrap(true);
			_jointChainLabel->setStyleSheet("color: rgb(200,205,215);");
			layout->addWidget(_jointChainLabel);

			// --- Index readout: "joint  3 / 6" ---
			_jointIndexLabel = new QLabel();
			_jointIndexLabel->setTextFormat(Qt::RichText);
			_jointIndexLabel->setAlignment(Qt::AlignCenter);
			layout->addWidget(_jointIndexLabel);

			layout->addSpacing(4);

			_jointIdxSlider = new QSlider(Qt::Horizontal, _jointInfoGroup);
			_jointIdxSlider->setMinimum(1);
			_jointIdxSlider->setValue(1);
			_jointIdxSlider->setTickPosition(QSlider::TicksBelow);
			_jointIdxSlider->setTickInterval(1);
			_jointIdxSlider->setSingleStep(1);
			_jointIdxSlider->setPageStep(1);

			connect(_jointIdxSlider, &QSlider::valueChanged, this, [this](int value) {
				selectJointAndFollow(value - 1); refreshJointChainLabel(value - 1);
			});
			layout->addWidget(_jointIdxSlider);
			layout->addSpacing(6);
			buildJointTelemetryWidgets(layout);
			_contentLayout->addWidget(_jointInfoGroup);
		}
		if (!_sim || _sim->isFreeBody() || !_sim->hasRigidBody()) { _jointInfoGroup->setVisible(false); return; }
		auto& body = _sim->rigidBodySystem();
		auto& joints = body.joints();
		auto& links = body.links();
		if (joints.empty() || links.empty()) { _jointInfoGroup->setVisible(false); return; }

		_jointInfoGroup->setVisible(true);
		_jointIdxSlider->setMaximum(static_cast<int>(joints.size()));

		static int currentJointIndex = 0;
		currentJointIndex = std::clamp(currentJointIndex, 0, (int)joints.size() - 1);
		refreshJointChainLabel(_jointIdxSlider->value() - 1);
	}
	// Renders "parent  ->  [ Joint_n ]  ->  child" and the "n / total" index readout for the given joint.
	void ControlPanelWidget::refreshJointChainLabel(int idx) {
		if (!_sim || !_sim->hasRigidBody()) { return; }
		auto& body = _sim->rigidBodySystem();
		auto& joints = body.joints();
		if (joints.empty()) { return; }
		idx = std::clamp(idx, 0, (int)joints.size() - 1);
		const auto& j = joints[idx];
		const QString parent = QString::fromStdString(j.parent);
		const QString child  = QString::fromStdString(j.child);
		const QString name   = QString::fromStdString(j.name);
		// parent link (dim) -> joint (bright, italic-ish) -> child link (dim)
		_jointChainLabel->setText(QString(
			"<span style='color:#8a8f9a'>%1</span>"
			"  <span style='color:#5a6070'>\u27F6</span>  "
			"<span style='color:#d8dbe2; font-weight:600'>[ %2 ]</span>"
			"  <span style='color:#5a6070'>\u27F6</span>  "
			"<span style='color:#8a8f9a'>%3</span>")
			.arg(parent, name, child)
		);
		// "joint  n / total" with the count in a dim weight
		_jointIndexLabel->setText(QString(
			"<span style='color:#8a8f9a; font-size:9pt'>joint</span> "
			"<span style='color:#e0e3ea; font-family:Consolas; font-weight:600'>%1</span>"
			"<span style='color:#5a6070'> / </span>"
			"<span style='color:#8a8f9a; font-family:Consolas'>%2</span>")
			.arg(idx + 1).arg(joints.size())
		);
	}
	// Updates the joint telemetry display with the latest data from the simulation.
	void ControlPanelWidget::updateJointTelemetryInfo(const diagnostics::JointTelemetry& j) {
		auto& t = _jointTelLabels;
		const double e = j.q_ref - j.q;

		// fixed-width numeric formatting so columns don't jitter as values change
		auto num = [](double v, const char* unit) {
			return QString("%1 <span style='color:#888'>%2</span>").arg(v, 0, 'f', 4).arg(unit);
		};
		// State Telemetry
		t.q->setText(num(j.q, "rad"));
		t.qd->setText(num(j.qd, "rad\u00B7s\u207B\u00B9"));
		t.tau->setText(num(j.torqueNm, "N\u00B7m"));
		// Reference Telemetry
		t.qRef->setText(num(j.q_ref, "rad"));
		t.qdRef->setText(num(j.qd_ref, "rad\u00B7s\u207B\u00B9"));
		t.qddRef->setText(num(j.qdd_ref, "rad\u00B7s\u207B\u00B2"));
		t.err->setText(num(e, "rad"));
		// Trajectory Telemetry
		t.qTraj->setText(num(j.traj_q, "rad"));
		t.qdTraj->setText(num(j.traj_qd, "rad\u00B7s\u207B\u00B9"));
		t.qddTraj->setText(num(j.traj_qdd, "rad\u00B7s\u207B\u00B2"));
		// Clamping Telemetry
		t.qClamped->setText(j.clampTheta
			? "<span style='color:#569cd6'>active</span>"
			: "<span style='color:#666'>\u2014</span>");
		t.qdClamped->setText(j.clampOmega
			? "<span style='color:#569cd6'>active</span>"
			: "<span style='color:#666'>\u2014</span>");
		// Constants Telemetry
		t.damping->setText(num(j.damping, "kg\u00B7m\u00B2\u00B7s\u207B\u00B9"));
		t.friction->setText(num(j.friction, "N\u00B7m"));
	}
	// Builds the joint telemetry display widgets and adds them to the given layout.
	void ControlPanelWidget::buildJointTelemetryWidgets(QVBoxLayout* layout) {
		auto headerFont = [](QLabel* label) {
			QFont font = label->font();
			font.setBold(true);
			font.setPointSize(font.pointSize() + 1);
			font.setLetterSpacing(QFont::PercentageSpacing, 115);  // tracked-out caps read as section headers
			label->setFont(font);
			label->setStyleSheet("color: rgb(200,205,215);");
		};

		// A math-symbol row label: rich-text italic variable, e.g. "θ" or "θ_ref".
		auto symLabel = [](const QString& html) {
			auto* l = new QLabel(html);
			l->setTextFormat(Qt::RichText);
			l->setObjectName("telem_symbol");
			return l;
		};

		// A value label: monospace, right-aligned so digits column up.
		auto valueLabel = [](QLabel* l) {
			l->setTextFormat(Qt::RichText);
			l->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
			QFont f("Consolas");            // or "JetBrains Mono"/"Cascadia Mono" if bundled
			f.setStyleHint(QFont::Monospace);
			f.setPointSize(l->font().pointSize());
			l->setFont(f);
			l->setStyleSheet("color: rgb(225,228,235);");
			return l;
		};
		auto& t = _jointTelLabels;

		t.stateHeader      = new QLabel("STATE");
		t.referenceHeader  = new QLabel("REFERENCE");
		t.trajectoryHeader = new QLabel("TRAJECTORY");
		t.clampedHeader    = new QLabel("LIMITS");
		t.constantsHeader  = new QLabel("PHYSICAL");
		for (QLabel* h : {
				t.stateHeader, 
				t.referenceHeader, 
				t.trajectoryHeader,
				t.clampedHeader,
				t.constantsHeader
			}
		) {
			headerFont(h);
		}

		t.q = new QLabel(); t.qd = new QLabel(); t.tau = new QLabel();
		t.qRef = new QLabel(); t.qdRef = new QLabel(); t.qddRef = new QLabel(); t.err = new QLabel();
		t.qTraj = new QLabel(); t.qdTraj = new QLabel(); t.qddTraj = new QLabel();
		t.qClamped = new QLabel(); t.qdClamped = new QLabel();
		t.damping = new QLabel(); t.friction = new QLabel();
		for (QLabel* v : {
				t.q, t.qd, t.tau, t.err,
				t.qRef, t.qdRef, t.qddRef, 
				t.qTraj, t.qdTraj, t.qddTraj,
				t.qClamped, t.qdClamped, 
				t.damping, t.friction
			}
		) {
				valueLabel(v);
		}

		auto makeGrid = [](std::initializer_list<std::pair<QLabel*, QLabel*>> rows) {
			auto* g = new QGridLayout(); int r = 0;
			for (auto& [sym, val] : rows) {
				g->addWidget(sym, r, 0, Qt::AlignLeft | Qt::AlignVCenter);
				g->addWidget(val, r, 1);
				++r;
			}
			g->setHorizontalSpacing(14);
			g->setVerticalSpacing(3);
			g->setColumnStretch(0, 0);
			g->setColumnStretch(1, 1);
			return g;
		};

		// θ (theta), ω (omega), τ (tau); subscripts for ref/traj; Δ for error.
		auto* stateGrid = makeGrid({
			{ symLabel("<i>\u03B8</i>"),   t.q   },   // θ  position
			{ symLabel("<i>\u03C9</i>"),   t.qd  },   // ω  velocity
			{ symLabel("<i>\u03C4</i>"),   t.tau },   // τ  torque
		});

		auto* refGrid = makeGrid({
			{ symLabel("<i>\u03B8</i><sub>ref</sub>"),  t.qRef   },
			{ symLabel("<i>\u03C9</i><sub>ref</sub>"),  t.qdRef  },
			{ symLabel("<i>\u03B1</i><sub>ref</sub>"),  t.qddRef },   // α  target accel
			{ symLabel("\u0394<i>\u03B8</i>"),          t.err    },   // Δθ error
		});

		auto* trajGrid = makeGrid({
			{ symLabel("<i>\u03B8</i><sub>traj</sub>"), t.qTraj   },
			{ symLabel("<i>\u03C9</i><sub>traj</sub>"), t.qdTraj  },
			{ symLabel("<i>\u03B1</i><sub>traj</sub>"), t.qddTraj },
		});

		auto* limitGrid = makeGrid({
			{ symLabel("<i>\u03B8</i> clamp"), t.qClamped  },
			{ symLabel("<i>\u03C9</i> clamp"), t.qdClamped },
		});

		auto* physicalGrid = makeGrid({
			{ symLabel("<i>b</i> <span style='color:#888'>damping</span>"),   t.damping  },
			{ symLabel("<i>c</i> <span style='color:#888'>friction</span>"),  t.friction },
		});

		auto addSection = [layout](QLabel* header, QGridLayout* grid, int gap) {
			layout->addSpacing(gap);
			layout->addWidget(header);
			layout->addLayout(grid);
		};

		addSection(t.stateHeader,      stateGrid,    5);
		addSection(t.referenceHeader,  refGrid,      10);
		addSection(t.trajectoryHeader, trajGrid,     10);
		addSection(t.clampedHeader,    limitGrid,    10);
		addSection(t.constantsHeader,  physicalGrid, 10);
	}
	// Updates the joint telemetry display with the latest data from the simulation.
	void ControlPanelWidget::updateJointTelemetryDisplay() {
		if (!_sim || _sim->isFreeBody()) { return; }
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
		updateJointTelemetryInfo(s.j[jointIdx]);
	}

	/*
	 * FreeBody Telemetry Display
	 */
	// Builds the free body telemetry display widgets and adds them to the given layout.
	void ControlPanelWidget::freeBodyInfoPanel() {
		if (!_freeBodyInfoGroup) {
			_freeBodyInfoGroup = new QGroupBox("RigidBody Free Body Information");
			auto* layout = new QVBoxLayout(_freeBodyInfoGroup);
			_freeBodyInfoGroup->setLayout(layout);

			// --- Free body context: parent  ->  [joint]  ->  child ---
			_freeBodyLabel = new QLabel();
			_freeBodyLabel->setTextFormat(Qt::RichText);
			_freeBodyLabel->setAlignment(Qt::AlignCenter);
			_freeBodyLabel->setWordWrap(true);
			_freeBodyLabel->setStyleSheet("color: rgb(200,205,215);");
			layout->addWidget(_freeBodyLabel);

			// --- Index readout: "joint  3 / 6" ---
			_freeBodyIndexLabel = new QLabel();
			_freeBodyIndexLabel->setTextFormat(Qt::RichText);
			_freeBodyIndexLabel->setAlignment(Qt::AlignCenter);
			layout->addWidget(_freeBodyIndexLabel);

			layout->addSpacing(4);

			buildFreeBodyTelemetryWidgets(layout);
			_contentLayout->addWidget(_freeBodyInfoGroup);
		}
		if (!_sim || !_sim->hasRigidBody() || !_sim->isFreeBody()) { _freeBodyInfoGroup->setVisible(false); return; }
		auto& body = _sim->rigidBodySystem();
		auto& joints = body.joints();
		auto& links = body.links();
		if (links.size() != 1 || joints.size() != 1) { _freeBodyInfoGroup->setVisible(false); return; }
		_freeBodyInfoGroup->setVisible(true);
		refreshFreeBodyLabel(0);
	}
	// Renders "parent  ->  [ name ]  ->  child" and the "n / total" index readout for the given free body.
	void ControlPanelWidget::refreshFreeBodyLabel(int idx) {
		if (!_sim) { return; }
		auto& body = _sim->rigidBodySystem();
		auto& joints = body.joints();
		if (joints.empty()) { return; }
		idx = std::clamp(idx, 0, (int)joints.size() - 1);
		const auto& j = joints[idx];
		const QString parent = QString::fromStdString(j.parent);
		const QString child  = QString::fromStdString(j.child);
		const QString name   = QString::fromStdString(j.name);
		// parent link (dim) -> joint (bright, italic-ish) -> child link (dim)
		_freeBodyLabel->setText(QString(
			"<span style='color:#8a8f9a'>%1</span>"
			"  <span style='color:#5a6070'>\u27F6</span>  "
			"<span style='color:#d8dbe2; font-weight:600'>[ %2 ]</span>"
			"  <span style='color:#5a6070'>\u27F6</span>  "
			"<span style='color:#8a8f9a'>%3</span>")
			.arg(parent, name, child)
		);
		// "joint  n / total" with the count in a dim weight
		_freeBodyIndexLabel->setText(QString(
			"<span style='color:#8a8f9a; font-size:9pt'>joint</span> "
			"<span style='color:#e0e3ea; font-family:Consolas; font-weight:600'>%1</span>"
			"<span style='color:#5a6070'> / </span>"
			"<span style='color:#8a8f9a; font-family:Consolas'>%2</span>")
			.arg(idx + 1).arg(joints.size())
		);
	}
	// Updates the free body telemetry display with the latest data from the simulation.
	void ControlPanelWidget::updateFreeBodyTelemetryInfo(const systems::RigidBodySystem& body) {
		auto& t = _freeBodyTelLabels;

		// fixed-width numeric formatting so columns don't jitter as values change
		auto num = [](double v, const char* unit) {
			return QString("%1 <span style='color:#888'>%2</span>").arg(v, 0, 'f', 4).arg(unit);
		};
		// State Telemetry
		t.position->setText(num(0.0, "m"));  // Placeholder for actual position
		t.orientation->setText(num(0.0, "rad"));  // Placeholder for actual orientation
		t.linearVelocity->setText(num(0.0, "m\u00B7s\u207B\u00B9"));  // Placeholder for actual linear velocity
		t.angularAcceleration->setText(num(0.0, "rad\u00B7s\u207B\u00B2"));
		// Time Derivative Telemetry
		t.linearAcceleration->setText(num(0.0, "m\u00B7s\u207B\u00B2"));
		t.angularAcceleration->setText(num(0.0, "rad\u00B7s\u207B\u00B2"));
		t.netAccumulatedForce->setText(num(0.0, "N"));
		t.netAccumulatedTorque->setText(num(0.0, "N\u00B7m"));
		// Energy & Performance Telemetry
		t.KE->setText(num(0.0, "J"));
		t.PE->setText(num(0.0, "J"));
		t.linearMomentum->setText(num(0.0, "kg\u00B7m\u00B7s\u207B\u00B9"));
		t.angularMomentum->setText(num(0.0, "kg\u00B7m\u00B2\u00B7s\u207B\u00B9"));
		// Sleep State Telemetry
		t.sleepState->setText(false
			? "<span style='color:#569cd6'>active</span>"
			: "<span style='color:#666'>\u2014</span>");
		// Mass & Inertia Telemetry
		t.mass->setText(num(0.0, "kg"));
		t.inverse_mass->setText(num(0.0, "kg\u207B\u00B9"));
		t.inertia->setText(num(0.0, "kg\u00B7m\u00B2"));
		t.inverse_inertia->setText(num(0.0, "(kg\u00B7m\u00B2)\u207B\u00B9"));
	}
	// Builds the free body telemetry display widgets and adds them to the given layout.
	void ControlPanelWidget::buildFreeBodyTelemetryWidgets(QVBoxLayout* layout) {
		auto headerFont = [](QLabel* label) {
			QFont font = label->font();
			font.setBold(true);
			font.setPointSize(font.pointSize() + 1);
			font.setLetterSpacing(QFont::PercentageSpacing, 115);  // tracked-out caps read as section headers
			label->setFont(font);
			label->setStyleSheet("color: rgb(200,205,215);");
		};

		// A math-symbol row label: rich-text italic variable, e.g. "θ" or "θ_ref".
		auto symLabel = [](const QString& html) {
			auto* l = new QLabel(html);
			l->setTextFormat(Qt::RichText);
			l->setObjectName("telem_symbol");
			return l;
		};

		// A value label: monospace, right-aligned so digits column up.
		auto valueLabel = [](QLabel* l) {
			l->setTextFormat(Qt::RichText);
			l->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
			QFont f("Consolas");            // or "JetBrains Mono"/"Cascadia Mono" if bundled
			f.setStyleHint(QFont::Monospace);
			f.setPointSize(l->font().pointSize());
			l->setFont(f);
			l->setStyleSheet("color: rgb(225,228,235);");
			return l;
		};
		auto& t = _freeBodyTelLabels;

		t.stateHeader      		  = new QLabel("STATE");
		t.timeDerivativeHeader    = new QLabel("TIME DERIVATIVE");
		t.energyPerformanceHeader = new QLabel("ENERGY & PERFORMANCE");
		t.sleepStateHeader 		  = new QLabel("SLEEP STATE");
		t.massInertialHeader  	  = new QLabel("MASS & INERTIA");
		for (QLabel* h : {
				t.stateHeader, 
				t.timeDerivativeHeader, 
				t.energyPerformanceHeader,
				t.sleepStateHeader,
				t.massInertialHeader
			}
		) {
			headerFont(h);
		}

		t.position = new QLabel(); t.orientation = new QLabel();
		t.linearVelocity = new QLabel(); t.angularVelocity = new QLabel();
		t.linearAcceleration = new QLabel(); t.angularAcceleration = new QLabel();
		t.netAccumulatedForce = new QLabel(); t.netAccumulatedTorque = new QLabel();
		t.KE = new QLabel(); t.PE = new QLabel();
		t.linearMomentum = new QLabel(); t.angularMomentum = new QLabel();
		t.sleepState = new QLabel(); 
		t.mass = new QLabel(); t.inverse_mass = new QLabel();
		t.inertia = new QLabel(); t.inverse_inertia = new QLabel();

		for (QLabel* v : {
				t.position, t.orientation,
				t.linearVelocity, t.angularVelocity,
				t.linearAcceleration, t.angularAcceleration,
				t.netAccumulatedForce, t.netAccumulatedTorque,
				t.KE, t.PE,
				t.linearMomentum, t.angularMomentum,
				t.sleepState,
				t.mass, t.inverse_mass,
				t.inertia, t.inverse_inertia
			}
		) {
				valueLabel(v);
		}

		auto makeGrid = [](std::initializer_list<std::pair<QLabel*, QLabel*>> rows) {
			auto* g = new QGridLayout(); int r = 0;
			for (auto& [sym, val] : rows) {
				g->addWidget(sym, r, 0, Qt::AlignLeft | Qt::AlignVCenter);
				g->addWidget(val, r, 1);
				++r;
			}
			g->setHorizontalSpacing(14);
			g->setVerticalSpacing(3);
			g->setColumnStretch(0, 0);
			g->setColumnStretch(1, 1);
			return g;
		};

		// θ (theta), ω (omega), τ (tau); subscripts for ref/traj; Δ for error.
		auto* stateGrid = makeGrid({
			{ symLabel("<i>\u03B8</i>"), t.position   },   	  // θ  position
			{ symLabel("\u0052"),   	 t.orientation  }, 	  // R  orientation
			{ symLabel("<i>\u03C4</i>"), t.linearVelocity },  // τ  linear velocity
			{ symLabel("<i>\u03C9</i>"), t.angularVelocity }, // α  angular velocity
		});

		auto* timeDerivGrid = makeGrid({
			{ symLabel("<i>\u03B1</i><sub>linear</sub>"),  t.linearAcceleration },
			{ symLabel("<i>\u03B1</i><sub>angular</sub>"), t.angularAcceleration },
			{ symLabel("\u0046<sub>net</sub>"),     t.netAccumulatedForce },
			{ symLabel("<i>\u03C4</i><sub>net</sub>"),     t.netAccumulatedTorque },
		});

		auto* energyPerformanceGrid = makeGrid({
			{ symLabel("\u004B\u0045"), t.KE },
			{ symLabel("\u0050\u0045"), t.PE },
			{ symLabel("<i>\u0070</i>"), t.linearMomentum },
			{ symLabel("\u004C"), t.angularMomentum },
		});

		auto* sleepStateGrid = makeGrid({
			{ symLabel("Sleep State"), t.sleepState  },
		});

		auto* massInertiaGrid = makeGrid({
			{ symLabel("<i>\u006D</i>"),   		 	 t.mass  },
			{ symLabel("<i>\u006D\u207B\u00B9</i>"), t.inverse_mass },
			{ symLabel("\u0049"),   		 	 t.inertia  },
			{ symLabel("\u0049\u207B\u00B9"), t.inverse_inertia },
		});

		auto addSection = [layout](QLabel* header, QGridLayout* grid, int gap) {
			layout->addSpacing(gap);
			layout->addWidget(header);
			layout->addLayout(grid);
		};

		addSection(t.stateHeader,      		  stateGrid,    		 5);
		addSection(t.timeDerivativeHeader,    timeDerivGrid,      	 10);
		addSection(t.energyPerformanceHeader, energyPerformanceGrid, 10);
		addSection(t.sleepStateHeader,    	  sleepStateGrid,    	 10);
		addSection(t.massInertialHeader,  	  massInertiaGrid, 	  	 10); 
	}
	// Updates the free body telemetry display with the latest data from the simulation.
	void ControlPanelWidget::updateFreeBodyTelemetryDisplay() {
		if (!_sim || !_sim->hasRigidBody()) { return; }
		const auto& body = _sim->rigidBodySystem();
		updateFreeBodyTelemetryInfo(body);
	}
	
	// --- Selection and Follow ---
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
		_grav->setValue(_sim->gravity());
		
	}
} // namespace widgets