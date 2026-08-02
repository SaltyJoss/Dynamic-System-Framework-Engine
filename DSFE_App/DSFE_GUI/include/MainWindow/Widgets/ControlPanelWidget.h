// DSFE_GUI ControlPanelWidget.h
#pragma once

#include <QWidget>
#include <future>
#include <atomic>
#include <mutex>
#include <cmath>
#include <unordered_map>
#include <chrono>

#include "Platform/SimulationState.h"
#include "Numerics/IntegrationMethods.h"
#include "Platform/Logger.h"

namespace scene {
	class Camera;
}
namespace render {
	enum class ResolutionPreset;
	enum class QualityPreset;
}

namespace systems { class RigidBodySystem;}
namespace diagnostics { class TelemetryRecorder; struct JointTelemetry; struct FreeBodyTelemetry; }
namespace gui { class SimulationManager; }

class QVBoxLayout;
class QCheckBox;
class QComboBox;
class QGroupBox;
class QLabel;
class QPushButton;
class QSlider;
class QTreeWidget;
class QTreeWidgetItem;

namespace widgets {
	class FractionSelectorWidget;
	class GravityVectorWidget;

	class ControlPanelWidget : public QWidget {
	public:
		explicit ControlPanelWidget(gui::SimulationManager* sim, QWidget* parent = nullptr);
		void refreshFromSim();
		void refreshSelectorTree();

	private:
		struct IntegratorEntry {
			integration::eIntegrationMethod method;
			const char* name;
		};

		struct ADIntegratorEntry {
			integration::eAutoDiffIntegrationMethod method;
			const char* name;
		};

		// Telemetry display for joint information
		struct JointTelemetryLabels {
			// Headers
			QLabel* stateHeader = nullptr;
			QLabel* referenceHeader = nullptr;
			QLabel* trajectoryHeader = nullptr;
			QLabel* clampedHeader = nullptr;
			QLabel* constantsHeader = nullptr;

			// State
			QLabel* q = nullptr;
			QLabel* qd = nullptr;
			QLabel* tau = nullptr;

			// Reference
			QLabel* qRef = nullptr;
			QLabel* qdRef = nullptr;
			QLabel* qddRef = nullptr;
			QLabel* err = nullptr;

			// Trajectory
			QLabel* qTraj = nullptr;
			QLabel* qdTraj = nullptr;
			QLabel* qddTraj = nullptr;

			// Clamping
			QLabel* qClamped = nullptr;
			QLabel* qdClamped = nullptr;

			// Constants
			QLabel* damping = nullptr;
			QLabel* friction = nullptr;
		};
		// Telemetry display for free bodies (not joints)
		struct FreeBodyTelemetryLabels {
			// Headers
			QLabel* stateHeader = nullptr;
			QLabel* timeDerivativeHeader = nullptr;
			QLabel* energyPerformanceHeader = nullptr;
			QLabel* sleepStateHeader = nullptr;
			QLabel* massInertialHeader = nullptr;

			// State
			QLabel* position = nullptr;
			QLabel* orientation = nullptr;
			QLabel* linearVelocity = nullptr;
			QLabel* angularVelocity = nullptr;

			// Time Derivatives
			QLabel* linearAcceleration = nullptr;
			QLabel* angularAcceleration = nullptr;
			QLabel* netAccumulatedForce = nullptr;
			QLabel* netAccumulatedTorque = nullptr;
			
			// Energy & Performance
			QLabel* KE = nullptr;
			QLabel* PE = nullptr;
			QLabel* linearMomentum = nullptr;
			QLabel* angularMomentum = nullptr;

			// Sleep State
			QLabel* sleepState = nullptr;

			// Mass & Inertial Properties
			QLabel* mass = nullptr;
			QLabel* inverse_mass = nullptr;
			QLabel* inertia = nullptr;
			QLabel* inverse_inertia = nullptr;
		};

		// Selection state for the control panel
		QTreeWidget* _selectorTree = nullptr;
		void selectorTreePanel();
		void onSelectorItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* prev);

		// Selection state for the control panel
		void simPropertiesPanel();
		void buildIntegratorCombos();
		void buildTimestepSelectors(QVBoxLayout* layout);
		void worldPropertiesPanel();
		// Telemetry display
		void telemetryInfoPanels();
		// Joint telemetry display
		void jointInfoPanel();
		void updateJointTelemetryInfo(const diagnostics::JointTelemetry& j);
		void buildJointTelemetryWidgets(QVBoxLayout* layout);
		void updateJointTelemetryDisplay();
		// Free Body telemetry display
		void freeBodyInfoPanel();
		void updateFreeBodyTelemetryInfo(const diagnostics::FreeBodyTelemetry& fb);
		void buildFreeBodyTelemetryWidgets(QVBoxLayout* layout);
		void updateFreeBodyTelemetryDisplay();

		void selectJointAndFollow(int jointIdx);

		void updateSimClock();

		QLabel* _jointChainLabel = nullptr;
		QLabel* _jointIndexLabel = nullptr;
		void refreshJointChainLabel(int idx);

		QLabel* _freeBodyLabel = nullptr;
		QLabel* _freeBodyIndexLabel = nullptr;
		void refreshFreeBodyLabel(int idx);

		gui::SimulationManager* _sim = nullptr;

		QVBoxLayout* _contentLayout = nullptr;
		QGroupBox* _simPropertiesGroup = nullptr;
		QCheckBox* _useAutoDiffCheck = nullptr;
		QComboBox* _integratorCombo = nullptr;
		QLabel* _currentIntegratorLabel = nullptr;
		QLabel* _simTimeLabel = nullptr;

		QLabel* _simDtLabel = nullptr;
		FractionSelectorWidget* _simDtSelector = nullptr;
		QLabel* _simDtValue = nullptr;
		
		QLabel* _telDtLabel = nullptr;
		FractionSelectorWidget* _telDtSelector = nullptr;
		QLabel* _telDtValue = nullptr;

		QGroupBox* _worldPropertiesGroup = nullptr;

		QLabel* _gravityLabel = nullptr;
		GravityVectorWidget* _grav = nullptr;

		QGroupBox* _jointInfoGroup = nullptr;
		QSlider* _jointIdxSlider = nullptr;

		QGroupBox* _freeBodyInfoGroup = nullptr;


		static constexpr IntegratorEntry integrators[] = {
			{ integration::eIntegrationMethod::Euler, "Euler" },
			{ integration::eIntegrationMethod::Midpoint, "Midpoint" },
			{ integration::eIntegrationMethod::Heun, "Heun" },
			{ integration::eIntegrationMethod::Ralston, "Ralston" },
			{ integration::eIntegrationMethod::RK4, "RK4" },
			{ integration::eIntegrationMethod::RK45, "RK45" },
			{ integration::eIntegrationMethod::ImplicitEuler, "Implicit Euler" },
			{ integration::eIntegrationMethod::ImplicitMidpoint, "Implicit Midpoint" },
			{ integration::eIntegrationMethod::GLRK2, "GLRK2" },
			{ integration::eIntegrationMethod::GLRK3, "GLRK3" }
		};

		static constexpr ADIntegratorEntry adIntegrators[] = {
			{ integration::eAutoDiffIntegrationMethod::AD_ImplicitEuler, "Implicit Euler (AutoDiff)" },
			{ integration::eAutoDiffIntegrationMethod::AD_ImplicitMidpoint, "Implicit Midpoint (AutoDiff)" },
			{ integration::eAutoDiffIntegrationMethod::AD_GLRK2, "GLRK2 (AutoDiff)" },
			{ integration::eAutoDiffIntegrationMethod::AD_GLRK3, "GLRK3 (AutoDiff)" }
		};

		JointTelemetryLabels _jointTelLabels;
		FreeBodyTelemetryLabels _freeBodyTelLabels;

		// Current selection state
		Selection _selection;

		// Current items selected
		std::string _requestedRobot;    // name of requested robot to load
		std::string _currentObjectName; // name of currently selected object
		std::string _currentLinkName;   // name of currently selected link
		std::string _currentJointName;  // name of currently selected joint
		std::string _lastLinkName;      // name of last selected link

		// Storage of joint angles
		std::unordered_map<std::string, float> _linkAngles;

		// Internal states
		bool simulationRunning = false;
		bool _jointSelected = false;
		bool diagRunning = false;
		bool _robotRequested = false;
		bool _hasBody = false;
		bool _openStats = true;
		bool _useAutoDiff = false;
		// Simulation and diagnostics timing
		float simLength = 30.0f;  // ~30 seconds default
		float diagLength = 15.0f; // ~15 seconds default
		double deltaTime = 1.0f / 180.0f; // ~180 FPS default
		float simTime = 0.0f;     // current simulation time
		float diagTime = 0.0f;    // current diagnostic time
		// Camera properties
		int povMode = 0;
		float fov = 60.0f;
		// Internal Physics
		float velocity = 0.0f;
		float torque = 0.0f;
		float linkLength = 1.0f;
		float damping = 0.1f;
		float position = 0.0f;

		int _lastTreeJointCount = -1;

		// Time tracking for simulation updates
		std::chrono::high_resolution_clock::time_point simLastUpdateTime = std::chrono::high_resolution_clock::now();
		std::chrono::high_resolution_clock::time_point diagLastUpdateTime = std::chrono::high_resolution_clock::now();
	};
} // namespace widgets