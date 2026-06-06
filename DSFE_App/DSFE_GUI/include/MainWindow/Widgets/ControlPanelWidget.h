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
	class Mesh;
	class Object;
	class Light;
	class Camera;
}
namespace render {
	enum class ResolutionPreset;
	enum class QualityPreset;
}

namespace robots { class RobotSystem; }
namespace diagnostics { class TelemetryRecorder; }

namespace gui { class SimManager; }

class QVBoxLayout;
class QCheckBox;
class QComboBox;
class QGroupBox;
class QLabel;
class QSlider;

namespace widgets {
	class ControlPanelWidget : public QWidget {
	public:
		explicit ControlPanelWidget(gui::SimManager* sim, QWidget* parent = nullptr);
	private:
		struct IntegratorEntry {
			integration::eIntegrationMethod method;
			const char* name;
		};

		struct ADIntegratorEntry {
			integration::eAutoDiffIntegrationMethod method;
			const char* name;
		};

		void simPropertiesPanel();
		void buildIntegratorCombos();
		void buildDtFractions();

		void jointInfoPanel();
		void displayJointInfo(const diagnostics::TelemetryRecorder& rec, int& selectedJoint, QVBoxLayout* layout);

		void displayPanel();
		void selectJointAndFollow(int jointIdx);

		gui::SimManager* _sim = nullptr;

		QVBoxLayout* _contentLayout = nullptr;
		QGroupBox* _simPropertiesGroup = nullptr;
		QCheckBox* _useAutoDiffCheck = nullptr;
		QComboBox* _integratorCombo = nullptr;
		QLabel* _currentIntegratorLabel = nullptr;

		QGroupBox* _jointInfoGroup = nullptr;
		QSlider* _jointIdxSlider = nullptr;
		QLabel* _currentSimTimeJointLabel = nullptr;


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
		bool _hasRobot = false;
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

		// Time tracking for simulation updates
		std::chrono::high_resolution_clock::time_point simLastUpdateTime = std::chrono::high_resolution_clock::now();
		std::chrono::high_resolution_clock::time_point diagLastUpdateTime = std::chrono::high_resolution_clock::now();
	};
} // namespace widgets