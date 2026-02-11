#pragma once
// File:    ControlPanel.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include <cmath>
#include "Physics/PhysicsSystem.h"

#include "Scene/Object.h"
#include "Analysis/Telemetry.h"
#include "Scene/SimulationManager.h"
#include "Scene/Light.h"
#include "Platform/Logger.h"
#include "Platform/SimulationState.h"
#include <unordered_map>

#include <imgui.h>
#include "Platform/imguiWidgets.h"
#include <imfilebrowser.h>

namespace gui {
	// Gravity UI Modes
    enum class GravityUIMode {
        Preset,
        SolarSystem,
        Custom
    };

	// Gravity Presets (in m/s^2)
    enum GravityPreset {
		// Common Presets
        PRESET_ZERO_G,
        PRESET_MICRO_G,
        PRESET_SOLAR_SYSTEM,
        PRESET_CUSTOM,
		// Solar System Bodies
        PRESET_SUN,
        PRESET_MERCURY,
        PRESET_VENUS,
        PRESET_EARTH,
        PRESET_MARS,
        PRESET_JUPITER,
        PRESET_SATURN,
        PRESET_URANUS,
        PRESET_NEPTUNE,
        PRESET_PLUTO,
		// Moons
        PRESET_MOON,
        PRESET_TITAN,
        PRESET_ENCELADUS,
        PRESET_EUROPA,
        PRESET_GANYMEDE,
        PRESET_IO
    };

	// UI Navigation Levels for Gravity Presets
    enum class GravityLevel {
        Root,
        SolarSystem,
        Planets,
        Moons
    };

    inline GravityLevel gravityLevel = GravityLevel::Root;
    inline GravityUIMode gravityMode = GravityUIMode::Preset;

	// ControlPanel Class
    class ENGINE_API ControlPanel {
    public:
        ControlPanel(SimManager* sim);

		void drawMenus(SimManager* sim);
        void render(SimManager* sim);
        void setSimulationCallback(const std::function<void(bool)>& callback) { simCallback = callback; }
        void setMeshLoadCallback(const std::function<void(const std::string&)>& callback) { meshLoadCallback = callback; }

    private:
		// Internal Pointers
        std::shared_ptr<scene::Mesh> _mesh;

        SimManager* _sim = nullptr;
        physics::PhysicsSystem* _phys;
        scene::Light* _light;
        scene::Object* _obj;
        ImGui::FileBrowser _meshLoad;
        ImGui::FileBrowser _hdrLoad;
        std::string _currentMeshFile;
        std::string _currentHDRFile;

        SimManager::ControlMode* _controlMode;

        std::function<void(const std::string&)> meshLoadCallback;
        std::function<void(bool)> simCallback;

        Selection _selection;

		// Internal Methods
        void simulationProperties();
        void objectProperties();
        void jointProperties();
        void displaySettings();

		void tempLightControls();

        void roboticArmSelector();
        void roboticCardDisplay(const char* name, const char* company);

        void sceneObjectsTable();

        void selectJointAndFollow(int jointIndex);
		void drawTelemetryPlots(const diagnostics::TelemetryRecorder& rec);
        void drawTrajectoryInspector(const diagnostics::TelemetryRecorder& rec, int jointCount, int& selectedJoint);


		// Helper Methods
        void beginControlPanel(const char* id, ImVec2 size = ImVec2(0, 0));
		void endControlPanel();
  
        // Menu Button States
        bool _showRobotSelector = false;
		bool _showSimulationProperties = false;
		bool _showCameraProperties = false;
		bool _showDisplaySettings = false;
        bool autoScroll = true;
        bool scrollToBottom = false;

		// Render Presets
		render::ResolutionPreset r = render::ResolutionPreset::R_4K;
        render::QualityPreset q = render::QualityPreset::Ultra;
        
		bool _qualityChanged = false;
		bool _resChanged = false;

        // Internal states
        bool simulationRunning = false;
		bool _jointSelected = false;
		bool diagRunning = false;
        bool _robotRequested = false;
        bool _hasRobot = false;
        bool _openStats = true;

		std::string _requestedRobot;    // name of requested robot to load
		std::string _currentObjectName; // name of currently selected object
		std::string _currentLinkName;   // name of currently selected link
		std::string _currentJointName;  // name of currently selected joint
		std::string _lastLinkName;      // name of last selected link

        std::unordered_map<std::string, float> _linkAngles;

		float simLength = 30.0f;  // ~30 seconds default
		float diagLength = 15.0f; // ~15 seconds default
        float deltaTime = 1 / 180; // ~180 FPS default
		float simTime = 0.0f;     // current simulation time
		float diagTime = 0.0f;    // current diagnostic time

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
} // namespace gui

// Helper Macros

#define INDENT()   ImGui::Indent(20.0f)
#define UNINDENT() ImGui::Unindent(20.0f)