#pragma once

//=============================================
//            File: ControlPanel.h
//=============================================
// GUI Control Panel for interacting with the simManager.
//
// ============================================
//              GitHub: SaltyJoss
// ============================================

// Includes
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
	// ControlPanel Class
    class ENGINE_API ControlPanel {
    public:
        ControlPanel(simManager* sim);

		void drawMenus(simManager* sim);
        void render(simManager* sim);
        void setSimulationCallback(const std::function<void(bool)>& callback) { simCallback = callback; }
        void setMeshLoadCallback(const std::function<void(const std::string&)>& callback) { meshLoadCallback = callback; }

    private:
		// Internal Pointers
        std::shared_ptr<scene::Mesh> _mesh;

        simManager* _sim = nullptr;
        physics::PhysicsSystem* _phys;
        scene::Light* _light;
        scene::Object* _obj;
        ImGui::FileBrowser _meshLoad;
        ImGui::FileBrowser _hdrLoad;
        std::string _currentMeshFile;
        std::string _currentHDRFile;

        simManager::ControlMode* _controlMode;

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
}