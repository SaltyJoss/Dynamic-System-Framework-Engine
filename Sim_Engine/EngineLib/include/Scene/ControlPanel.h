#pragma once

//=============================================
//            File: ControlPanel.h
//=============================================
// GUI Control Panel for interacting with the simManager.
// 
// Summary:
// ============================================
// 
// public:
// --------------------------------------------
// ControlPanel(simManager* sceneView)
//      -> Constructor that initializes the ControlPanel with a reference to the simManager.
// render(simManager* sceneView)
//      -> Renders the control panel GUI scene.
// setSimulationCallback(const std::function<void(bool)>& callback)
//      -> Sets the callback function to be called when the simulation state changes.
// setMeshLoadCallback(const std::function<void(const std::string&)>& callback)
//      -> Sets the callback function to be called when a new mesh is loaded.
// --------------------------------------------
// 
// private:
// --------------------------------------------
// void simulationProperties()
//      -> Renders the simulation properties section of the control panel.
// void cameraProperties()
//      -> Renders the camera properties section of the control panel.
// void objectProperties()
// 	    -> Renders the object properties section of the control panel.
// void linkProperties()
//      -> Renders the link properties section of the control panel.
// void displaySettings()
//      -> Renders the display settings section of the control panel.
// void stats()
//      -> Renders the simulation statistics section of the control panel.
// void roboticArmSelector()
//      -> Renders the robotic arm selector section of the control panel.
// void roboticCardDisplay(const char* name, const char* company)
//      -> Renders a card for a robotic arm model in the selector.
// void sceneObjectsTable()
//      -> Renders the list of scene objects in the control panel.
// --------------------------------------------
// 
// Internal State Variables:
// --------------------------------------------
// bool simulationRunning
//      -> Indicates whether the simulation is currently running.
// bool _showRobotSelector
//      -> Indicates whether the robotic arm selector is visible.
// bool _robotRequested
// 	    -> Indicates whether a robotic arm has been requested for loading.
// bool _hasRobot
// 	    -> Indicates whether a robotic arm is currently loaded in the scene.
// std::string _requestedRobot
//      -> Stores the name of the requested robotic arm model.
// float simulationSpeed
//      -> Controls the speed of the simulation.
// int povMode
//      -> Controls the point-of-view mode of the camera.
// float velocity
//      -> Controls the velocity parameter for robotic arm links.
// float torque
//      -> Controls the torque parameter for robotic arm links.
// float linkLength
//      -> Controls the length parameter for robotic arm links.
// float damping
//      -> Controls the damping parameter for robotic arm links.
// float position
//      -> Controls the position parameter for robotic arm links
// double PI
// 	    -> Constant value for PI.
// std::shared_ptr<scene::Mesh> _mesh
//      -> Shared pointer to the mesh being manipulated.
// scene::Light* _sunLight
//      -> Pointer to the sun light in the scene.
// scene::Object* _obj
//      -> Pointer to the currently selected object in the scene.
// ImGui::FileBrowser _meshLoad
//      -> File browser for loading mesh files.
// ImGui::FileBrowser _hdrLoad
//      -> File browser for loading HDR environment files.
// std::string _currentMeshFile
//      -> Stores the name of the currently loaded mesh file.
// std::string _currentHDRFile
//      -> Stores the name of the currently loaded HDR file.
// std::function<void(const std::string&)> meshLoadCallback
// 	    -> Callback function to be called when a new mesh is loaded.
// std::function<void(bool)> simCallback
//      -> Callback function to be called when the simulation state changes.
// simManager* _sim
// 	    -> Pointer to the associated simManager.
// simManager::ControlMode* _controlMode
//      -> Pointer to the current control mode of the simManager.
// Selection _selection
//      -> Stores the current selection state in the control panel.
//--------------------------------------------
//
// ============================================
//              GitHub: SaltyJoss
// ============================================

// Includes
#include "EngineCore.h"
#include <cmath>

#include "Physics/PhysicsSystem.h"

#include "Scene/Object.h"
#include "Scene/SimulationManager.h"
#include "Scene/Light.h"
#include "Platform/Logger.h"
#include "Platform/SimulationState.h"
#include <unordered_map>

#include <imgui.h>
#include "Platform/imguiWidgets.h"
#include <imfilebrowser.h>

extern ENGINE_API Debug gLog;

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
        scene::Light* _sunLight;
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
        void cameraProperties();
        void objectProperties();
        void linkProperties();
        void displaySettings();
        void stats();

        void roboticArmSelector();
        void roboticCardDisplay(const char* name, const char* company);

        void sceneObjectsTable();
  
        // Menu Button States
        bool _showRobotSelector = false;
		bool _showSimulationProperties = false;
		bool _showCameraProperties = false;
		bool _showDisplaySettings = false;
        bool autoScroll = true;
        bool scrollToBottom = false;

		// Render Presets
        render::LookPreset l = render::LookPreset::Studio;
        render::QualityPreset q = render::QualityPreset::Medium;
        

        // Internal states
        bool simulationRunning = false;
		bool diagRunning = false;
        bool _robotRequested = false;
        bool _hasRobot = false;

        std::string _requestedRobot;
        std::string _currentObjectName;
        std::string _currentLinkName;
        std::string _lastLinkName;

        std::unordered_map<std::string, float> _linkAngles;

		float simLength = 30.0f;  // ~30 seconds default
		float diagLength = 15.0f; // ~15 seconds default
        float deltaTime = 1 / 120; // ~120 FPS default
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
        double finalTime = 0.0f;
		std::chrono::high_resolution_clock::time_point simLastUpdateTime = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point diagLastUpdateTime = std::chrono::high_resolution_clock::now();
    };
}