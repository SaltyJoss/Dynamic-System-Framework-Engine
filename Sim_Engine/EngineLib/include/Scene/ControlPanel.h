#pragma once

//=============================================
//            File: ControlPanel.h
//=============================================
// GUI Control Panel for interacting with the SceneView.
// 
// Summary:
// ============================================
// 
// public:
// --------------------------------------------
// ControlPanel(SceneView* sceneView)
//      -> Constructor that initializes the ControlPanel with a reference to the SceneView.
// render(SceneView* sceneView)
//      -> Renders the control panel GUI elements.
// setSimulationCallback(const std::function<void(bool)>& callback)
//      -> Sets the callback function to be called when the simulation state changes.
// setMeshLoadCallback(const std::function<void(const std::string&)>& callback)
//      -> Sets the callback function to be called when a new mesh is loaded.
// --------------------------------------------
// 
// private:
// --------------------------------------------
// void renderSimulationProperties()
//      -> Renders the simulation properties section of the control panel.
// void renderCameraProperties()
//      -> Renders the camera properties section of the control panel.
// void renderObjectProperties()
// 	    -> Renders the object properties section of the control panel.
// void renderLinkProperties()
//      -> Renders the link properties section of the control panel.
// void renderDisplaySettings()
//      -> Renders the display settings section of the control panel.
// void renderStats()
//      -> Renders the simulation statistics section of the control panel.
// void renderRoboticSelector()
//      -> Renders the robotic arm selector section of the control panel.
// void renderRoboticCard(const char* name, const char* company)
//      -> Renders a card for a robotic arm model in the selector.
// void renderSceneObjects()
//      -> Renders the list of scene objects in the control panel.
// --------------------------------------------
// 
// Internal State Variables:
// --------------------------------------------
// bool simulationRunning
//      -> Indicates whether the simulation is currently running.
// bool gravityEnabled
//      -> Indicates whether gravity is enabled in the simulation.
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
// std::shared_ptr<elements::Mesh> _mesh
//      -> Shared pointer to the mesh being manipulated.
// elements::Light* _sunLight
//      -> Pointer to the sun light in the scene.
// elements::Object* _obj
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
// SceneView* _sceneView
// 	    -> Pointer to the associated SceneView.
// SceneView::ControlMode* _controlMode
//      -> Pointer to the current control mode of the SceneView.
// Selection _selection
//      -> Stores the current selection state in the control panel.
//--------------------------------------------
// 
// ============================================

// Includes
#include "EngineCore.h"
#include <MathLibAPI.h>
#include "const_phys.h"

#include "Scene/Object.h"
#include "Scene/SceneView.h"
#include "Scene/Light.h"
#include "Platform/Logger.h"
#include "Platform/SimulationState.h"

#include <imgui.h>
#include "Platform/imguiWidgets.h"
#include <imfilebrowser.h>

extern ENGINE_API Debug gLog;

namespace gui {
	// ControlPanel Class
    class ENGINE_API ControlPanel {
    public:
        ControlPanel(SceneView* sceneView);

        void render(SceneView* sceneView);
        void setSimulationCallback(const std::function<void(bool)>& callback) { simCallback = callback; }
        void setMeshLoadCallback(const std::function<void(const std::string&)>& callback) { meshLoadCallback = callback; }

    private:
        void renderSimulationProperties();
        void renderCameraProperties();
        void renderObjectProperties();
        void renderLinkProperties();
        void renderDisplaySettings();
        void renderStats();

        void renderRoboticSelector();
        void renderRoboticCard(const char* name, const char* company);

        void renderSceneObjects();
  
        // Internal state
        bool simulationRunning = false;
        bool gravityEnabled = false;
        bool _showRobotSelector = false;
        bool _robotRequested = false;
        bool _hasRobot = false;

        std::string _requestedRobot;
        std::string _currentObjectName;

		float simLength = 30.0f;  // ~30 seconds default
		float deltaTime = 0.016f; // ~60 FPS default
		float simTime = 0.0f;     // current simulation time

        int povMode = 0;

        // Physics
        float velocity = 0.0f;
        float torque = 0.0f;
        float linkLength = 1.0f;
        float damping = 0.1f;
        float position = 0.0f;

		double PI = constants::PhysConstants::PI;

        std::shared_ptr<elements::Mesh> _mesh;
        elements::Light* _sunLight;
		elements::Object* _obj;

        ImGui::FileBrowser _meshLoad;
        ImGui::FileBrowser _hdrLoad;
        std::string _currentMeshFile;
        std::string _currentHDRFile;

        std::function<void(const std::string&)> meshLoadCallback;
        std::function<void(bool)> simCallback;
        SceneView* _sceneView = nullptr;
        SceneView::ControlMode* _controlMode;

        Selection _selection;


		// Time tracking for simulation updates
        double finalTime = 0.0f;
		std::chrono::high_resolution_clock::time_point lastUpdateTime = std::chrono::high_resolution_clock::now();
    };
}