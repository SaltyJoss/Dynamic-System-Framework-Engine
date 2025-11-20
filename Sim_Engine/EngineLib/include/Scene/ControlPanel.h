#pragma once

#include "EngineCore.h"
#include "Scene/Object.h"
#include "Scene/SceneView.h"
#include "Scene/Light.h"
#include "Platform/Logger.h"

#include <imgui.h>
#include "Platform/imguiWidgets.h"
#include <imfilebrowser.h>

extern ENGINE_API Debug gLog;

namespace gui {
    class ENGINE_API ControlPanel {
    public:
        ControlPanel(SceneView* sceneView) : 
            _sceneView(sceneView), _controlMode(&sceneView->ctrlMode),
            _meshLoad(ImGuiFileBrowserFlags_CloseOnEsc | ImGuiFileBrowserFlags_NoModal),
            _hdrLoad(ImGuiFileBrowserFlags_CloseOnEsc | ImGuiFileBrowserFlags_NoModal) 
        {
            _currentMeshFile = "<...>";
            _currentHDRFile = "<...>";

            _meshLoad.SetTitle("Open Object Mesh");
			_meshLoad.SetDirectory("Engine/assets/objects/shapes");
            _meshLoad.SetTypeFilters({ ".fbx", ".obj" });

			_hdrLoad.SetTitle("Load HDR Environment");
			_hdrLoad.SetDirectory("Engine/assets/hdr");
			_hdrLoad.SetTypeFilters({ ".hdr", ".exr" });
        }

        void render(gui::SceneView* sceneView);
        void setSimulationCallback(const std::function<void(bool)>& callback) { _simCallback = callback; }
        void setMeshLoadCallback(const std::function<void(const std::string&)>& callback) { _meshLoadCallback = callback; }

    private:
        void renderSimulationProperties();
        void renderCameraProperties();
        void renderObjectProperties();
        void renderLinkProperties();
        void renderDisplaySettings();
        void renderStats();

        void renderHierarchy(SceneView* view);
        void renderSceneObjects();
  
        // Internal state
        bool simulationRunning = false;
        bool gravityEnabled = false;
        float simulationSpeed = 1;
        int povMode = 0;

        // Physics
        float velocity = 0.0f;
        float torque = 0.0f;
        float linkLength = 1.0f;
        float damping = 0.1f;
        float position = 0.0f;

		double PI = 3.14159265358979323846;

        std::shared_ptr<elements::Mesh> _mesh;
        elements::Light* _sunLight;
		elements::Object* _obj;

        ImGui::FileBrowser _meshLoad;
        ImGui::FileBrowser _hdrLoad;
        std::string _currentMeshFile;
        std::string _currentHDRFile;

        std::function<void(const std::string&)> _meshLoadCallback;
        std::function<void(bool)> _simCallback;
        SceneView* _sceneView = nullptr;
        SceneView::ControlMode* _controlMode;
    };
}