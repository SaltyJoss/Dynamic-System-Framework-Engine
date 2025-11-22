
#include "pch.h"

#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "Scene/ControlPanel.h"
#include <imgui.h>

#include "EngineLib/LogMacros.h"


namespace gui {
    void ControlPanel::render(gui::SceneView* sceneView) {
        _sceneView = sceneView; // store pointer for convenience
        _mesh = sceneView->getMesh();
        _obj = sceneView->getObject();
        _sunLight = _sceneView->getSunLight();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_FirstUseEver);

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.129f, 0.129f, 0.129f, 0.8f));
        ImGui::Begin("Control Panel", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoTitleBar);


        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Load Obj")) { _meshLoad.Open(); LOG_INFO("File dialog opened"); }
                if (ImGui::MenuItem("Load HDR")) { _hdrLoad.Open(); LOG_INFO("HDR file dialog opened"); }

                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Edit")) {
                if (ImGui::MenuItem("Reset View")) {
                    _sceneView->resetView();
                    LOG_INFO("Scene view reset to default position and orientation.");
                }
                if (ImGui::MenuItem("Properties")) {
                    // Placeholder for future properties dialog
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Robotic Arms")) {
                if (ImGui::MenuItem("Select Model")) {
                    _showRobotSelector = true;
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Shader"))
            {
                // Reload button
                if (ImGui::MenuItem("Reload Shaders")) {
                    LOG_INFO("Shader reload requested.");
                    _sceneView->reloadAllShaders();
                }

                ImGui::Separator();

                // Shader selection
                if (ImGui::MenuItem("Basic Shader", nullptr, _sceneView->currentShaderMode == SceneView::ShaderMode::Basic)) {
                    _sceneView->currentShaderMode = SceneView::ShaderMode::Basic;
                }

                if (ImGui::MenuItem("Lit Shader", nullptr, _sceneView->currentShaderMode == SceneView::ShaderMode::Lit)) {
                    _sceneView->currentShaderMode = SceneView::ShaderMode::Lit;
                }

                if (ImGui::MenuItem("PBR Shader", nullptr, _sceneView->currentShaderMode == SceneView::ShaderMode::PBR)) {
                    _sceneView->currentShaderMode = SceneView::ShaderMode::PBR;
                }

                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        renderRoboticSelector();

        if (ImGui::CollapsingHeader("Simulation Settings")) { renderSimulationProperties(); renderObjectProperties(); renderLinkProperties(); renderStats(); }
        if (ImGui::CollapsingHeader("Camera Settings")) { renderCameraProperties(); }
        if (ImGui::CollapsingHeader("Display Settings")) { renderDisplaySettings(); }

		renderSceneObjects();

        ImGui::End();
        ImGui::PopStyleColor();

        _meshLoad.Display();
        if (_meshLoad.HasSelected()) {
            auto file_path = _meshLoad.GetSelected().string();
            _currentMeshFile = file_path.substr(file_path.find_last_of("/\\") + 1);
            meshLoadCallback(file_path);
            LOG_INFO("Mesh loaded from file: %s", _currentMeshFile.c_str());

            _meshLoad.ClearSelected();
        }

        _hdrLoad.Display();
        if (_hdrLoad.HasSelected()) {
            auto file_path = _hdrLoad.GetSelected().string();
            _currentHDRFile = file_path.substr(file_path.find_last_of("/\\") + 1);
            _sceneView->loadNewHDR(file_path);
            LOG_INFO("HDR loaded from file: %s", _currentHDRFile.c_str());
            _hdrLoad.ClearSelected();
        }
    }

    void ControlPanel::renderSimulationProperties() {
        ImGui::SeparatorText("Simulation Controls");
        ImGui::Checkbox("Enable Gravity", &gravityEnabled);
    }

    void ControlPanel::renderObjectProperties() {
        if (!_obj) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No object selected.");
            ImGui::Separator();
            return;
        }

        ImGui::SeparatorText("Object Controls");

        if (gravityEnabled) {
            double minGravity = 0.0; double maxGravity = 20.0;
            ImGui::DragScalar("Gravity", ImGuiDataType_Double, &_obj->state.gravity, 0.00005f, &minGravity, &maxGravity); // 5 decimal places for precision (because I want to test realistic gravity values)
        }

        ImGui::Separator();

        ImGui::Text("Mass:");
        double minMass = 0.25; double maxMass = 100.0;
        ImGui::DragScalar("kg", ImGuiDataType_Double, &_obj->state.mass, 0.025f, &minMass, &maxMass);

		ImGui::Separator();

        ImGui::Text("Scale:");
        float minScale = 0.0001; float maxScale = 100.0;
        ImGui::DragFloat("(x)", &_obj->transform.scale.x, 0.001f, minScale, maxScale);
        ImGui::DragFloat("(y)", &_obj->transform.scale.y, 0.001f, minScale, maxScale);
        ImGui::DragFloat("(z)", &_obj->transform.scale.z, 0.001f, minScale, maxScale);

        ImGui::Separator();

        ImGui::Text("Linear Velocity:");
        double minVelocity = -100.0; double maxVelocity = 100.0;
        ImGui::DragScalar("(x-axis)##2", ImGuiDataType_Double, &_obj->state.linearVelocity.x(), 0.0025f, &minVelocity, &maxVelocity);
        ImGui::DragScalar("(y-axis)##2", ImGuiDataType_Double, &_obj->state.linearVelocity.y(), 0.0025f, &minVelocity, &maxVelocity);
        ImGui::DragScalar("(z-axis)##2", ImGuiDataType_Double, &_obj->state.linearVelocity.z(), 0.0025f, &minVelocity, &maxVelocity);

        ImGui::Separator();

        ImGui::Text("Angular Velocity:");
        double minTorque = -100.0; double maxTorque = 100.0;
        ImGui::DragScalar("(x-axis)##3", ImGuiDataType_Double, &_obj->state.angularVelocity.x(), 0.0025f, &minTorque, &maxTorque);
        ImGui::DragScalar("(y-axis)##3", ImGuiDataType_Double, &_obj->state.angularVelocity.y(), 0.0025f, &minTorque, &maxTorque);
        ImGui::DragScalar("(z-axis)##3", ImGuiDataType_Double, &_obj->state.angularVelocity.z(), 0.0025f, &minTorque, &maxTorque);

        ImGui::Separator();

        ImGui::Text("Reset Object:");
        // Reset Object Button
        if (ImGui::Button("Reset")) {
            _obj->reset();
            LOG_INFO("Object reset to initial position and orientation.");
        }
    }

    void ControlPanel::renderLinkProperties() {
        ImGui::SeparatorText("Link Controls");
        // Placeholder for future link properties
    }

    void ControlPanel::renderStats() {
        ImGui::SeparatorText("Simulation Statistics");
        if (_obj && _obj->getMesh())
        {
            const glm::vec3& pos = _obj->transform.position;
            const glm::vec3& rot = _obj->transform.rotation; // radians most likely

            ImGui::SeparatorText("Telemetry");

            // Position block
            if (ImGui::BeginTable("telemetryTable", 2, ImGuiTableFlags_BordersInnerV))
            {
                // Position
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::Text("Position (m)");
                ImGui::TableSetColumnIndex(1); ImGui::Text("X: %.3f  Y: %.3f  Z: %.3f", pos.x, pos.y, pos.z);

                // Rotation
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::Text("Rotation (deg)");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("Pitch: %.1f  Yaw: %.1f  Roll: %.1f",
                    glm::degrees(rot.x),
                    glm::degrees(rot.y),
                    glm::degrees(rot.z));

                ImGui::EndTable();
            }

            // Velocity, acceleration, forces, torque, mass…
        }
    }

    void ControlPanel::renderCameraProperties() {
        ImGui::Text("Control Mode:");
        if (ImGui::RadioButton("Camera", *_controlMode == SceneView::ControlMode::Camera)) { *_controlMode = SceneView::ControlMode::Camera; }
        ImGui::SameLine();
        if (ImGui::RadioButton("Object", *_controlMode == SceneView::ControlMode::Object)) { *_controlMode = SceneView::ControlMode::Object; }

        if (*_controlMode == SceneView::ControlMode::Object) { _sceneView->attachCameraToObject(_obj); }
        else { _sceneView->detachCameraFromObject(); }


        ImGui::SeparatorText("Light Controls");

        // Kept for future use with star light simulation (NOT NEEDED PURELY VISUAL)

        ImGui::SeparatorText("Object Appearance");
        if (!_mesh) {
            ImGui::Text("No mesh loaded!");
            LOG_WARN_ONCE("No mesh loaded while rendering Object Appearance");
        }
    }

    void ControlPanel::renderDisplaySettings() {
        ImGui::SeparatorText("Display Settings");

        bool enabled = _sceneView->isSkyboxEnabled();
        if (ImGui::Checkbox("Enable Skybox", &enabled)) {
            _sceneView->setSkyboxEnabled(enabled);
            LOG_INFO("Skybox Enabled = %s", enabled ? "true" : "false");
        }

        float fov = _sceneView->getCamera()->getFOV(); // in degrees
        if (ImGui::SliderFloat("Field of View", &fov, 30.0f, 120.0f, "%.5f")) {
            _sceneView->getCamera()->setFOV(fov);
        }
    }

	// Robotic Arm Selector
    void ControlPanel::renderRoboticSelector() {
        if (!_showRobotSelector) return;

        ImGui::Begin("Choose Robotic Arm", &_showRobotSelector, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoDocking);

        ImGui::Text("Select a robotic arm model:");
        ImGui::Separator();
        ImGui::Spacing();

        renderRoboticCard("Z1", "Unitree Robotics");
        renderRoboticCard("UR5", "Universal Robots");
        renderRoboticCard("Panda", "Franka Robotics");
        renderRoboticCard("KUKA iiwa", "KUKA");

        ImGui::End();
    }

	// Robotic Arm Card for Selector (lists robotic arms to choose from)
    void ControlPanel::renderRoboticCard(const char* name, const char* company) {
        ImGui::PushID(name);

        ImGui::BeginChild("robot_card", ImVec2(0, 42.5), true, ImGuiWindowFlags_None);

        if (ImGui::Selectable(name, false, ImGuiSelectableFlags_AllowDoubleClick)) {
            // Load JSON + STLs here
            LOG_INFO("Selected robot: %s", name);
            _showRobotSelector = false;

            _requestedRobot = name;
            _robotRequested = true;

            _sceneView->loadRobot(_requestedRobot);
        }

        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "%s", company);

        ImGui::EndChild();
        ImGui::Spacing();

        ImGui::PopID();
	}

	// Scene Objects List
    void ControlPanel::renderSceneObjects() {
        ImGui::BeginChild("SceneObjectsChild", ImVec2(0, 250), true);

        auto& objs = _sceneView->getObjects();
        int indexToDelete = -1;

        if (ImGui::BeginTable("ObjTable", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV))
        {
            ImGui::TableSetupColumn("Object ID");
            ImGui::Separator();
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 60.f);
            ImGui::TableHeadersRow();

            for (int i = 0; i < objs.size(); i++) {
                auto* obj = objs[i].get();
                bool isSelected = (obj == _sceneView->getObject());

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);

                if (ImGui::Selectable(("Object " + std::to_string(i)).c_str(), isSelected)) {
                    _sceneView->setSelectedObject(obj);
                }

                ImGui::TableSetColumnIndex(1);
                if (ImGui::Button(("Delete##" + std::to_string(i)).c_str())) {
                    indexToDelete = i;
                }
            }

            ImGui::EndTable();
        }

        if (indexToDelete != -1) {
            _sceneView->deleteObject(indexToDelete);
            LOG_INFO("Deleted object at index %d", indexToDelete);
        }

        ImGui::EndChild();
    }
}

