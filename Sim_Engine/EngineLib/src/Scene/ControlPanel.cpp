// ==================================================
//				File: ControlPanel.cpp
// ==================================================

#include "pch.h"

#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "Scene/ControlPanel.h"
#include "Robots/RobotModel.h"
#include <imgui.h>
#include <chrono>

#include "EngineLib/LogMacros.h"

namespace gui {
    ControlPanel::ControlPanel(simManager* sceneView) :
		_sim(sceneView), _controlMode(&sceneView->ctrlMode), _phys(nullptr), _obj(nullptr), _sunLight(nullptr),
        _meshLoad(ImGuiFileBrowserFlags_CloseOnEsc | ImGuiFileBrowserFlags_NoModal),
        _hdrLoad(ImGuiFileBrowserFlags_CloseOnEsc | ImGuiFileBrowserFlags_NoModal)
    {
		diagTime = 0.0f; // initialize diagnostic time
		simTime = 0.0f;  // initialize simulation time

        diagRunning = false;
		simulationRunning = false;

        // File browsers
        _currentMeshFile = "<...>";
        _currentHDRFile = "<...>";
        // Mesh loader
        _meshLoad.SetTitle("Open Object Model");
        _meshLoad.SetDirectory("Engine/assets/objects");
        _meshLoad.SetTypeFilters({ ".fbx", ".obj", ".dae", ".stl"});
        // HDR loader
        _hdrLoad.SetTitle("Load HDR Environment");
        _hdrLoad.SetDirectory("Engine/assets/hdr");
        _hdrLoad.SetTypeFilters({ ".hdr", ".exr" });
    }

    void ControlPanel::drawMenus(simManager* sim) {
        _sim = sim;
        _phys = &_sim->getPhysicsSystem();
        _obj = _sim->getObject();

        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Save Layout")) {
                ImGui::SaveIniSettingsToDisk("Engine/configs/imgui_layout.ini");
            }
            if (ImGui::MenuItem("Load Layout")) {
                ImGui::LoadIniSettingsFromDisk("Engine/configs/imgui_layout.ini");
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Reset View")) {
                _sim->resetView();
                LOG_INFO("Scene view reset to default position and orientation.");
            }
            if (ImGui::MenuItem("Properties")) {
                // Placeholder for future properties dialog
            }
            /*if (ImGui::MenuItem("Reset HDR")) {
                _sim->resetHDRToPreset();
            }*/
            ImGui::EndMenu();
        }


        if (ImGui::BeginMenu("Project")) {
            if (ImGui::MenuItem("Load Obj")) { _meshLoad.Open(); LOG_INFO("File dialog opened"); }
            if (ImGui::MenuItem("Load Robotic Arm")) { _showRobotSelector = true; LOG_INFO("Robotic Arm Menu Opened"); }
            //if (ImGui::MenuItem("Load HDR")) { _hdrLoad.Open(); LOG_INFO("HDR file dialog opened"); }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Render")) {
            if (ImGui::MenuItem("Quality: Low", nullptr, q == render::QualityPreset::Low)) {
                q = render::QualityPreset::Low;
                _sim->applyRenderProfile(render::MakeSettings(l, q), l);
            }
            if (ImGui::MenuItem("Quality: Medium", nullptr, q == render::QualityPreset::Medium)) {
                q = render::QualityPreset::Medium;
                _sim->applyRenderProfile(render::MakeSettings(l, q), l);
            }
            if (ImGui::MenuItem("Quality: High", nullptr, q == render::QualityPreset::High)) {
                q = render::QualityPreset::High;
                _sim->applyRenderProfile(render::MakeSettings(l, q), l);
            }
            if (ImGui::MenuItem("Quality: Ultra", nullptr, q == render::QualityPreset::Ultra)) {
                q = render::QualityPreset::Ultra;
                _sim->applyRenderProfile(render::MakeSettings(l, q), l);
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Look: Studio", nullptr, l == render::LookPreset::Studio)) {
                l = render::LookPreset::Studio;
                _sim->applyRenderProfile(render::MakeSettings(l, q), l);
                LOG_INFO("Render settings applied: LookPreset=%d, QualityPreset=%d", static_cast<int>(l), static_cast<int>(q));
                D_INFO("Render settings applied: LookPreset=%d, QualityPreset=%d", static_cast<int>(l), static_cast<int>(q));
            }
            if (ImGui::MenuItem("Look: Cinematic", nullptr, l == render::LookPreset::Cinematic)) {
                l = render::LookPreset::Cinematic;
                _sim->applyRenderProfile(render::MakeSettings(l, q), l);
                LOG_INFO("Render settings applied: LookPreset=%d, QualityPreset=%d", static_cast<int>(l), static_cast<int>(q));
                D_INFO("Render settings applied: LookPreset=%d, QualityPreset=%d", static_cast<int>(l), static_cast<int>(q));
            }

            ImGui::EndMenu();
        }
        /*if (ImGui::BeginMenu("Robotic Arms")) {
            if (ImGui::MenuItem("Select Model")) {
                _showRobotSelector = true;
            }
            ImGui::EndMenu();
        }*/
        if (ImGui::BeginMenu("Shader"))
        {
            // Reload shader button
            if (ImGui::MenuItem("Reload Shaders")) {
                LOG_INFO("Shader reload requested.");
                _sim->reloadAllShaders();
            }

            ImGui::Separator();

            // Shader selection
            if (ImGui::MenuItem("Basic Shader", nullptr, _sim->currentShaderMode == simManager::ShaderMode::Basic)) {
                _sim->currentShaderMode = simManager::ShaderMode::Basic;
                D_INFO("Shader -> Basic Shader");
            }

            if (ImGui::MenuItem("Lit Shader", nullptr, _sim->currentShaderMode == simManager::ShaderMode::Lit)) {
                _sim->currentShaderMode = simManager::ShaderMode::Lit;
                D_INFO("Shader -> Lit Shader");
            }

            if (ImGui::MenuItem("PBR Shader", nullptr, _sim->currentShaderMode == simManager::ShaderMode::PBR)) {
                _sim->currentShaderMode = simManager::ShaderMode::PBR;
                D_INFO("Shader -> PBR Shader");
            }

            ImGui::EndMenu();
        }
    }

    void ControlPanel::render(simManager* sceneView) {
        // Initialize pointers to scene scene
        _sim = sceneView;
        fov = _sim->getCamera()->getFOVRadians();
        _mesh = _sim->getMesh();
        _obj = _sim->getObject();
        _sunLight = _sim->getSunLight();
        _hasRobot = _sim->hasRobot();

		_phys = &_sim->getPhysicsSystem();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_FirstUseEver);

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.129f, 0.129f, 0.129f, 0.8f));
        ImGui::Begin("Control Panel", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoTitleBar);

        if (ImGui::BeginMenuBar()) {
			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 6.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 8.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 4.0f));

            const bool wasRunning = simulationRunning; // snapshot

            if (wasRunning) {
                ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.80f, 0.15f, 0.15f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.90f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.70f, 0.10f, 0.10f, 1.0f));
            }

            // Simulation Start/Stop Button
            if (ImGui::Button(wasRunning ? "Terminate" : "Run")) {
                simulationRunning = !simulationRunning;

				if (wasRunning && !simulationRunning) {
                    // Stopping simulation
                    simTime = 0.0f;
                }

				// Logs 
                LOG_INFO("Simulation %s", simulationRunning ? "started" : "stopped");
                D_RUNTIME("Simulation %s", simulationRunning ? "started" : "stopped");
            }

            if (wasRunning) {
                ImGui::PopStyleColor(3);
			}

            // Diagnostic Start/Stop Button
            if (ImGui::Button(diagRunning ? "Stop" : "Start")) {
                diagRunning = !diagRunning;
                LOG_INFO("Diagnostics %s", diagRunning ? "started" : "stopped");
                D_RUNTIME("Diagnostics %s", diagRunning ? "started" : "stopped");

                if (diagRunning) { 
                    _phys->startDiagnostics(_obj); 

                    if (!_phys->diagnosticsRunning()) {
                        diagRunning = false;
                        LOG_WARN("Diagnostics terminated");
                        D_FAIL("Diagnostics terminated");
                    }
                }
                else {
                    _phys->stopDiagnostics();
                    diagRunning = false;
                    D_RUNTIME("Diagnostic run time: %.3f seconds", diagTime);
                    diagTime = 0.0f;
                }
            }
			ImGui::PopStyleVar(3);
            ImGui::EndMenuBar();
        }

		beginControlPanel("ControlPanel"); // Begin Child Panel

        roboticArmSelector();
		ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader("Simulation")) {
            simulationProperties();
            linkProperties();
            objectProperties();
            stats();
        }
        if (ImGui::CollapsingHeader("Camera")) { cameraProperties(); }
        if (ImGui::CollapsingHeader("Display")) { displaySettings(); }

        sceneObjectsTable();

		endControlPanel(); // End Child Panel

        ImGui::End();
        ImGui::PopStyleColor();

        _meshLoad.Display();
        if (_meshLoad.HasSelected()) {
            auto file_path = _meshLoad.GetSelected().string();
            _currentMeshFile = file_path.substr(file_path.find_last_of("/\\") + 1);
            meshLoadCallback(file_path);
            LOG_INFO("Mesh loaded from file: %s", _currentMeshFile.c_str());
			D_SUCCESS("Mesh loaded from file: %s", _currentMeshFile.c_str());

            _meshLoad.ClearSelected();
        }

        _hdrLoad.Display();
        if (_hdrLoad.HasSelected()) {
            auto file_path = _hdrLoad.GetSelected().string();
            _currentHDRFile = file_path.substr(file_path.find_last_of("/\\") + 1);
            _sim->loadNewHDR_UI(file_path);
            LOG_INFO("HDR loaded from file: %s", _currentHDRFile.c_str());
			D_SUCCESS("HDR loaded from file: %s", _currentHDRFile.c_str());
            _hdrLoad.ClearSelected();
        }
    }

    void ControlPanel::simulationProperties() {
        ImGui::Text("Setup");
        ImGui::Separator();

		ImGui::Text("Simulation Length:");
		ImGui::SameLine();
		ImGui::Text("           Diagnostic Length:");

        float step = 0.001f;
        float stepFast = 0.01f;
        ImGui::SetNextItemWidth(150.0f);
        ImGui::InputScalar("seconds##sim", ImGuiDataType_Float, &simLength, &step, &stepFast, "%.3f");

		ImGui::SameLine();

        ImGui::SetNextItemWidth(150.0f);
        ImGui::InputScalar("seconds##diag", ImGuiDataType_Float, &diagLength, &step, &stepFast, "%.3f");
        ImGui::Text("Delta Time (dt)");
        ImGui::SetNextItemWidth(150.0f);
		ImGui::InputScalar("seconds##dt", ImGuiDataType_Float, &deltaTime, &step, &stepFast, "%.5f");

		ImGui::NewLine();

        ImGui::Text("Integration Method");
        auto& phys = _sim->getPhysicsSystem();
        auto currentEnum = phys.getIntegrationMethod();

        static const char* methodNames[] = { "Euler", "Midpoint", "Heun", "Ralston", "RK4" };
        const char* currentMethod = methodNames[static_cast<int>(currentEnum)];
        
		ImGui::SetNextItemWidth(150.0f);
        if (ImGui::BeginCombo("##", currentMethod)) {
            for (int n = 0; n < IM_ARRAYSIZE(methodNames); ++n) {
                bool isSelected = (n == static_cast<int>(currentEnum));

                if (ImGui::Selectable(methodNames[n], isSelected)) {
                    auto updatedMethod = static_cast<physics::PhysicsSystem::eIntegrationMethod>(n);
                    phys.setIntegrationMethod(updatedMethod);

                    switch (updatedMethod) {
                    case physics::PhysicsSystem::eIntegrationMethod::Euler:
                        D_INFO("Integrator set to Euler");
                        break;
                    case physics::PhysicsSystem::eIntegrationMethod::Midpoint:
                        D_INFO("Integrator set to RK2 (Midpoint)");
                        break;
                    case physics::PhysicsSystem::eIntegrationMethod::Heun:
                        D_INFO("Integrator set to RK2 (Heun)");
						break;
                    case physics::PhysicsSystem::eIntegrationMethod::Ralston:
                        D_INFO("Integrator set to RK2 (Ralston)");
						break;
                    case physics::PhysicsSystem::eIntegrationMethod::RK4:
                        D_INFO("Integrator set to RK4");
                        break;
                    default:
                        break;
                    }
                }
                if (isSelected) {
                    ImGui::SetItemDefaultFocus();
                }
			}
            ImGui::EndCombo();
        }

        ImGui::NewLine();

		// Deals with simulation time tracking using chrono
        if (simulationRunning) {
            auto now = std::chrono::high_resolution_clock::now();
            double deltaSeconds = std::chrono::duration<double>(now - simLastUpdateTime).count();
            simLastUpdateTime = now;
            simTime += static_cast<float>(deltaSeconds);

            ImGui::Text("Elapsed Time...");
            ImGui::Text("Simulation Time: %.3f / %.3f seconds", simTime, simLength);

            if (simTime >= simLength) {
                simulationRunning = false;
                simTime = 0.0f;
                D_RUNTIME("Total elapsed time : % .1f seconds.", simLength);
            }
        }
        else {
            simLastUpdateTime = std::chrono::high_resolution_clock::now();
        }

        if (diagRunning) {
			// Update diagnostic time
            auto now = std::chrono::high_resolution_clock::now();
            double deltaSeconds = std::chrono::duration<double>(now - diagLastUpdateTime).count();
            diagLastUpdateTime = now;
            diagTime += static_cast<float>(deltaSeconds);

            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Diagnostics Running...");
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Diagnostic Time: %.3f seconds", diagTime);

            if (diagTime >= diagLength) {
                diagRunning = false;
                diagTime = 0.0f;
                _phys->stopDiagnostics();
                D_RUNTIME("Diagnostic run time: %.3f seconds", diagLength);
			}
		} else {
            diagLastUpdateTime = std::chrono::high_resolution_clock::now();
        }
		ImGui::Separator();

    }

    void ControlPanel::objectProperties() {
        if (!_obj) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No object selected.");
            ImGui::Separator();
            return;
        }

        ImGui::SeparatorText("Physics Settings:");

		// Gravity Controls
        ImGui::Text("Gravity");
        double minGravity = 0.0; double maxGravity = 20.0;
		ImGui::SetNextItemWidth(150.0f);
        ImGui::DragScalar("m/s^2", ImGuiDataType_Double, &_obj->state.gravity, 0.00005f, &minGravity, &maxGravity); // 5 decimal places for precision (because I want to test realistic gravity values)

        ImGui::Separator();

		// Mass Controls
        ImGui::Text("Mass:");
        double minMass = 0.25; double maxMass = 100.0;
        ImGui::SetNextItemWidth(150.0f);
        ImGui::DragScalar("kg", ImGuiDataType_Double, &_obj->state.mass, 0.025f, &minMass, &maxMass);

		ImGui::Separator();

		// Damping Controls
        ImGui::Text("Damping Coefficient");
		double minDamping = 0.0; double maxDamping = 1.0;
		ImGui::SetNextItemWidth(150.0f);
		ImGui::DragScalar("kg/s", ImGuiDataType_Double, &_obj->state.damping, 0.001f, &minDamping, &maxDamping);

        ImGui::Separator();

        if (_obj->category == scene::ObjectCategory::General) {
            ImGui::SetNextItemWidth(150.0f);
            // Scale Controls
            ImGui::Text("Scale:");
            float minScale = 0.0001f; float maxScale = 100.0f;
            ImGui::DragFloat("(x)", &_obj->transform.scale.x, 0.001f, minScale, maxScale);
            ImGui::DragFloat("(y)", &_obj->transform.scale.y, 0.001f, minScale, maxScale);
            ImGui::DragFloat("(z)", &_obj->transform.scale.z, 0.001f, minScale, maxScale);

            ImGui::Separator();

            // Linear Velocity Controls
            ImGui::Text("Linear Velocity:");
            double minVelocity = -100.0; double maxVelocity = 100.0;
            ImGui::DragScalar("X##linVelX", ImGuiDataType_Double, &_obj->state.linearVelocity.x(), 0.0025f, &minVelocity, &maxVelocity);
            ImGui::DragScalar("Y##linVelY", ImGuiDataType_Double, &_obj->state.linearVelocity.y(), 0.0025f, &minVelocity, &maxVelocity);
            ImGui::DragScalar("Z##linVelZ", ImGuiDataType_Double, &_obj->state.linearVelocity.z(), 0.0025f, &minVelocity, &maxVelocity);

            ImGui::Separator();

            // Angular Velocity Controls
            ImGui::Text("Angular Velocity:");
            double minTorque = -100.0; double maxTorque = 100.0;
            ImGui::DragScalar("X##angVelX", ImGuiDataType_Double, &_obj->state.angularVelocity.x(), 0.0025f, &minTorque, &maxTorque);
            ImGui::DragScalar("Y##angVelY", ImGuiDataType_Double, &_obj->state.angularVelocity.y(), 0.0025f, &minTorque, &maxTorque);
            ImGui::DragScalar("Z##angVelZ", ImGuiDataType_Double, &_obj->state.angularVelocity.z(), 0.0025f, &minTorque, &maxTorque);

            ImGui::Separator();
        }

        ImGui::Text("Reset Object:");
        // Reset Object Button
        if (ImGui::Button("Reset")) {
            if (!_obj) {
                LOG_WARN("No object selected to reset.");
                return;
			}

            if (_hasRobot && _obj->category == scene::ObjectCategory::General) {
                LOG_WARN("Cannot reset individual robot links. Please reset the entire robot model.");
                return;
			}
            _obj->reset();
            LOG_INFO("Object reset to initial position and orientation.");
            D_INFO("Reset %s", _obj);
        }
    }

    void ControlPanel::linkProperties() {
        if (!_hasRobot) { return; }

        ImGui::Text("Link Controls");
		ImGui::Separator();

        // Rotate link01 around y axis, rotates all child links#
        const RobotModel& robot = _sim->getRobotModel();
		float minAngle = -360.0f; float maxAngle = 360.0f;

        for (const auto& joint : robot.joints) {
            if (joint.child == _currentLinkName) {
                minAngle = joint.minAngle;
                maxAngle = joint.maxAngle;
                break;
            }
        }

        float& angle = _linkAngles[_currentLinkName];

        ImGui::Text("Rotate %s", _currentLinkName.c_str());
        ImGui::SliderFloat("Angle## (deg)", &angle, minAngle, maxAngle, "%.1f");
		_sim->setRobotLinkRotation(_currentLinkName, angle);
		ImGui::Separator();
    }

    void ControlPanel::stats() {
        if (!_obj) { return; }

        ImGui::SeparatorText("Simulation Statistics");
        if (_obj && _obj->getMesh())
        {
            const glm::vec3& pos = _obj->transform.position;
            const glm::vec3& rot = _obj->transform.rotation;

            ImGui::Text("Plots");

            if (_obj->category == scene::ObjectCategory::General) {
                // Linear velocity plot
                static std::vector<float> linVelHistory;
                linVelHistory.push_back(static_cast<float>(_obj->state.linearVelocity.norm()));
                if (linVelHistory.size() > 100) linVelHistory.erase(linVelHistory.begin());

                // Angular velocity plot
                static std::vector<float> angVelHistory;
                angVelHistory.push_back(static_cast<float>(_obj->state.angularVelocity.norm()));
                if (angVelHistory.size() > 100) angVelHistory.erase(angVelHistory.begin());

                // Plot Outputs
                ImGui::PlotLines("Linear Velocity Magnitude", linVelHistory.data(), (int)linVelHistory.size(), 0, nullptr, 0.0f, 50.0f, ImVec2(0, 25));
                ImGui::PlotLines("Angular Velocity Magnitude", angVelHistory.data(), (int)angVelHistory.size(), 0, nullptr, 0.0f, 50.0f, ImVec2(0, 25));
            }

            ImGui::Separator();

            ImGui::Text("Telemetry");
			ImGui::Separator();

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
        }
    }

    void ControlPanel::cameraProperties() {
        ImGui::Text("Control Mode:");
        if (ImGui::RadioButton("Camera##", *_controlMode == simManager::ControlMode::Camera)) { *_controlMode = simManager::ControlMode::Camera; }
        ImGui::SameLine();
        if (ImGui::RadioButton("Object##", *_controlMode == simManager::ControlMode::Object)) { *_controlMode = simManager::ControlMode::Object; }

        if (*_controlMode == simManager::ControlMode::Object) { _sim->attachCameraToObject(_obj); }
        else { _sim->detachCameraFromObject(); }

        ImGui::SeparatorText("Light Controls");

        // Kept for future use with star light simulation (NOT NEEDED PURELY VISUAL)

        ImGui::SeparatorText("Object Appearance");
        if (!_mesh) {
            ImGui::Text("No mesh loaded!");
            LOG_WARN_ONCE("No mesh loaded while rendering Object Appearance");
        }
    }

    void ControlPanel::displaySettings() {
        ImGui::SeparatorText("Display Settings");

        bool enabled = _sim->isSkyboxEnabled();
        if (ImGui::Checkbox("Enable Skybox", &enabled)) {
            _sim->setSkyboxEnabled(enabled);
            LOG_INFO("Skybox Enabled = %s", enabled ? "true" : "false");
        }

        static float fovDeg = 70.0f;

        bool edited = ImGui::SliderFloat("Field of View", &fovDeg, 25.0f, 125.0f, "%.1f");
        bool active = ImGui::IsItemActive();

        if (!active && !edited) { fovDeg = _sim->getCamera()->getFOVDegrees(); }
        if (edited) { _sim->getCamera()->setFOVDegrees(fovDeg); }
    }

	// Robotic Arm Selector
    void ControlPanel::roboticArmSelector() {
        if (!_showRobotSelector) return;

        ImGui::Begin("Choose Robotic Arm", &_showRobotSelector, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoDocking);

        ImGui::Text("Select a robotic arm model:");
        ImGui::Separator();
        ImGui::Spacing();

        roboticCardDisplay("Z1", "Unitree Robotics");
        roboticCardDisplay("UR5", "Universal Robots");
        roboticCardDisplay("Panda", "Franka Robotics");
        roboticCardDisplay("KUKA iiwa", "KUKA");

        ImGui::End();
    }

	// Robotic Arm Card for Selector (lists robotic arms to choose from)
    void ControlPanel::roboticCardDisplay(const char* name, const char* company) {
        ImGui::PushID(name);

        ImGui::BeginChild("robot_card", ImVec2(0, 42.5), true, ImGuiWindowFlags_None);

        // Loads robot
        if (ImGui::Selectable(name, false, ImGuiSelectableFlags_AllowDoubleClick)) {
            LOG_INFO("Selected robot: %s", name);
            _showRobotSelector = false;

            _requestedRobot = name;
            _robotRequested = true;

            _sim->loadRobot(_requestedRobot);
			_hasRobot = true;
        }

        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "%s", company);

        ImGui::EndChild();
        ImGui::Spacing();

        ImGui::PopID();
	}

	// Scene Objects List
    void ControlPanel::sceneObjectsTable() {
        ImGui::BeginChild("SceneObjectsChild", ImVec2(0, 250), true);

        auto& objs = _sim->getObjects();          // get reference to scene objects
        int indexToDelete = -1;

        ImGui::Text("Scene Table");
        ImGui::Separator();

        ImGuiTableFlags tableFlags =
            ImGuiTableFlags_BordersV |
            ImGuiTableFlags_BordersOuterH |
            ImGuiTableFlags_Resizable |
            ImGuiTableFlags_RowBg |
            ImGuiTableFlags_NoBordersInBody;

        if (ImGui::BeginTable("SceneTable", 3, tableFlags))
        {
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_NoHide);
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Parent / Action");
            ImGui::TableHeadersRow();

            // Robot section
            if (_hasRobot) {
                RobotModel& robot = _sim->getRobotModel();

                bool robotSelected = (_selection.type == SelectionType::ROBOT);

                // Robot Root Row
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);

                ImGuiTreeNodeFlags rootFlags =
                    ImGuiTreeNodeFlags_SpanAllColumns |
                    ImGuiTreeNodeFlags_DefaultOpen;

                bool openRoot = ImGui::TreeNodeEx("Z1", rootFlags);

                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted("ROOT");

                ImGui::TableSetColumnIndex(2);
                if (ImGui::Button("Remove Robot")) {
                    _sim->clearRobot();
                    _hasRobot = false;
                    LOG_INFO("%s removed from scene.", _requestedRobot.c_str());
                    D_INFO("%s removed.", _requestedRobot.c_str());
                }

                if (openRoot)
                {
                    for (int i = 0; i < (int)robot.links.size(); ++i)
                    {
                        const auto& link = robot.links[i];

                        auto* attachedObj = link.attachedObject;
                        bool linkSelected = (_selection.type == SelectionType::LINK && _selection.index == i);

                        // Determine parent link name
                        const bool hasPrevJoint = (i > 0 && (i - 1) < (int)robot.joints.size());
                        const auto* joint = hasPrevJoint ? &robot.joints[i - 1] : nullptr;

                        std::string parentName;

                        if (i == 0) {
                            // Base link: parent is robot root
                            parentName = _requestedRobot; // robot base name if first link (link00)
                        }
                        else if (hasPrevJoint) {
                            // For link i, previous joint connects parent->child
                            parentName = joint->parent; // parent link name
                        }
                        else {
                            parentName = "?";
                        }

                        ImGui::TableNextRow();

                        // Column 0 -> name
                        ImGui::TableSetColumnIndex(0);
                        ImGuiTreeNodeFlags leafFlags =
                            ImGuiTreeNodeFlags_Leaf |
                            ImGuiTreeNodeFlags_NoTreePushOnOpen |
                            ImGuiTreeNodeFlags_DrawLinesToNodes |
                            ImGuiTreeNodeFlags_OpenOnArrow;

                        if (linkSelected) {
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.95f, 0.6f, 1.0f));
                        }

                        ImGui::TreeNodeEx(link.name.c_str(), leafFlags);

                        if (linkSelected) {
                            ImGui::PopStyleColor();
                        }

                        if (ImGui::IsItemClicked() && attachedObj != nullptr) {
                            _currentLinkName = link.name;
                            _selection.type = SelectionType::LINK;
                            _selection.index = i;
                            _selection.source = SelectionSource::CONTROL_PANEL;
                            _sim->setSelectedObject(attachedObj);
                            LOG_INFO("Selected link: %s", link.name.c_str());

                        }

                        // Column 1 -> type
                        ImGui::TableSetColumnIndex(1);
                        ImGui::TextUnformatted("LINK");

                        // Column 2 -> parent
                        ImGui::TableSetColumnIndex(2);
                        if (!parentName.empty())
                            ImGui::TextUnformatted(parentName.c_str());
                        else
                            ImGui::TextDisabled("--");
                    }

                    ImGui::TreePop();
                }
                _currentObjectName = "Robot: " + _requestedRobot;
            }

            // General objects
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Separator();
            ImGui::TextUnformatted("General Objects");

            ImGui::TableSetColumnIndex(1);
            ImGui::TextDisabled("OBJECT");

            ImGui::TableSetColumnIndex(2);
            ImGui::TextDisabled("Actions");

            // General Object Loop
            for (int i = 0; i < objs.size(); i++) {
                auto* obj = objs[i].get();

                bool isSelected = (_selection.type == SelectionType::OBJECT && _selection.index == i);

				if (obj->category != scene::ObjectCategory::General) { continue; } // skip non-general objects

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                std::string label = "Object " + std::to_string(i);

                if (ImGui::Selectable(label.c_str(), isSelected)) {
                    _currentObjectName = label;
                    _selection.type = SelectionType::OBJECT;
                    _selection.index = i;
                    _selection.source = SelectionSource::CONTROL_PANEL;
                    _sim->setSelectedObject(obj); // fine to keep for inspector
                    LOG_INFO("Selected Object: %s", label.c_str());
                }

                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted("OBJECT");

                ImGui::TableSetColumnIndex(2);
                if (ImGui::Button(("Delete##" + std::to_string(i)).c_str())) {
                    indexToDelete = i;
                }
            }

            ImGui::EndTable();
        }

        if (indexToDelete != -1) {
            _sim->deleteObject(indexToDelete);
            LOG_INFO("Deleted object at index %d", indexToDelete);
        }

		ImGui::EndChild();
    }

	// --- HELPER FUNCTIONS ---

	// Begin Control Panel Helper
    void ControlPanel::beginControlPanel(const char* id, ImVec2 size) {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 5.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 8.0f));

        ImGui::BeginChild(id, size, true, ImGuiWindowFlags_AlwaysUseWindowPadding);
    }

	// End Control Panel Helper
    void ControlPanel::endControlPanel() {
        ImGui::EndChild();
        ImGui::PopStyleVar(4);
    }

	// Section Header Helper
    static void SectionHeader(const char* title, const char* desc = nullptr) {
        ImGui::Spacing();
        ImGui::TextUnformatted(title);
        if (desc) {
            ImGui::SameLine();
            ImGui::TextDisabled("%s", desc);
        }
        ImGui::Separator();
        ImGui::Spacing();
    }
    
}