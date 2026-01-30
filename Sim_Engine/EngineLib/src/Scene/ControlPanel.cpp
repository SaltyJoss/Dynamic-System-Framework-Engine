// ==================================================
//				File: ControlPanel.cpp
// ==================================================

#include "pch.h"

#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "Scene/ControlPanel.h"
#include "Robots/RobotSystem.h"
#include <imgui.h>
#include <chrono>

#include "EngineLib/LogMacros.h"

namespace gui {
    ControlPanel::ControlPanel(simManager* sceneView) :
		_sim(sceneView), _controlMode(&sceneView->ctrlMode), _phys(nullptr), _obj(nullptr), _light(nullptr),
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
                _qualityChanged = true;
                q = render::QualityPreset::Low;
            }
            if (ImGui::MenuItem("Quality: Medium", nullptr, q == render::QualityPreset::Medium)) {
                _qualityChanged = true;
                q = render::QualityPreset::Medium;
            }
            if (ImGui::MenuItem("Quality: High", nullptr, q == render::QualityPreset::High)) {
                _qualityChanged = true;
                q = render::QualityPreset::High;
            }
            if (ImGui::MenuItem("Quality: Ultra", nullptr, q == render::QualityPreset::Ultra)) {
				_qualityChanged = true;
                q = render::QualityPreset::Ultra;
            }

            ImGui::Separator();

            if (ImGui::MenuItem("1280x720", nullptr, r == render::ResolutionPreset::R_720p)) {
                _resChanged = true;
				r = render::ResolutionPreset::R_720p;
            }
            if (ImGui::MenuItem("1920x1080", nullptr, r == render::ResolutionPreset::R_1080p)) {
				_resChanged = true;
				r = render::ResolutionPreset::R_1080p;
			}
			if (ImGui::MenuItem("2560x1440", nullptr, r == render::ResolutionPreset::R_1440p)) {
				_resChanged = true;
                r = render::ResolutionPreset::R_1440p;
			}
			if (ImGui::MenuItem("3840x2160", nullptr, r == render::ResolutionPreset::R_4K)) {
                _resChanged = true;
				r = render::ResolutionPreset::R_4K;
			}

            if (_qualityChanged) {
                _sim->applyRenderProfile(render::MakeSettings(r, q), r);
				const render::RenderSettings s;
				LOG_INFO("Render quality changed to %d", (int)q);
                D_INFO("Render quality changed to %d", (int)q);
                _qualityChanged = false;
            }

            if (_resChanged) {
                auto s = render::MakeSettings(r, q);
                _sim->applyRenderProfile(s, r);
                LOG_INFO("Render resolution preset changed to %dx%d", (int)(_sim->getSize().x * s.renderScale), (int)(_sim->getSize().y * s.renderScale));
				D_INFO("Render resolution preset changed to %dx%d", (int)(_sim->getSize().x * s.renderScale), (int)(_sim->getSize().y * s.renderScale));

                _resChanged = false;
			}

            ImGui::EndMenu();
        }

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
        _light = _sim->getLight();
        _hasRobot = _sim->hasRobot();

		_phys = &_sim->getPhysicsSystem();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_FirstUseEver);

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.129f, 0.129f, 0.129f, 0.8f));
        ImGui::Begin("Control Panel", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

        const bool wasRunning = _sim->isSimRunning(); // snapshot

        // Simulation Start/Stop Button
        if (_sim->isSimRunning()) {
		    if (wasRunning && !_sim->isSimRunning()) { _sim->setSimTime(0.0f); } // reset time if just stopped
                LOG_INFO_ONCE("Simulation %s", _sim->isSimRunning() ? "started" : "stopped");
                D_RUNTIME_ONCE("Simulation %s", _sim->isSimRunning() ? "started" : "stopped");
        }

		beginControlPanel("ControlPanel"); // Begin Child Panel

        roboticArmSelector();
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader("Simulation")) {
            simulationProperties();
            jointProperties();
            objectProperties();
			if (_openStats) { stats(); }
        }
        if (ImGui::CollapsingHeader("Display")) { displaySettings(); }

		endControlPanel(); // End Child Panel

        ImGui::End();
        ImGui::PopStyleColor();

        sceneObjectsTable();

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

		ImGui::NewLine();

        ImGui::Text("Integration Method");
        auto& phys = _sim->getPhysicsSystem();
        auto currentEnum = phys.getIntegrationMethod();

        static const char* methodNames[] = { "Euler", "Midpoint", "Heun", "Ralston", "RK4", "RK45"};
        const char* currentMethod = methodNames[static_cast<int>(currentEnum)];
        
		ImGui::SetNextItemWidth(150.0f);
        if (ImGui::BeginCombo("##", currentMethod)) {
            for (int n = 0; n < IM_ARRAYSIZE(methodNames); ++n) {
                bool isSelected = (n == static_cast<int>(currentEnum));

                if (ImGui::Selectable(methodNames[n], isSelected)) {
                    auto updatedMethod = static_cast<integration::eIntegrationMethod>(n);
                    phys.setIntegrationMethod(updatedMethod);

                    switch (updatedMethod) {
                    case integration::eIntegrationMethod::Euler:
                        D_INFO("Integrator set to Euler"); break;
                    case integration::eIntegrationMethod::Midpoint:
                        D_INFO("Integrator set to RK2 (Midpoint)"); break;
                    case integration::eIntegrationMethod::Heun:
                        D_INFO("Integrator set to RK2 (Heun)"); break;
                    case integration::eIntegrationMethod::Ralston:
                        D_INFO("Integrator set to RK2 (Ralston)"); break;
                    case integration::eIntegrationMethod::RK4:
                        D_INFO("Integrator set to RK4"); break;
                    case integration::eIntegrationMethod::RK45:
						D_INFO("Integrator set to RK45 (Dormand-Prince)"); break;
                    default:
                        break;
                    }
                }
                if (isSelected) { ImGui::SetItemDefaultFocus(); }
            }
            ImGui::EndCombo();
        }

        float step = 0.001f;
        float stepFast = 0.01f;

		ImGui::BeginDisabled(_sim->isSimRunning());
        ImGui::Text("Delta Time (dt)");
        ImGui::SetNextItemWidth(150.0f);
        ImGui::InputScalar("seconds##dt", ImGuiDataType_Float, &deltaTime, &step, &stepFast, "%.5f");
		ImGui::EndDisabled();

        ImGui::Text("Current dt: %.5f seconds", _sim->getFixedDeltaTime());

        // Deals with simulation time tracking using chrono
        if (_sim->isSimRunning()) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Simulation Running...");
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Elapsed Time: %.3f", _sim->getSimTime());

			// make sure to stop sim when commands are finished
            if (!_sim->isSimRunning()) {
                ImGui::Text("Simulation Stopped.");
                ImGui::Text("Elapsed Time: %.3f", _sim->getSimTime());
            }
        }

		ImGui::Separator();
    }

    void ControlPanel::objectProperties() {
        if (_sim->isSimRunning()) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Cannot edit object properties while simulation is running.");
            ImGui::Separator();
            return;
		}
        if (!_obj && !_hasRobot) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No object selected.");
            ImGui::Separator();
            return;
        }

        ImGui::SeparatorText("Physics Settings:");

        ImGui::Separator();

        double minMass = 0.0;    double maxMass = 100.0;   // mass limits
		float minDamping = 0.0;  float maxDamping = 1.0;   // damping limits
        float minFriction = 0.0; float maxFriction = 10.0; // friction limits
		double minGravity = 0.0; double maxGravity = 10.0; // gravity limits

        if (_hasRobot) {
            robots::RobotSystem* robot = _sim->getRobotSystem();
			auto& links = robot->links();
            auto& joints = robot->joints();

            static int currentJointIndex = 0;
            currentJointIndex = std::clamp(currentJointIndex, 0, (int)joints.size() - 1);

            for (int i = 0; i < (int)joints.size(); ++i) {
                if (joints[i].name == _currentJointName) { currentJointIndex = i; break; }
            }

			auto& j = joints[currentJointIndex];

            int linkIndex = currentJointIndex;
            linkIndex = std::clamp(linkIndex, 0, (int)links.size() - 1);
            auto& L = links[linkIndex];

            float c = (float)j.dynamics.damping;
            float f = (float)j.dynamics.friction;
            double g = (double)robot->getGravity();
			
			ImGui::BeginDisabled(_sim->isSimRunning());

            ImGui::Text("Selected Joint: %s - Child Link: %s", j.name.c_str(), L.name.c_str());
            ImGui::Text("Joint Angle: %.3f - Link Mass: %.3f kg", glm::degrees(j.thetaRad), L.inertial.mass);
            ImGui::Spacing();

            ImGui::Text("Damping:");
            ImGui::SetNextItemWidth(150.0f);
            if (ImGui::DragFloat("kg/s##damp", &c, 0.001f, minDamping, maxDamping)) { j.dynamics.damping = c; }
            ImGui::Spacing();

            ImGui::Text("Friction:");
            ImGui::SetNextItemWidth(150.0f);
            if (ImGui::DragFloat("##fric", &f, 0.001f, minFriction, maxFriction)) { j.dynamics.friction = f;  }
            ImGui::Spacing();

            ImGui::Text("Gravity:");
            ImGui::SetNextItemWidth(150.0f);
            if (ImGui::DragScalar("m/s^2##g", ImGuiDataType_Double, &g, 0.00005f, &minGravity, &maxGravity)) {
                robot->setGravity(g); // you need a setter
            }
            ImGui::Spacing();

			ImGui::EndDisabled();

            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Note: Some object properties are locked for individual robot joints and links.");
            ImGui::Separator();
        }
        else if (!_sim->isSimRunning()) {
            minMass = 0.25; maxMass = 100.0;
            minDamping = 0.0; maxDamping = 1.0;

            ImGui::BeginDisabled(_sim->isSimRunning());

            ImGui::Text("Mass:");
            ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("kg##mass", ImGuiDataType_Double, &_obj->state.mass, 0.025f, &minMass, &maxMass);
            ImGui::Spacing();

            ImGui::Text("Damping:");
            ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("kg/s##damp", ImGuiDataType_Double, &_obj->state.damping, 0.001f, &minDamping, &maxDamping);
            ImGui::Spacing();

			ImGui::Text("Gravity:");
			ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("m/s^2##g", ImGuiDataType_Double, &_obj->state.gravity, 0.00005f, &minGravity, &maxGravity);
            ImGui::Spacing();

			ImGui::Separator();

            if (_obj->category == scene::ObjectCategory::General) {
                ImGui::SetNextItemWidth(150.0f);
                // Scale Controls
                ImGui::Text("Scale:");
                float minScale = 0.0001f; float maxScale = 100.0f;
                ImGui::SetNextItemWidth(150.0f); ImGui::DragFloat("(x)", &_obj->transform.scale.x, 0.001f, minScale, maxScale);
                ImGui::SetNextItemWidth(150.0f); ImGui::DragFloat("(y)", &_obj->transform.scale.y, 0.001f, minScale, maxScale);
                ImGui::SetNextItemWidth(150.0f); ImGui::DragFloat("(z)", &_obj->transform.scale.z, 0.001f, minScale, maxScale);

                ImGui::Spacing();

                // Linear Velocity Controls
                ImGui::Text("Linear Velocity:");
                double minVelocity = -100.0; double maxVelocity = 100.0;
                ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("X##linVelX", ImGuiDataType_Double, &_obj->state.linearVelocity.x(), 0.0025f, &minVelocity, &maxVelocity);
                ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("Y##linVelY", ImGuiDataType_Double, &_obj->state.linearVelocity.y(), 0.0025f, &minVelocity, &maxVelocity);
                ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("Z##linVelZ", ImGuiDataType_Double, &_obj->state.linearVelocity.z(), 0.0025f, &minVelocity, &maxVelocity);

                ImGui::Spacing();

                // Angular Velocity Controls
                ImGui::Text("Angular Velocity:");
                double minTorque = -100.0; double maxTorque = 100.0;
                ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("X##angVelX", ImGuiDataType_Double, &_obj->state.angularVelocity.x(), 0.0025f, &minTorque, &maxTorque);
                ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("Y##angVelY", ImGuiDataType_Double, &_obj->state.angularVelocity.y(), 0.0025f, &minTorque, &maxTorque);
                ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("Z##angVelZ", ImGuiDataType_Double, &_obj->state.angularVelocity.z(), 0.0025f, &minTorque, &maxTorque);

                ImGui::Separator();

				ImGui::EndDisabled();
            }
        }

        ImGui::Text("Reset Object:");
        // Reset Object Button
        if (ImGui::Button("Reset")) {
            if (!_obj) {
                LOG_WARN("No object selected to reset.");
                return;
			}

            if (_hasRobot) {
				_sim->getRobotSystem()->resetRobot();
                LOG_INFO("Robot reset to initial position and orientation.");
                D_INFO("Reset Robot");
				return;
			}

            _obj->reset();
            LOG_INFO("Object reset to initial position and orientation.");
            D_INFO("Reset %s", _obj);
        }

		ImGui::Separator();
    }

    void ControlPanel::jointProperties() {
        if (!_hasRobot) return;

        robots::RobotSystem* robot = _sim->getRobotSystem();
        if (!robot || !robot->hasRobot()) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No robot model loaded.");
            ImGui::Separator();
            return;
        }

        const auto& joints = robot->joints();
        if (joints.empty()) {
            ImGui::TextDisabled("Robot has no joints.");
            ImGui::Separator();
            return;
        }

        static int currentJointIndex = 0;
        currentJointIndex = std::clamp(currentJointIndex, 0, (int)joints.size() - 1);

        static float minAngleDeg = -180.0f;
        static float maxAngleDeg = 180.0f;

        // Auto-select first joint if nothing selected yet
        if (_currentJointName.empty()) {
            _currentJointName = joints[currentJointIndex].name;
            _currentLinkName = joints[currentJointIndex].child;
            minAngleDeg = glm::degrees(joints[currentJointIndex].limits.minAngle);
            maxAngleDeg = glm::degrees(joints[currentJointIndex].limits.maxAngle);
        }

        const char* preview = _currentJointName.c_str();

        if (!_sim->isSimRunning()) {
            ImGui::SetNextItemWidth(150.0f);
            if (ImGui::BeginCombo("Joint", preview)) {
                for (int i = 0; i < (int)joints.size(); ++i) {
                    const bool selected = (i == currentJointIndex);
                    if (ImGui::Selectable(joints[i].name.c_str(), selected)) {
                        currentJointIndex = i;
                        _currentJointName = joints[i].name;
                        _currentLinkName = joints[i].child;

                        minAngleDeg = glm::degrees(joints[i].limits.minAngle);
                        maxAngleDeg = glm::degrees(joints[i].limits.maxAngle);
                    }
                    if (selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        }

        float angleRad = 0.0f;
        if (!robot->tryGetJointAngleRad(_currentLinkName, angleRad)) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Failed to get joint angle for link: %s", _currentLinkName.c_str());
            return;
        }

        ImGui::TextDisabled("Limits: [%.1f°, %.1f°]", minAngleDeg, maxAngleDeg);

        float angleDeg = glm::degrees(angleRad);
        if (ImGui::DragFloat("Angle (deg)", &angleDeg, 0.1f, minAngleDeg, maxAngleDeg)) {
            // strongly prefer radians API:
            robot->trySetJointAngleRad(_currentLinkName, glm::radians(angleDeg));
        }

        if (ImGui::Button("Reset Joint")) {
			robot->trySetJointAngleRad(_currentLinkName, 0.0f);
            LOG_INFO("Joint %s reset to 0 degrees.", joints[currentJointIndex].name.c_str());
			D_INFO("Reset Joint %s", joints[currentJointIndex].name.c_str());
        }

		ImGui::SetNextItemWidth(150.0f);
        if (ImGui::Checkbox("Enable Statistics", &_openStats)) { D_INFO("Statistics %s.", _openStats ? "enabled" : "disabled"); }
    }

    void ControlPanel::stats() {
        if (!_openStats && !_obj && !_hasRobot) { return; }

        ImGui::SeparatorText("Simulation Statistics");
        if (_obj && _obj->getMesh()) {
            const glm::vec3& pos = _obj->transform.position;
            const glm::quat& rot = _obj->transform.rotQ;

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

                ImGui::Separator();
            }

            if (_hasRobot) {
                robots::RobotSystem* robot = _sim->getRobotSystem();
                if (robot) {
                    ImGui::Separator();
                    ImGui::Text("Robot Joint Angles:");
                    ImGui::Separator();
                    const auto& joints = robot->joints();
                    for (const auto& joint : joints) {
                        float angleDeg = glm::degrees(joint.thetaRad);
                        ImGui::Text("%s: %.2f deg", joint.name.c_str(), angleDeg);

                        // Plot Outputs specific to robotic arm
						ImGui::Text("Joint Angle History - %s", joint.name.c_str());
                        static std::vector<float> jointAngleHistory;
                        jointAngleHistory.push_back(angleDeg);
                        if (jointAngleHistory.size() > 100) jointAngleHistory.erase(jointAngleHistory.begin());
						ImGui::PlotLines(("##" + joint.name + "_angle_plot").c_str(), jointAngleHistory.data(), (int)jointAngleHistory.size(), 0, nullptr, -180.0f, 180.0f, ImVec2(0, 25));
                    }
                }
            }

            ImGui::Text("Telemetry");
			ImGui::Separator();
            // Position block
			if (ImGui::BeginTable("telemetryTable", 2, ImGuiTableFlags_BordersInnerV)) {
				// Position
				ImGui::BeginDisabled(_hasRobot); // disable position display for robot joints
				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0); ImGui::Text("Position (m)");
				ImGui::TableSetColumnIndex(1); ImGui::Text("X: %.3f  Y: %.3f  Z: %.3f", pos.x, pos.y, pos.z);

				glm::vec3 eulerDeg = glm::degrees(glm::eulerAngles(rot)); // convert quaternion to Euler angles in degrees

				// Rotation
				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0); ImGui::Text("Rotation (deg)");
				ImGui::TableSetColumnIndex(1); ImGui::Text("Pitch: %.1f  Yaw: %.1f  Roll: %.1f", eulerDeg.x, eulerDeg.y, eulerDeg.z);
                ImGui::EndDisabled();

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
    }

    void ControlPanel::displaySettings() {
        ImGui::SeparatorText("Display Settings");
        static float fovDeg = 70.0f;
        ImGui::BeginDisabled(_sim->isSimRunning());
        bool edited = ImGui::SliderFloat("Field of View", &fovDeg, 25.0f, 125.0f, "%.1f");
        bool active = ImGui::IsItemActive();

        if (!active && !edited) { fovDeg = _sim->getCamera()->getFOVDegrees(); }
        if (edited) { _sim->getCamera()->setFOVDegrees(fovDeg); }
        ImGui::EndDisabled();
    }

	// Robotic Arm Selector
    void ControlPanel::roboticArmSelector() {
        if (!_showRobotSelector) return;

        ImGui::Begin("Choose Robotic Arm", &_showRobotSelector, ImGuiWindowFlags_NoDocking);

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

        ImGui::BeginChild("robot_card", ImVec2(0, 55), true, ImGuiWindowFlags_NoScrollbar);

        // Loads robot
        if (ImGui::Selectable(name, false, ImGuiSelectableFlags_AllowDoubleClick)) {
            LOG_INFO("Selected robot: %s", name);
            _showRobotSelector = false;

            _requestedRobot = name;
            _robotRequested = true;

            _sim->loadRobot(_requestedRobot);
			_hasRobot = true;
        }

        ImGui::TextDisabled("Company: %s", company);

        ImGui::EndChild();
        ImGui::Spacing();

        ImGui::PopID();
	}

	// Scene Objects List
	void ControlPanel::sceneObjectsTable() {
        ImGui::SetNextWindowPos(ImVec2(0, 250), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_FirstUseEver);

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.129f, 0.129f, 0.129f, 0.8f));
		ImGui::Begin("SceneObjectsTable", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

        beginControlPanel("Rigid Body Table");

		auto& objs = _sim->getObjects();
		int indexToDelete = -1;

		ImGui::Text("Active Rigid-Bodies");
		ImGui::Separator();

		ImGuiTableFlags tableFlags =
			ImGuiTableFlags_BordersV |
			ImGuiTableFlags_BordersOuterH |
			ImGuiTableFlags_Resizable |
			ImGuiTableFlags_RowBg |
			ImGuiTableFlags_NoBordersInBody;

		if (ImGui::BeginTable("SceneTable", 3, tableFlags)) {
			ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_NoHide);
			ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed);
			ImGui::TableSetupColumn("Child / Action");
			ImGui::TableHeadersRow();

			float rowH = 20.0f;

			// Robot section
			if (_hasRobot) {
				robots::RobotSystem* robotSys = _sim->getRobotSystem();
				if (robotSys && robotSys->hasRobot()) {
					const auto& links = robotSys->links();
					const auto& joints = robotSys->joints();

					// Root label
					std::string rootName = robotSys->robotName(); // or robotSys->robotName()
					if (rootName.empty()) rootName = "Robot";

					ImGui::TableNextRow(ImGuiTableRowFlags_None, rowH);
					ImGui::TableSetColumnIndex(0);

					ImGuiTreeNodeFlags rootFlags =
						ImGuiTreeNodeFlags_SpanAllColumns |
						ImGuiTreeNodeFlags_OpenOnArrow;

					bool openRoot = ImGui::TreeNodeEx(rootName.c_str(), rootFlags);

					bool rowHovered = ImGui::IsItemHovered();
					if (rowHovered) { ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImGuiCol_HeaderHovered)); }

					// Column 1: centered "ROOT"
					ImGui::TableSetColumnIndex(1);
					{
						const char* txt = "ROOT";
						float columnWidth = ImGui::GetColumnWidth();
						float textWidth = ImGui::CalcTextSize(txt).x;
						ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (columnWidth - textWidth) * 0.5f);
						ImGui::TextUnformatted(txt);
					}

					// Column 2: centered Remove button
					ImGui::TableSetColumnIndex(2);
					{
						float columnWidth = ImGui::GetColumnWidth();
						float buttonWidth = 80.0f;
						ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (columnWidth - buttonWidth) * 0.5f);

						if (ImGui::Button(("Remove##robot_" + rootName).c_str(), ImVec2(buttonWidth, 0))) {
							_sim->clearRobot();
							_hasRobot = false;
						}
					}

					if (openRoot) {
						for (int i = 0; i < (int)joints.size(); ++i) {
							auto& joint = joints[i];
							scene::Object* attachedObj = nullptr;
							rowH = 10.0f;

							// Find attached object for this joint's child link
							for (auto& l : links) { if (l.name == joint.child) { attachedObj = l.attachedObject; break; } }

							bool jointSelected = (_selection.type == SelectionType::JOINT && _selection.index == i);

							ImGui::TableNextRow(ImGuiTableRowFlags_None, rowH);
							ImGui::TableSetColumnIndex(0);

							ImGui::PushID(i);

							bool rowClicked = ImGui::Selectable("##joint_row", jointSelected,
								ImGuiSelectableFlags_SpanAllColumns, ImVec2(0.0f, rowH)
							);

							rowHovered = ImGui::IsItemHovered();
							if (rowHovered) { ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImGuiCol_HeaderHovered)); }

							ImGui::SameLine(0.0f, 6.0f);

							ImGuiTreeNodeFlags leafFlags = ImGuiTreeNodeFlags_Leaf
								| ImGuiTreeNodeFlags_NoTreePushOnOpen
								| ImGuiTreeNodeFlags_NoAutoOpenOnLog
								| ImGuiTreeNodeFlags_SpanAllColumns
								| ImGuiTreeNodeFlags_NoTreePushOnOpen;

							ImGui::TreeNodeEx("##leaf", leafFlags);

							// Center joint name *within column 0*
							{
								float columnWidth = ImGui::GetColumnWidth();
								float textWidth = ImGui::CalcTextSize(joint.name.c_str()).x;
								ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (columnWidth - textWidth) * 0.5f);
							}

							ImGui::SameLine();

							if (jointSelected) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.95f, 0.6f, 1.0f));
							ImGui::TextUnformatted(joint.name.c_str());
                            if (ImGui::IsItemHovered()) {
                                ImGui::BeginTooltip();
                                ImGui::TextDisabled("Angle(deg): %.2f\nOmega(rad/s): %.2f\nk_p: %.2f\nk_d: %.2f\nDamping: %.2f\nFriction: %.2f",
                                    glm::degrees(joint.thetaRad), joint.omegaRad_s, joint.k_p, joint.k_d, joint.dynamics.damping, joint.dynamics.friction);

                                static std::vector<float> jointAngleHistory;
                                jointAngleHistory.push_back(glm::degrees(joint.thetaRad));
                                if (jointAngleHistory.size() > 100) jointAngleHistory.erase(jointAngleHistory.begin());
                                ImGui::PlotLines(("##" + joint.name + "_angle_plot").c_str(), jointAngleHistory.data(), (int)jointAngleHistory.size(), 0, nullptr, -180.0f, 180.0f, ImVec2(0, 25));
                                ImGui::EndTooltip();
                            }
							if (jointSelected) ImGui::PopStyleColor();

							if (rowClicked) {
								_currentJointName = joint.name;
								_selection.type = SelectionType::JOINT;
								_selection.index = i;
								_selection.source = SelectionSource::CONTROL_PANEL;
								if (attachedObj) { _sim->setSelectedObject(attachedObj); }
							}

							ImGui::PopID();

							// Column 1: centered "Joint"
							ImGui::TableSetColumnIndex(1);
							{
								const char* txt = "Joint";
								float columnWidth = ImGui::GetColumnWidth();
								float textWidth = ImGui::CalcTextSize(txt).x;
								ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (columnWidth - textWidth) * 0.5f);
								ImGui::TextUnformatted(txt);
							}

							// Column 2: centered child link name
							ImGui::TableSetColumnIndex(2);
							{
								float columnWidth = ImGui::GetColumnWidth();
								float textWidth = ImGui::CalcTextSize(joint.child.c_str()).x;
								ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (columnWidth - textWidth) * 0.5f);
								ImGui::TextUnformatted(joint.child.c_str());
							}
						}
						ImGui::TreePop();
					}
					_currentObjectName = "Robot: " + rootName;
				}
			}

			// General objects
			rowH = 20.0f;
			ImGui::TableNextRow(ImGuiTableRowFlags_None, rowH);

			// General Object Loop
			for (int i = 0; i < objs.size(); i++) {
				auto* obj = objs[i].get();
				rowH = 10.0f;
				bool isSelected = (_selection.type == SelectionType::OBJECT && _selection.index == i);

				if (obj->category != scene::ObjectCategory::General) { continue; } // skip non-general objects

				ImGui::TableNextRow(ImGuiTableRowFlags_None, rowH);
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

				// Column 1: centered "OBJECT"
				ImGui::TableSetColumnIndex(1);
				{
					const char* txt = "OBJECT";
					float columnWidth = ImGui::GetColumnWidth();
					float textWidth = ImGui::CalcTextSize(txt).x;
					ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (columnWidth - textWidth) * 0.5f);
					ImGui::TextUnformatted(txt);
				}

				// Column 2: centered Delete button
				ImGui::TableSetColumnIndex(2);
				{
					float columnWidth = ImGui::GetColumnWidth();
					float buttonWidth = 80.0f;
					ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (columnWidth - buttonWidth) * 0.5f);
					if (ImGui::Button(("Delete##" + std::to_string(i)).c_str())) { indexToDelete = i; }
				}
			}

            ImGui::EndTable();
		}

		if (indexToDelete != -1) {
			_sim->deleteObject(indexToDelete);
			LOG_INFO("Deleted object at index %d", indexToDelete);
		}

		endControlPanel();

		ImGui::End();
        ImGui::PopStyleColor();
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
}