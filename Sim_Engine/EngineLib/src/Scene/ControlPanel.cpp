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

#include <implot.h>

#include "EngineLib/LogMacros.h"

namespace gui {
	// --- Helper Functions ---

	// Segmented Button Row Helper
    static bool SegmentedButtonRow(const char* label, const char* const* items, int itemCount, int& current, float buttonWidth) {
        ImGui::TextUnformatted(label);

        bool changed = false;
        ImGui::PushID(label);

        for (int i = 0; i < itemCount; ++i) {
            if (i > 0) ImGui::SameLine();

            const bool selected = (current == i);
            if (selected) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.2f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.7f, 0.3f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.1f, 0.5f, 0.1f, 1.0f));
            }

            if (ImGui::Button(items[i], ImVec2(buttonWidth, 0))) {
                if (current != i) {
                    current = i;
                    changed = true;
                }
            }

            if (selected) {
                ImGui::PopStyleColor(3);
            }
        }

        ImGui::PopID();
        return changed;
    }

    // Helper to build telemetry series
    static void buildSeries(const diagnostics::TelemetryRing& ring, std::vector<float>& out, std::function<float(const diagnostics::TelemetrySample&)> f) {
        out.resize(ring.size());
        for (size_t i = 0; i < ring.size(); ++i) {
            out[i] = f(ring.at(i));
        }
    }

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

	// --- ControlPanel Implementation ---

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

		if (ImGui::BeginTabBar("ControlPanelTabs")) {
            if (ImGui::BeginTabItem("Simulation Properties")) {
                simulationProperties();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Rigid Body Properties")) {
                objectProperties();
                ImGui::EndTabItem();
			}
			if (ImGui::BeginTabItem("Multi-Body Properties")) {
                jointProperties();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Display Settings")) {
                displaySettings();
                ImGui::EndTabItem();
			}
            // Additional tabs can be added here
            ImGui::EndTabBar();
        }

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

		ImGui::Spacing();

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
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Cannot edit object properties while simulation is running.");
            return;
		}
        if (!_obj) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No object selected.");
            return;
        }

        ImGui::SeparatorText("Physics Settings:");

        ImGui::Separator();

        double minMass    = 0.25; double maxMass    = 100.0; // mass limits
        float minDamping  =  0.0; float maxDamping  =   1.0; // damping limits
        double minGravity =  0.0; double maxGravity =  10.0; // gravity limits

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

        ImGui::Text("Reset Object:");
        // Reset Object Button
        if (ImGui::Button("Reset")) {
            if (!_obj) {
                LOG_WARN("No object selected to reset.");
                return;
			}

            _obj->reset();
            LOG_INFO("Object reset to initial position and orientation.");
            D_INFO("Reset %s", _obj);
        }

		ImGui::Separator();
    }

    void ControlPanel::jointProperties() {
        if (!_hasRobot) { ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No robot model loaded."); return; }
        robots::RobotSystem* robot = _sim->getRobotSystem();

		auto& links = robot->links();
		auto& joints = robot->joints();
        if (joints.empty()) {
            ImGui::TextDisabled("Robot has no joints.");
            return;
        }

        float minDamping  = 0.0; float maxDamping  = 1.0;   // damping limits
        float minFriction = 0.0; float maxFriction = 10.0;  // friction limits
        double minGravity = 0.0; double maxGravity = 10.0;  // gravity limits

        static int currentJointIndex = 0;
        currentJointIndex = std::clamp(currentJointIndex, 0, (int)joints.size() - 1);

		auto& j = joints[currentJointIndex];
		int linkIndex = currentJointIndex;
		linkIndex = std::clamp(linkIndex, 0, (int)links.size() - 1);
		auto& L = links[linkIndex];

		float c = (float)j.dynamics.damping;
		float f = (float)j.dynamics.friction;
		double g = (double)robot->getGravity();

		// Trajectory Inspector
		ImGui::Text("Joint Telemetry:");
        ImGui::Text("Selected Joint: %s - Child Link: %s", j.name.c_str(), L.name.c_str());
        ImGui::Spacing();

        const auto& rec = _sim->telemetry();
        drawTelemetryPlots(rec);

        ImGui::BeginDisabled(_sim->isSimRunning());

        ImGui::Text("Joint Dynamics:");

        ImGui::Text("Damping:");
        ImGui::SetNextItemWidth(150.0f);
        if (ImGui::DragFloat("kg/s##damp", &c, 0.001f, minDamping, maxDamping)) { j.dynamics.damping = c; }
        ImGui::Spacing();

        ImGui::Text("Friction:");
        ImGui::SetNextItemWidth(150.0f);
        if (ImGui::DragFloat("##fric", &f, 0.001f, minFriction, maxFriction)) { j.dynamics.friction = f; }
        ImGui::Spacing();

        ImGui::Text("Gravity:");
        ImGui::SetNextItemWidth(150.0f);
        if (ImGui::DragScalar("m/s^2##g", ImGuiDataType_Double, &g, 0.00005f, &minGravity, &maxGravity)) {
            robot->setGravity(g); // you need a setter
        }
        ImGui::Spacing();
        ImGui::Separator();

        ImGui::EndDisabled();

        drawTrajectoryInspector(rec, (int)robot->joints().size(), _selection.index);

		ImGui::Spacing();
		ImGui::Text("Reset Robot:");
        ImGui::Spacing();

        if (ImGui::Button("Reset")) {
            if (!_hasRobot) { LOG_WARN("No robot selected to reset."); return; } // should not happen

			_sim->getRobotSystem()->resetRobot();
			LOG_INFO("Robot reset to initial position and orientation.");
			D_INFO("Reset Robot to initial position and orientation.");
			return;
        }
    }

    void ControlPanel::displaySettings() {
        static float fovDeg = 70.0f;
        ImGui::BeginDisabled(_sim->isSimRunning());

		ImGui::Text("Camera Field of View (FOV):");
		
        ImGui::SetNextItemWidth(150.0f);
        bool edited = ImGui::SliderFloat("Field of View", &fovDeg, 25.0f, 125.0f, "%.f");
        bool active = ImGui::IsItemActive();

        if (!active && !edited) { fovDeg = _sim->getCamera()->getFOVDegrees(); }
        if (edited) { _sim->getCamera()->setFOVDegrees(fovDeg); }
        ImGui::EndDisabled();

        ImGui::Spacing();

		// Graphics Quality Presets
        static int graphicsIndx = 3;
		const char* qualityOptions[] = { "Low", "Medium", "High", "Ultra" };

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, ImGui::GetStyle().ItemSpacing.y));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);

		// Quality Buttons
        bool qualityChanged = SegmentedButtonRow("Graphics Settings:", qualityOptions, IM_ARRAYSIZE(qualityOptions), graphicsIndx, 70.0f);

		ImGui::PopStyleVar(2);

		// Apply quality changes if needed
        if (qualityChanged) {
            switch (graphicsIndx) {
                case 0: q = render::QualityPreset::Low;    break;
                case 1: q = render::QualityPreset::Medium; break;
                case 2: q = render::QualityPreset::High;   break;
                case 3: q = render::QualityPreset::Ultra;  break;
                default: break;
            }

			auto s = render::MakeSettings(r, q);
			_sim->applyRenderProfile(s, r);

            LOG_INFO("Render quality preset changed to %s",
                graphicsIndx == 0 ? "Low" :
                graphicsIndx == 1 ? "Medium" :
                graphicsIndx == 2 ? "High" : "Ultra");
            D_INFO("Render quality preset changed to %s",
                graphicsIndx == 0 ? "Low" :
                graphicsIndx == 1 ? "Medium" :
				graphicsIndx == 2 ? "High" : "Ultra");
        }

		ImGui::Spacing();

		// Resolution Presets
		static int resIndx = 3;
		const char* resOptions[] = { "1280x720", "1920x1080", "2560x1440", "3840x2160" };

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, ImGui::GetStyle().ItemSpacing.y));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);

		// Resolution Buttons
        bool resChanged = SegmentedButtonRow("Resolution Presets:", resOptions, IM_ARRAYSIZE(resOptions), resIndx, 90.0f);

		ImGui::PopStyleVar(2);

		// Apply resolution changes if needed
		if (resChanged) {
            switch (resIndx) {
                case 0: r = render::ResolutionPreset::R_720p;   break;
                case 1: r = render::ResolutionPreset::R_1080p;  break;
                case 2: r = render::ResolutionPreset::R_1440p;  break;
                case 3: r = render::ResolutionPreset::R_4K;     break;
                default: break;
            }
                
            auto s = render::MakeSettings(r, q);
            _sim->applyRenderProfile(s, r);

            LOG_INFO("Render resolution preset changed to %dx%d",
                (int)(_sim->getSize().x * s.renderScale),
                (int)(_sim->getSize().y * s.renderScale));

            D_INFO("Render resolution preset changed to %dx%d",
                (int)(_sim->getSize().x * s.renderScale),
                (int)(_sim->getSize().y * s.renderScale));
		}

        ImGui::Spacing();

		// Shader Mode Selector
		static int shaderIndx = 2;
		const char* shaderOptions[] = { "Basic Shader", "Lit Shader", "PBR Shader" };

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, ImGui::GetStyle().ItemSpacing.y));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);

		// Shader Buttons
        bool shaderChanged = SegmentedButtonRow("Shader Mode:", shaderOptions, IM_ARRAYSIZE(shaderOptions), shaderIndx, 110.0f);

		ImGui::PopStyleVar(2);

		// Apply shader changes if needed
        if (shaderChanged) {
            switch (shaderIndx) {
                case 0: _sim->currentShaderMode = simManager::ShaderMode::Basic; D_INFO("Shader -> Basic Shader"); break;
                case 1: _sim->currentShaderMode = simManager::ShaderMode::Lit;   D_INFO("Shader -> Lit Shader");   break;
                case 2: _sim->currentShaderMode = simManager::ShaderMode::PBR;   D_INFO("Shader -> PBR Shader");   break;
                default: break;
            }
        }

        if (ImGui::Button("Reload Shaders")) {
            LOG_INFO("Shader reload requested.");
            _sim->reloadAllShaders();
        }
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
                                ImGui::TextDisabled("Angle(rad): %.2f\nOmega(rad/s): %.2f\nk_p: %.2f\nk_d: %.2f\nDamping: %.2f\nFriction: %.2f",
                                    joint.thetaRad, joint.omegaRad_s, joint.k_p, joint.k_d, joint.dynamics.damping, joint.dynamics.friction);
                                ImGui::EndTooltip();
                            }
							if (jointSelected) ImGui::PopStyleColor();

                            if (rowClicked) {
                                _currentJointName = joint.name;
                                _selection.type = SelectionType::JOINT;
                                _selection.index = i;
                                _selection.source = SelectionSource::CONTROL_PANEL;
                                if (attachedObj) { _sim->setSelectedObject(attachedObj); }

                                _sim->followRobotJoint(_currentJointName, glm::vec3(0.0f, 0.2f, 0.6f));
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

    void ControlPanel::selectJointAndFollow(int jointIdx)
    {
        if (!_sim || !_sim->hasRobot()) return;

        robots::RobotSystem* robot = _sim->getRobotSystem();
        if (!robot) return;

        auto& joints = robot->joints();
        auto& links = robot->links();
        if (joints.empty()) return;

        jointIdx = std::clamp(jointIdx, 0, (int)joints.size() - 1);

        const auto& joint = joints[jointIdx];
        _currentJointName = joint.name;

        _selection.type = SelectionType::JOINT;
        _selection.index = jointIdx;
        _selection.source = SelectionSource::CONTROL_PANEL;

        // find attached object for child link
        scene::Object* attachedObj = nullptr;
        for (auto& l : links) {
            if (l.name == joint.child) { attachedObj = l.attachedObject; break; }
        }
        if (attachedObj) {
            _sim->setSelectedObject(attachedObj);
        }

        // camera follow
        _sim->followRobotJoint(_currentJointName, glm::vec3(0.0f, 0.2f, 0.6f));
    }

    static void computeWindow(int total, int windowN, int& start, int& count) {
        count = std::min(windowN, total);
        start = std::max(0, total - count);
    }

    void ControlPanel::drawTelemetryPlots(const diagnostics::TelemetryRecorder& rec) {
        const auto& ring = rec.ring;
        if (ring.size() < 2) { ImGui::TextUnformatted("No telemetry plots yet."); return; }

		// Sim samples selector
		static int windowN = 600; // default to 10 seconds at 60Hz
        ImGui::SetNextItemWidth(140.0f);
        ImGui::SliderInt("Window (samples)", &windowN, 50, (int)ring.size());
        ImGui::SameLine(); ImGui::TextDisabled("(%0.1fs @60Hz)", windowN / 60.0f);

		// Build series
        static std::vector<float> rms, mx, cs; // root mean square, max, clamp sum
        buildSeries(ring, rms, [](const diagnostics::TelemetrySample& s) { return s.err_rms; });
        buildSeries(ring, mx,  [](const diagnostics::TelemetrySample& s) { return s.err_max; });
        buildSeries(ring, cs,  [](const diagnostics::TelemetrySample& s) { return (float)s.clamp_sum; });

        // Latest values
        const auto& last = ring.at(ring.size() - 1);
        const float lastRms = rms.back();
        const float lastMax = mx.back();
        const float lastCs = cs.back();

        // Compact “stats row”
        ImGui::Text("Samples: %zu / %zu", ring.size(), ring.capacity());
        ImGui::SameLine(); ImGui::TextDisabled("t=%.3fs", last.timeSec);

        ImGui::Spacing();

        // Build per-joint error series
        const int sampleCount = (int)ring.size();
        const int jointCount = (int)ring.at(sampleCount - 1).j.size();

		// Prepare data arrays
        static std::vector<float> x;
        static std::vector<std::vector<float>> y;

		// Resize time array
        x.resize(sampleCount);

		// Resize joint error arrays
        if ((int)y.size() != jointCount) y.resize(jointCount);
        for (int j = 0; j < jointCount; ++j) y[j].resize(sampleCount);

		// Fill data arrays
        for (int k = 0; k < sampleCount; ++k) {
            const auto& s = ring.at(k);
            x[k] = (float)s.timeSec;
            const int m = std::min(jointCount, (int)s.j.size());
            for (int j = 0; j < m; ++j) {
                y[j][k] = s.j[j].thetaRefRad - s.j[j].thetaRad;
            }
            for (int j = m; j < jointCount; ++j) {
                y[j][k] = 0.0f;
            }
        }

		// Determine plot window
        int start = 0, count = 0;
        windowN = std::clamp(windowN, 1, (int)ring.size());
        computeWindow((int)ring.size(), windowN, start, count);
        const ImVec2 plotSz(-1, 200); 

		// ---------- Draw Plots ----------

		// RMS & Error Max
        if (ImPlot::BeginPlot("Error Plot (RMS, Max)", plotSz)) {
            ImPlot::SetupAxes("t (s)", "error (rad)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_None);
            ImPlot::PlotLine("RMS", x.data() + start, rms.data() + start, count);
            ImPlot::PlotLine("Max", x.data() + start, mx.data() + start, count);
            ImPlot::EndPlot();
        }

        // Clamp Sum
        if (ImPlot::BeginPlot("Clamp Events", plotSz)) {
            ImPlot::SetupAxes("t (s)", "Clamp Sum", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_None);
            ImPlot::PlotStairs("Sum", x.data() + start, cs.data() + start, count);
            ImPlot::EndPlot();
        }

		// Joint Error
        if (ImPlot::BeginPlot("Joint Error Overlay", plotSz)) {
            ImPlot::SetupAxes("t (s)", "e (rad)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_None);
            for (int j = 0; j < jointCount; ++j) {
                char label[16];
                snprintf(label, sizeof(label), "J%02d", j + 1);
                ImPlot::PlotLine(label, x.data() + start, y[j].data() + start, count);
            }
            ImPlot::EndPlot();
        }
        ImGui::Spacing();
    }

    void ControlPanel::drawTrajectoryInspector(const diagnostics::TelemetryRecorder& rec, int jointCount, int& selectedJoint) {
        const auto& ring = rec.ring;
        if (ring.size() < 1) { ImGui::TextUnformatted("No trajectory telemetry yet."); return; }

		const diagnostics::TelemetrySample& s = ring.at(ring.size() - 1);
		if (selectedJoint < 0) { selectedJoint = 0; }
		if (selectedJoint >= (int)s.j.size()) { selectedJoint = (int)s.j.size() - 1; }

		ImGui::Text("t = %.3f s", s.timeSec);

		int currentJ = selectedJoint + 1;
        if (ImGui::SliderInt("Joint index", &currentJ, 1, (int)s.j.size())) {
            selectedJoint = currentJ - 1;
            selectJointAndFollow(selectedJoint);
        }
        else {
            selectedJoint = currentJ - 1; // keep it in sync even if unchanged
        }

		const diagnostics::JointTelemetry& j = s.j[selectedJoint];
		const float e = j.thetaRefRad - j.thetaRad;

        ImGui::Separator();
        ImGui::TextDisabled("Robot loaded:   %s", _requestedRobot.c_str());
        ImGui::TextDisabled("Selected Joint: %s", _currentJointName.c_str());

		// --------------- Joint Inspector ----------------
        ImGui::Separator();
		// Joint Info
        ImGui::Text("State:");
		ImGui::Text("theta:     %.6f rad",       j.thetaRad);
		ImGui::Text("omega:     %.6f rad/s",     j.omegaRad_s);
		ImGui::Text("damping:   %.6f kg·m^2/s",  j.damping);
        ImGui::Text("friction:  %.6f N·m",       j.friction);
        ImGui::Text("torque:    %.6f N·m",       j.torqueNm);
		ImGui::Text("Inertia:   %.6f kg·m^2",    j.I_eff);

        ImGui::Separator();
		// Reference Info
		ImGui::Text("Reference:");
		ImGui::Text("theta_ref: %.6f rad",     j.thetaRefRad);
		ImGui::Text("omega_ref: %.6f rad/s",   j.omegaRefRad_s);
        ImGui::Text("alpha_ref: %.6f rad/s^2", j.alphaRefRad_s2);
		ImGui::Text("error e:   %.6f drad",    e);

		ImGui::Separator();
		// Control Info
		ImGui::Text("Control:");
		ImGui::Text("Active: %s", j.traj_active ? "Yes" : "No");
        if (j.traj_active) {
            ImGui::Text("traj q:    %.6f", j.traj_q);
            ImGui::Text("traj qd:   %.6f", j.traj_qd);
		    ImGui::Text("traj qdd:  %.6f", j.traj_qdd);
		}

		ImGui::Separator();
		// Limit Info
        ImGui::Text("Limits:");
		ImGui::Text("Clamp_theta:   %s", j.clampTheta ? "Yes" : "No");
		ImGui::Text("Clamp_omega:   %s", j.clampOmega ? "Yes" : "No");

		// Find and show worst joint button
        if (ImGui::Button("Show Worst Joint")) {
            float worstErr = 0.0f;
            int worstIdx = 0;
            for (int i = 0; i < (int)s.j.size(); ++i) {
                float err = std::abs(s.j[i].thetaRefRad - s.j[i].thetaRad);
                if (err > worstErr) { worstErr = err; worstIdx = i; }
            }
            selectedJoint = worstIdx;
            selectJointAndFollow(selectedJoint);
        }
	}
}