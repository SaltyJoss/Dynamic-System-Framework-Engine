#include "pch.h"
// File:    ControlPanel.cpp
// GitHub:  SaltyJoss
#include "Scene/Camera.h"
#include "Scene/Mesh.h"
#include "ui/ControlPanel.h"
#include "Robots/RobotSystem.h"
#include "Platform/Paths.h"
#include <chrono>
#include <imgui.h>
#include <imgui.h>
#include "Platform/imguiWidgets.h"

#include <implot.h>

#ifdef __gl_h_
#undef __gl_h_ 
#endif
#include <glad/glad.h>
#include <stb/stb_image_write.h>

#include "EngineLib/LogMacros.h"

namespace gui {
	// --- Helper Functions ---

	// Get human-readable name for gravity level
	static const char* gravityLevelName(GravityLevel level) {
		switch (level) {
		case GravityLevel::Root:        return "Presets";
		case GravityLevel::SolarSystem: return "Solar System";
		case GravityLevel::Planets:     return "Planets";
		case GravityLevel::Moons:       return "Moons";
		default:                        return "";
		}
	}

	// Gravity value lookup from preset
    static double gravityFromPreset(GravityPreset p) {
        switch (p) {
        case PRESET_ZERO_G:    return constants::g_zero;
        case PRESET_MICRO_G:   return constants::g_micro;

        case PRESET_SUN:       return constants::g_Sun;
        case PRESET_MERCURY:   return constants::g_Mercury;
        case PRESET_VENUS:     return constants::g_Venus;
        case PRESET_EARTH:     return constants::g_Earth;
        case PRESET_MARS:      return constants::g_Mars;
        case PRESET_JUPITER:   return constants::g_Jupiter;
        case PRESET_SATURN:    return constants::g_Saturn;
        case PRESET_URANUS:    return constants::g_Uranus;
        case PRESET_NEPTUNE:   return constants::g_Neptune;
        case PRESET_PLUTO:     return constants::g_Pluto;

        case PRESET_MOON:      return constants::g_Moon;
        case PRESET_TITAN:     return constants::g_Titan;
        case PRESET_ENCELADUS: return constants::g_Enceladus;
        case PRESET_EUROPA:    return constants::g_Europa;
        case PRESET_GANYMEDE:  return constants::g_Ganymede;
        case PRESET_IO:        return constants::g_Io;

        default:               return constants::g_Earth;
        }
    }

	// Draw the gravity preset combo menu
    static void drawGravityChoiceMenu(GravityPreset& preset, double& g) {
        // Combo label shows navigation state
        char label[64];
        snprintf(label, sizeof(label), "Gravity / %s", gravityLevelName(gravityLevel));

        ImGui::Text("Gravity Preset");
        ImGui::SetNextItemWidth(175.0f);

        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 0.8f, 1.0f, 1.0f));

        if (ImGui::BeginCombo("##GravityCombo", label)) {

            // ---------------- ROOT ----------------
            if (gravityLevel == GravityLevel::Root) {

                if (ImGui::Selectable("Zero-G")) {
                    preset = PRESET_ZERO_G;
                    g = gravityFromPreset(preset);
                    ImGui::CloseCurrentPopup();
                }

                if (ImGui::Selectable("Micro-G")) {
                    preset = PRESET_MICRO_G;
                    g = gravityFromPreset(preset);
                    ImGui::CloseCurrentPopup();
                }

                ImGui::Separator();

                if (ImGui::Selectable("Solar System >", false, ImGuiSelectableFlags_DontClosePopups)) {
                    gravityLevel = GravityLevel::SolarSystem;
                }

                if (ImGui::Selectable("Custom")) {
                    preset = PRESET_CUSTOM;
                    gravityMode = GravityUIMode::Custom;
                    ImGui::CloseCurrentPopup();
                }
            }

            // ---------------- SOLAR SYSTEM ----------------
            else if (gravityLevel == GravityLevel::SolarSystem) {

                if (ImGui::Selectable("< Back", false, ImGuiSelectableFlags_DontClosePopups)) {
                    gravityLevel = GravityLevel::Root;
                }

                if (ImGui::Selectable("Planets >", false, ImGuiSelectableFlags_DontClosePopups)) {
                    gravityLevel = GravityLevel::Planets;
                }

                if (ImGui::Selectable("Moons >", false, ImGuiSelectableFlags_DontClosePopups)) {
                    gravityLevel = GravityLevel::Moons;
                }
            }

            // ---------------- PLANETS ----------------
            else if (gravityLevel == GravityLevel::Planets) {

                if (ImGui::Selectable("< Back")) {
                    gravityLevel = GravityLevel::SolarSystem;
                }

                ImGui::Separator();

                struct { const char* name; GravityPreset p; } planets[] = {
                    { "Sun",     PRESET_SUN },
                    { "Mercury", PRESET_MERCURY },
                    { "Venus",   PRESET_VENUS },
                    { "Earth",   PRESET_EARTH },
                    { "Mars",    PRESET_MARS },
                    { "Jupiter", PRESET_JUPITER },
                    { "Saturn",  PRESET_SATURN },
                    { "Uranus",  PRESET_URANUS },
                    { "Neptune", PRESET_NEPTUNE },
                    { "Pluto",   PRESET_PLUTO }
                };

				// List planets
                for (auto& p : planets) {
                    if (ImGui::Selectable(p.name)) {
                        preset = p.p;
                        g = gravityFromPreset(preset);
                    }
                }
            }

            // ---------------- MOONS ----------------
            else if (gravityLevel == GravityLevel::Moons) {
                if (ImGui::Selectable("< Back")) {
                    gravityLevel = GravityLevel::SolarSystem;
                }

                ImGui::Separator();

                struct { const char* name; GravityPreset p; } moons[] = {
                    { "Moon (Earth)", PRESET_MOON },
                    { "Titan",        PRESET_TITAN },
                    { "Enceladus",    PRESET_ENCELADUS },
                    { "Europa",       PRESET_EUROPA },
                    { "Ganymede",     PRESET_GANYMEDE },
                    { "Io",           PRESET_IO }
                };

				// List moons
                for (auto& m : moons) {
                    if (ImGui::Selectable(m.name)) {
                        preset = m.p;
                        g = gravityFromPreset(preset);
                    }
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PopStyleColor();
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

    ControlPanel::ControlPanel(SimManager* sceneView) :
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
        _meshLoad.SetDirectory((paths::assets() / "objects").string());
        _meshLoad.SetTypeFilters({ ".fbx", ".obj", ".dae", ".stl"});
        // HDR loader
        _hdrLoad.SetTitle("Load HDR Environment");
        _hdrLoad.SetDirectory((paths::assets() / "hdr").string());
        _hdrLoad.SetTypeFilters({ ".hdr", ".exr" });

		// One-time ImPlot styling
		static bool plotStyled = false;
		if (!plotStyled) {
			ImPlotStyle& style = ImPlot::GetStyle();
			ImVec4* colors = style.Colors;

			style.LineWeight = 1.1f;
			style.PlotPadding = ImVec2(14, 12);
			style.LabelPadding = ImVec2(6, 4);
			style.LegendPadding = ImVec2(6, 4);
			style.FitPadding = ImVec2(0.05f, 0.05f);

			colors[ImPlotCol_PlotBg] = ImVec4(0.129f, 0.129f, 0.129f, 1.0f);
			colors[ImPlotCol_PlotBorder] = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);
			colors[ImPlotCol_AxisGrid] = ImVec4(0.32f, 0.32f, 0.32f, 0.7f);
			colors[ImPlotCol_AxisText] = ImVec4(0.85f, 0.85f, 0.85f, 1.0f);
			colors[ImPlotCol_AxisTick] = ImVec4(0.75f, 0.75f, 0.75f, 1.0f);
			colors[ImPlotCol_LegendBg] = ImVec4(0.129f, 0.129f, 0.129f, 0.9f);
			colors[ImPlotCol_LegendBorder] = ImVec4(0.4f, 0.4f, 0.4f, 0.6f);

			plotStyled = true;
		}
    }

    void ControlPanel::drawMenus(SimManager* sim) {
        _sim = sim;
        _phys = &_sim->physicsSystem();
        _obj = _sim->getObject();

        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Save Layout")) {
                ImGui::SaveIniSettingsToDisk((paths::configs() / "imgui.ini").string().c_str());
				ImGui::SaveIniSettingsToDisk("imgui.ini");
            }
            if (ImGui::MenuItem("Load Layout")) {
                ImGui::LoadIniSettingsFromDisk((paths::configs() / "imgui.ini").string().c_str());
            }
			ImGui::Separator();
			if (ImGui::MenuItem("Load HDR")) { _hdrLoad.Open(); LOG_INFO("HDR file dialog opened"); }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Reset View")) {
                _sim->resetView();
                LOG_INFO("Scene view reset to default position and orientation.");
            }
            if (ImGui::MenuItem("Reset HDR")) {
                _sim->resetHDRToPreset();
            }
			ImGui::Separator();
			if (ImGui::MenuItem("Properties")) {
				// Placeholder for future properties dialog
			}
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Project")) {
            if (ImGui::MenuItem("Load Obj")) { _meshLoad.Open(); LOG_INFO("File dialog opened"); }
            if (ImGui::MenuItem("Load Robotic Arm")) { _showRobotSelector = true; LOG_INFO("Robotic Arm Menu Opened"); }

            ImGui::Separator();
            const bool canShowResults = (_sim->telemetry().ring.size() >= 2);
            if (ImGui::MenuItem("View Results", nullptr, false, canShowResults)) {
				_showResultsWindow = true;
				_resultsFocusNeeded = true;
            }

            ImGui::EndMenu();
        }
    }

    void ControlPanel::render(SimManager* sceneView) {
        // Initialize pointers to scene scene
        _sim = sceneView;
        fov = _sim->getCamera()->getFOVRadians();
        _mesh = _sim->getMesh();
        _obj = _sim->getObject();
        _light = _sim->getLight();
        _hasRobot = _sim->hasRobot();

		_phys = &_sim->physicsSystem();

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
            if (ImGui::BeginTabItem("Rigid Body Properties")) {
				simulationProperties();
                objectProperties();
                ImGui::EndTabItem();
			}
			if (ImGui::BeginTabItem("Multi-Body Properties")) {
				simulationProperties();
                jointProperties();
                ImGui::EndTabItem();
            }
			if (ImGui::BeginTabItem("Display Settings")) {
				displaySettings();
				// Temporary light controls (for testing)
				// tempLightControls();
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
        }

		endControlPanel(); // End Child Panel

        ImGui::End();
        ImGui::PopStyleColor();

        sceneObjectsTable();

        // Detect simulation completion: was running last frame, stopped this frame
        {
            const bool runningNow = _sim->isSimRunning();
            if (_simWasRunningLastFrame && !runningNow) {
                LOG_INFO("Simulation stopped detected (edge). Ring size: %zu", _sim->telemetry().ring.size());
                if (_sim->telemetry().ring.size() >= 2) {
                    _selectResultsTab = true;
                    _showResultsWindow = true;
                    _resultsFocusNeeded = true;
                    LOG_INFO("Auto-opening Results window.");
                }
            }
			_simWasRunningLastFrame = runningNow;
		}

		// Draw separate results window (if open)
		drawResultsWindow();

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


    void ControlPanel::tempLightControls() {
        if (!_light) return;
        ImGui::SeparatorText("Light Settings:");
        ImGui::Text("Intensity");
        ImGui::SetNextItemWidth(150.0f);
        ImGui::DragFloat("##intensity", &_light->_intensity, 0.1f, 0.0f, 100.0f, "%.1f");
        ImGui::Text("Color");
        ImGui::SetNextItemWidth(150.0f);
        ImGui::ColorEdit3("##Colour", glm::value_ptr(_light->_colour)), ImGui::SameLine();
        ImGui::Separator();
        ImGui::Text("Direction");
        ImGui::SetNextItemWidth(150.0f);
        ImGui::DragFloat3("##direction", &_light->_direction.x, 0.1f, -25.0f, 25.0f, "%.2f");
        ImGui::Separator();
        ImGui::Text("Position");
        ImGui::SetNextItemWidth(150.0f);
        ImGui::DragFloat3("##position", &_light->_position.x, 0.1f, -100.0f, 100.0f, "%.1f");
        ImGui::Separator();
    }

    void ControlPanel::simulationProperties() {
        ImGui::SectionHeader("Simulation Settings");
		ImGui::SectionDivider();

		robots::RobotSystem* robot = _sim->robotSystem();
        auto currentIntEnum = robot->getIntegrationMethod();
		auto currentTauEnum = robot->getTorqueMode();

        static const char* intMethodNames[] = { "Euler", "Midpoint", "Heun", "Ralston", "RK4", "RK45"};
        const char* currentIntMethod = intMethodNames[static_cast<int>(currentIntEnum)];

		static const char* torqueModeNames[] = { "None", "Passive", "Controlled" };
		const char* currentTorqueMode = torqueModeNames[static_cast<int>(currentTauEnum)];

        
		// Disable controls while sim is running to prevent conflicts and ensure stability of the simulations
		ImGui::BeginDisabled(_sim->isSimRunning());

		// Integration method combo box
		ImGui::SectionHeader("Integration Method");
		ImGui::SetNextItemWidth(150.0f);
        if (ImGui::BeginCombo("##", currentIntMethod)) {
            for (int n = 0; n < IM_ARRAYSIZE(intMethodNames); ++n) {
                bool isSelected = (n == static_cast<int>(currentIntEnum));

				// When a new method is selected, update the robot's integration method and log the change
                if (ImGui::Selectable(intMethodNames[n], isSelected)) {
                    auto updatedMethod = static_cast<integration::eIntegrationMethod>(n);
                    robot->setIntegrationMethod(updatedMethod);

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

		// Torque mode combo box
		ImGui::SectionHeader("Torque Mode");
		ImGui::SetNextItemWidth(150.0f);
		if (ImGui::BeginCombo("##tau_mode", currentTorqueMode)) {
			for (int n = 0; n < IM_ARRAYSIZE(torqueModeNames); ++n) {
				bool isSelected = (n == static_cast<int>(currentTauEnum));

				// When a new mode is selected, update the robot's torque mode and log the change
				if (ImGui::Selectable(torqueModeNames[n], isSelected)) {
					auto updatedMode = static_cast<robots::eTorqueMode>(n);
					robot->setTorqueMode(updatedMode);

					switch (updatedMode) {
					case robots::eTorqueMode::NONE:
						D_INFO("Torque mode set to None"); break;
					case robots::eTorqueMode::PASSIVE:
						D_INFO("Torque mode set to Passive"); break;
					case robots::eTorqueMode::CONTROLLED:
						D_INFO("Torque mode set to Controlled"); break;
					default:
						break;
					}
				}
				if (isSelected) { ImGui::SetItemDefaultFocus(); }
			}
			ImGui::EndCombo();
		}
		ImGui::EndDisabled();

		// Delta time controls
		ImGui::SectionHeader("Delta Time (dt) Settings:");
		ImGui::Spacing();

		ImGui::BeginDisabled(_sim->isSimRunning());
		// Simulation dt controls
		ImGui::BeginGroup();
		ImGui::Text("Simulation dt:");

		static int k = 6;
		if (ImGui::DragDtFraction("##simDtDrag", k, false)) {
			int x = 30 * k;
			double dt = 1.0 / (double)x;
			_sim->setFixedDt(dt);
		}
		ImGui::EndGroup();

		// Add spacing between the two groups of controls (40px)
		ImGui::SameLine(0.0f, 40.0f);

		// Telemetry dt controls
		ImGui::BeginGroup();
		ImGui::Text("Telemetry dt:");
		
		static int k_tel = 4;
		if (ImGui::DragDtFraction("##telDtDrag", k_tel, true)) {
			int x = 30 * k_tel;
			_sim->setTelemetryHz(x);
		}
		ImGui::EndGroup();
		ImGui::EndDisabled();

        // Deals with simulation time tracking using chrono
        if (_sim->isSimRunning()) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Simulation Running...");
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Elapsed Time: %.3f", _sim->simTime());

			// make sure to stop sim when commands are finished
            if (!_sim->isSimRunning()) {
                ImGui::Text("Simulation Stopped.");
                ImGui::Text("Elapsed Time: %.3f", _sim->simTime());
            }
        }
		ImGui::Separator();
    }

	// Object properties implementation
    void ControlPanel::objectProperties() {
		// Prevent editing properties while sim is running
        if (_sim->isSimRunning()) { 
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Cannot edit object properties while simulation is running.");
            return;
		}
		// Ensure we have an object to edit
        if (!_obj) {
            ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No object selected.");
            return;
        }

		// Display object name
        ImGui::SectionHeader("Physics Settings:");
        ImGui::Separator();

		// Mass, Damping, Gravity Controls
        double minMass    = 0.25; double maxMass    = 100.0; // mass limits
        float minDamping  =  0.0; float maxDamping  =   1.0; // damping limits
        double minGravity =  0.0; double maxGravity =  10.0; // gravity limits

		// Disable controls while sim is running to prevent conflicts
		ImGui::BeginDisabled(_sim->isSimRunning());

		// Mass Control
		ImGui::Text("Mass:");
		ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("kg##mass", ImGuiDataType_Double, &_obj->state.mass, 0.025f, &minMass, &maxMass);
		ImGui::Spacing();

		// Damping Control
		ImGui::Text("Damping:");
		ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("kg/s##damp", ImGuiDataType_Double, &_obj->state.damping, 0.001f, &minDamping, &maxDamping);
		ImGui::Spacing();

		// Gravity Control
		ImGui::Text("Gravity:");
		ImGui::SetNextItemWidth(150.0f); ImGui::DragScalar("m/s^2##g", ImGuiDataType_Double, &_obj->state.gravity, 0.00005f, &minGravity, &maxGravity);
		ImGui::Spacing();

		ImGui::Separator();

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

		// Reset Object Button
        ImGui::Text("Reset Object:");
        if (ImGui::Button("Reset")) {
			// Should not happen since button is disabled when no object
            if (!_obj) {
                LOG_WARN("No object selected to reset.");
                return;
			}
			// Reset object state to initial conditions
            _obj->reset();
            LOG_INFO("Object reset to initial position and orientation.");
            D_INFO("Reset %s", _obj);
        }
		ImGui::Separator();
    }

	// Joint properties implementation
    void ControlPanel::jointProperties() {
		// Prevent editing properties while sim is running
        if (!_hasRobot) { ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "No robot model loaded."); return; }
        robots::RobotSystem* robot = _sim->robotSystem();

		// Get references to robot's links and joints for easy access
		auto& links = robot->links();
		auto& joints = robot->joints();

		// Ensure we have joints to edit
        if (joints.empty()) {
            ImGui::TextDisabled("Robot has no joints.");
            return;
        }

        float minDamping  = 0.0; float maxDamping  = 1.0;   // damping limits
        float minFriction = 0.0; float maxFriction = 10.0;  // friction limits
        double minGravity = 0.0; double maxGravity = 100.0;  // gravity limits

        static int currentJointIndex = 0;
        currentJointIndex = std::clamp(currentJointIndex, 0, (int)joints.size() - 1);

		auto& j = joints[currentJointIndex];
		int linkIndex = currentJointIndex;
		linkIndex = std::clamp(linkIndex, 0, (int)links.size() - 1);
		auto& L = links[linkIndex];

		float c = (float)j.dynamics.damping;
		float f = (float)j.dynamics.friction;
		double g = (double)robot->getGravity();

        const auto& rec = _sim->telemetry();
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
        
        static GravityPreset gravityPreset = PRESET_EARTH;

		drawGravityChoiceMenu(gravityPreset, g);

        if (gravityMode == GravityUIMode::Custom) {
            ImGui::SetNextItemWidth(150.0f);
            if (ImGui::DragScalar("m/s²##g", ImGuiDataType_Double, &g, 0.005f, &minGravity, &maxGravity)) {
                robot->setGravity(g);
            }
        }
        else {
            robot->setGravity(g);
        }
        ImGui::TextDisabled("Gravity: %.3f", g);

        ImGui::Spacing();
        ImGui::Separator();

        ImGui::EndDisabled();

		ImGui::SectionHeader("Joint Telemetry:");

		drawTrajectoryInspector(rec, (int)robot->joints().size(), _selection.index);
		ImGui::TextDisabled("Selected Joint: %s - Child Link: %s", j.name.c_str(), L.name.c_str());
		ImGui::Spacing();

		ImGui::Spacing();
		ImGui::Text("Reset Robot:");
		ImGui::Spacing();

		if (ImGui::Button("Reset")) {
			if (!_hasRobot) { LOG_WARN("No robot selected to reset."); return; } // should not happen

			_sim->robotSystem()->resetRobot();

			// Clear selection
			_selection.type = SelectionType::NONE;
			_selection.index = -1;
			_selection.source = SelectionSource::NONE;
			_currentJointName = "";
			_currentLinkName = "";

			D_INFO("Reset Robot to initial position and orientation.");
			return;
		}
    }

	// Display settings implementation
    void ControlPanel::displaySettings() {
		ImGui::SectionHeader("Display Settings");
		ImGui::Spacing();

        static float fovDeg = 70.0f;
        ImGui::BeginDisabled(_sim->isSimRunning());

		ImGui::Spacing();
		ImGui::Text("Camera Field of View (FOV):");
		
        ImGui::SetNextItemWidth(150.0f);
        bool edited = ImGui::SliderFloat("Field of View", &fovDeg, 25.0f, 125.0f, "%.f");
        bool active = ImGui::IsItemActive();

        if (!active && !edited) { fovDeg = _sim->getCamera()->getFOVDegrees(); }
        if (edited) { _sim->getCamera()->setFOVDegrees(fovDeg); }
        ImGui::EndDisabled();

        ImGui::Spacing();

		// Graphics Quality Presets
        static int graphicsIndx = 1;
		const char* qualityOptions[] = { "Low", "Medium", "High", "Ultra" };

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, ImGui::GetStyle().ItemSpacing.y));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);

		// Quality Buttons
        bool qualityChanged = ImGui::SegmentedButtonRow("Graphics Settings:", qualityOptions, IM_ARRAYSIZE(qualityOptions), graphicsIndx, 70.0f);

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
		static int resIndx = 1;
		const char* resOptions[] = { "1280x720", "1920x1080", "2560x1440", "3840x2160" };

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, ImGui::GetStyle().ItemSpacing.y));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);

		// Resolution Buttons
        bool resChanged = ImGui::SegmentedButtonRow("Resolution Presets:", resOptions, IM_ARRAYSIZE(resOptions), resIndx, 90.0f);

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
                (int)(_sim->size().x * s.renderScale),
                (int)(_sim->size().y * s.renderScale));

            D_INFO("Render resolution preset changed to %dx%d",
                (int)(_sim->size().x * s.renderScale),
                (int)(_sim->size().y * s.renderScale));
		}

        ImGui::Spacing();

		// Shader Mode Selector
		static int shaderIndx = 2;
		const char* shaderOptions[] = { "Basic Shader", "Lit Shader", "PBR Shader" };

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, ImGui::GetStyle().ItemSpacing.y));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);

		// Shader Buttons
        bool shaderChanged = ImGui::SegmentedButtonRow("Shader Mode:", shaderOptions, IM_ARRAYSIZE(shaderOptions), shaderIndx, 110.0f);

		ImGui::PopStyleVar(2);

		// Apply shader changes if needed
        if (shaderChanged) {
            switch (shaderIndx) {
                case 0: _sim->currentShaderMode = SimManager::ShaderMode::Basic; D_INFO("Shader -> Basic Shader"); break;
                case 1: _sim->currentShaderMode = SimManager::ShaderMode::Lit;   D_INFO("Shader -> Lit Shader");   break;
                case 2: _sim->currentShaderMode = SimManager::ShaderMode::PBR;   D_INFO("Shader -> PBR Shader");   break;
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

        ImGui::SectionHeader("Select a robotic arm model:");
        ImGui::Separator();
        ImGui::Spacing();

        roboticCardDisplay("Z1", "Unitree Robotics");
        roboticCardDisplay("UR5e", "Universal Robots");
        roboticCardDisplay("Panda", "Franka Robotics");
        roboticCardDisplay("iiwa14", "KUKA");
        roboticCardDisplay("VISPA", "Airbus");
        roboticCardDisplay("H1", "Unitree Robotics");

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

		ImGui::SectionHeader("Active Rigid-Bodies");
		ImGui::SectionDivider();

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
				robots::RobotSystem* robotSys = _sim->robotSystem();
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

	// Select joint from table and set camera to follow it
    void ControlPanel::selectJointAndFollow(int jointIdx) {
        if (!_sim || !_sim->hasRobot()) return;

        robots::RobotSystem* robot = _sim->robotSystem();
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
	// --- Telemetry Plots ---

	// Trajectory Inspector
    void ControlPanel::drawTrajectoryInspector(const diagnostics::TelemetryRecorder& rec, int /*jointCount*/, int& selectedJoint) {
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
            selectedJoint = currentJ - 1;
        }

		const diagnostics::JointTelemetry& j = s.j[selectedJoint];
		const float e = (float)(j.thetaRefRad - j.thetaRad);

        ImGui::Separator();
        ImGui::TextDisabled("Robot loaded:   %s", _requestedRobot.c_str());
        ImGui::TextDisabled("Selected Joint: %s", _currentJointName.c_str());

		// --------------- Joint Inspector ----------------
		ImGui::Spacing();
		// Joint Info
        ImGui::SectionHeader("State:");
		ImGui::Text("theta:     %.6f rad",       j.thetaRad);
		ImGui::Text("omega:     %.6f rad/s",     j.omegaRad_s);
		ImGui::Text("damping:   %.6f kg·m^2/s",  j.damping);
        ImGui::Text("friction:  %.6f N·m",       j.friction);
        ImGui::Text("torque:    %.6f N·m",       j.torqueNm);

		ImGui::Spacing();
		// Reference Info
		ImGui::SectionHeader("Reference:");
		ImGui::Text("theta_ref: %.6f rad",     j.thetaRefRad);
		ImGui::Text("omega_ref: %.6f rad/s",   j.omegaRefRad_s);
        ImGui::Text("alpha_ref: %.6f rad/s^2", j.alphaRefRad_s2);
		ImGui::Text("error e:   %.6f drad",    e);

		ImGui::Spacing();
		// Control Info
		ImGui::SectionHeader("Control:");
		ImGui::Text("Active: %s", j.traj_active ? "Yes" : "No");
        if (j.traj_active) {
            ImGui::Text("traj q:    %.6f", j.traj_q);
            ImGui::Text("traj qd:   %.6f", j.traj_qd);
		    ImGui::Text("traj qdd:  %.6f", j.traj_qdd);
		}

		ImGui::SectionDivider();
		// Limit Info
		ImGui::SectionHeader("Limits:");
		ImGui::Text("Clamp_theta:   %s", j.clampTheta ? "Yes" : "No");
		ImGui::Text("Clamp_omega:   %s", j.clampOmega ? "Yes" : "No");

		// Find and show worst joint button
        if (ImGui::Button("Show Worst Joint")) {
            float worstErr = 0.0f;
            int worstIdx = 0;
            for (int i = 0; i < (int)s.j.size(); ++i) {
                float err = (float)std::abs(s.j[i].thetaRefRad - s.j[i].thetaRad);
                if (err > worstErr) { worstErr = err; worstIdx = i; }
            }
			selectedJoint = worstIdx;
			selectJointAndFollow(selectedJoint);
		}
	}

	// --- CSV Export ---

	// Export telemetry data to CSV for external analysis (e.g. Python, Excel)
	void ControlPanel::exportTelemetryCSV(const char* filepath) {
		const auto& rec = _sim->telemetry();
		const auto& ring = rec.ring;
		if (ring.size() < 2) return;

		FILE* f = nullptr;
		if (fopen_s(&f, filepath, "w") != 0 || !f) {
			D_FAIL("Failed to export CSV to: %s", filepath);
			LOG_ERROR("Failed to export CSV to: %s", filepath);
			return;
		}

		const int sampleCount = (int)ring.size();
		const int jointCount = (int)ring.at(sampleCount - 1).j.size();

		// Header
		fprintf(f, "time_s,err_rms,err_max,clamp_sum");
		for (int j = 0; j < jointCount; ++j) {
			fprintf(f, ",J%02d_theta,J%02d_omega,J%02d_torque,J%02d_theta_ref,J%02d_err", j+1, j+1, j+1, j+1, j+1);
		}
		fprintf(f, "\n");

		// Data rows
		for (int k = 0; k < sampleCount; ++k) {
			const auto& s = ring.at(k);
			fprintf(f, "%.6f,%.9f,%.9f,%d", s.timeSec, s.err_rms, s.err_max, s.clamp_sum);
			const int m = std::min(jointCount, (int)s.j.size());
			for (int j = 0; j < m; ++j) {
				const auto& jt = s.j[j];
				fprintf(f, ",%.9f,%.9f,%.9f,%.9f,%.9f",
					jt.thetaRad, jt.omegaRad_s, jt.torqueNm, jt.thetaRefRad,
					jt.thetaRefRad - jt.thetaRad);
			}
			fprintf(f, "\n");
		}

		fclose(f);
		D_SUCCESS("Telemetry CSV exported to: %s", filepath);
		LOG_INFO("Telemetry CSV exported to: %s", filepath);
	}

	// --- Integrator Comparison ---

	// Run the loaded script with all integrators and capture telemetry for comparison plots
	void ControlPanel::runComparisonAllIntegrators() {
		const std::string& scriptText = _sim->lastScriptText();
		if (scriptText.empty()) {
			D_FAIL("No script loaded. Run a script first, then compare.");
			return;
		}
		if (!_sim->hasRobot()) {
			D_FAIL("No robot loaded. Cannot run comparison.");
			return;
		}

		static const integration::eIntegrationMethod methods[] = {
			integration::eIntegrationMethod::Euler,
			integration::eIntegrationMethod::Midpoint,
			integration::eIntegrationMethod::Heun,
			integration::eIntegrationMethod::Ralston,
			integration::eIntegrationMethod::RK4,
			integration::eIntegrationMethod::RK45
		};
		static const char* names[] = { "Euler", "Midpoint", "Heun", "Ralston", "RK4", "RK45" };

		_comparisonResults.clear();
		_comparisonReady = false;

		D_INFO("Starting integrator comparison (6 methods)...");

		for (int i = 0; i < 6; ++i) {
			D_INFO("  Running: %s ...", names[i]);

			if (!_sim->runScriptToCompletion(scriptText, methods[i])) {
				D_FAIL("  %s failed — skipping.", names[i]);
				continue;
			}

			// Snapshot telemetry into comparison result
			const auto& ring = _sim->telemetry().ring;
			const int sampleCount = (int)ring.size();
			if (sampleCount < 2) continue;

			ComparisonSnapshot snap;
			snap.integratorName = names[i];
			snap.jointCount = (int)ring.at(sampleCount - 1).j.size();

			snap.time.resize(sampleCount);
			snap.errRms.resize(sampleCount);
			snap.errMax.resize(sampleCount);
			snap.jointErr.resize(snap.jointCount);
			for (int j = 0; j < snap.jointCount; ++j) snap.jointErr[j].resize(sampleCount);

			for (int k = 0; k < sampleCount; ++k) {
				const auto& s = ring.at(k);
				snap.time[k] = (float)s.timeSec;
				snap.errRms[k] = (float)s.err_rms;
				snap.errMax[k] = (float)s.err_max;
				const int m = std::min(snap.jointCount, (int)s.j.size());
				for (int j = 0; j < m; ++j) {
					snap.jointErr[j][k] = (float)(s.j[j].thetaRefRad - s.j[j].thetaRad);
				}
			}

			_comparisonResults.push_back(std::move(snap));
		}

		_comparisonReady = !_comparisonResults.empty();
		D_SUCCESS("Integrator comparison complete: %d/%d methods captured.", (int)_comparisonResults.size(), 6);
	}

	// Draw comparison plots (overlays of all integrators)
	void ControlPanel::drawComparisonPlots() {
		if (!_comparisonReady || _comparisonResults.empty()) {
			ImGui::TextDisabled("No comparison data. Click 'Compare All Integrators' first.");
			return;
		}

		ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Integrator Comparison (%d methods)", (int)_comparisonResults.size());
		ImGui::Separator();

		float totalAvail = ImGui::GetWindowHeight() - 200.0f;
		float splitterThickness = 4.0f;

		float reserved = 2.0f * splitterThickness;
		float usable = totalAvail - reserved;
		float minH = 150.0f;

		_cPlotH1 = ImClamp(_cPlotH1, minH, usable - minH);
		_cPlotH2 = ImClamp(_cPlotH2, minH, usable - _cPlotH1 - minH);

		float _cPlotH3 = usable - _cPlotH1 - _cPlotH2;

		float _plotWidth = -1;

		// --- RMS Error Overlay ---
		const ImVec2 cPlotSz1(_plotWidth, _cPlotH1);
		if (ImPlot::BeginPlot("RMS Error - All Integrators##cmp", cPlotSz1)) {
			ImPlot::SetupAxes("t (s)", "RMS error (rad)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
			ImPlot::SetupLegend(ImPlotLocation_NorthEast);
			for (const auto& res : _comparisonResults) {
				ImPlot::PlotLine(res.integratorName.c_str(), res.time.data(), res.errRms.data(), (int)res.time.size());
			}
			ImPlot::EndPlot();
		}

		// --- Splitter A ---
		ImGui::hSplitter("##cSplit1", &_plotH2, minH, usable - _cPlotH2 - minH, splitterThickness);

		// --- Max Error Overlay ---
		const ImVec2 cPlotSz2(_plotWidth, _cPlotH2);
		if (ImPlot::BeginPlot("Max Error - All Integrators##cmp", cPlotSz2)) {
			ImPlot::SetupAxes("t (s)", "Max error (rad)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
			ImPlot::SetupLegend(ImPlotLocation_NorthEast);
			for (const auto& res : _comparisonResults) {
				ImPlot::PlotLine(res.integratorName.c_str(), res.time.data(), res.errMax.data(), (int)res.time.size());
			}
			ImPlot::EndPlot();
		}

		// --- Splitter B ---
		ImGui::hSplitter("##cSplit2", &_cPlotH2, minH, usable - _cPlotH1 - minH, splitterThickness);

		// --- Per-joint error for worst joint (joint with max final error) ---
		const ImVec2 cPlotSz3(_plotWidth, _cPlotH3);
		if (_comparisonResults.front().jointCount > 0) {
			// Find which joint has the most variation across integrators
			static int selectedCmpJoint = 0;
			ImGui::SetNextItemWidth(150.0f);
			ImGui::SliderInt("Compare Joint##cmp", &selectedCmpJoint, 0, _comparisonResults.front().jointCount - 1, "J%02d");

			char plotLabel[64];
			snprintf(plotLabel, sizeof(plotLabel), "J%02d Error - All Integrators##cmpjoint", selectedCmpJoint + 1);
			if (ImPlot::BeginPlot(plotLabel, cPlotSz3)) {
				ImPlot::SetupAxes("t (s)", "error (rad)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
				ImPlot::SetupLegend(ImPlotLocation_NorthEast);
				for (const auto& res : _comparisonResults) {
					if (selectedCmpJoint < (int)res.jointErr.size()) {
						ImPlot::PlotLine(res.integratorName.c_str(), res.time.data(), res.jointErr[selectedCmpJoint].data(), (int)res.time.size());
					}
				}
				ImPlot::EndPlot();
			}
		}

		// --- Export comparison CSV ---
		if (ImGui::Button("Export Comparison CSV")) {
			auto now = std::chrono::system_clock::now();
			auto epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

			std::string filename = _requestedRobot.empty() ? "comparison" : _requestedRobot;
			filename += "_comparison_" + std::to_string(epoch) + ".csv";

			auto outPath = paths::runs() / filename;
			std::filesystem::create_directories(paths::runs());

			FILE* f = nullptr;
			if (fopen_s(&f, outPath.string().c_str(), "w") == 0 && f) {
				// Header: time, then rms/max for each integrator
				fprintf(f, "time_s");
				for (const auto& res : _comparisonResults) {
					fprintf(f, ",%s_rms,%s_max", res.integratorName.c_str(), res.integratorName.c_str());
				}
				fprintf(f, "\n");

				// Use longest time series
				int maxSamples = 0;
				for (const auto& res : _comparisonResults) { maxSamples = std::max(maxSamples, (int)res.time.size()); }

				for (int k = 0; k < maxSamples; ++k) {
					// Use first result's time as reference
					float t = (k < (int)_comparisonResults[0].time.size()) ? _comparisonResults[0].time[k] : 0.0f;
					fprintf(f, "%.6f", t);
					for (const auto& res : _comparisonResults) {
						if (k < (int)res.time.size()) {
							fprintf(f, ",%.9f,%.9f", res.errRms[k], res.errMax[k]);
						} else {
							fprintf(f, ",,");
						}
					}
					fprintf(f, "\n");
				}
				fclose(f);
				D_SUCCESS("Comparison CSV exported to: %s", outPath.string().c_str());
			}
		}
		ImGui::SameLine();
		ImGui::TextDisabled("Saves to: LocalAppData/DSFE/runs/");
	}

	// --- Results Window (post-simulation) ---

	// When a simulation completes, this modal window pops up with the telemetry plots and export options
	void ControlPanel::drawResultsWindow() {
		if (!_showResultsWindow) return;

		// Get the latest telemetry record
		const auto& rec = _sim->telemetry();
		const auto& ring = rec.ring;

		// If telemetry is too short, don't show the window
		if (ring.size() < 2) {
			_showResultsWindow = false;
			return;
		}

		// Force focus on first frame the window opens
		if (_resultsFocusNeeded) { ImGui::SetNextWindowFocus(); }
		// Get display size for dynamic window sizing
		ImVec2 display = ImGui::GetIO().DisplaySize;

		// Use 75% of screen size, clamped to reasonable limits
		ImVec2 desiredSize(display.x * 0.75f, display.y * 0.75f);

		// Safety clamp for very large monitors (>4K or ultrawides)
		desiredSize.x = ImClamp(desiredSize.x, 700.0f, 1200.0f);
		desiredSize.y = ImClamp(desiredSize.y, 500.0f, 900.0f);

		// Set the window size on first appearance
		ImGui::SetNextWindowSize(desiredSize, ImGuiCond_Appearing);

		// Center the window on screen
		ImVec2 center((display.x - desiredSize.x) * 0.5f, (display.y - desiredSize.y) * 0.5f);
		ImGui::SetNextWindowPos(center, ImGuiCond_FirstUseEver);

		// Clear focus flag after using it
		if (_resultsFocusNeeded) { _resultsFocusNeeded = false; }

		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.95f));

		bool open = true;
		
		// Rounded corners + accent title bar
		ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16, 12));
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, 4));
		ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.08f, 0.08f, 0.12f, 1.0f));
		ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.15f, 0.18f, 0.28f, 1.0f));
		
		ImGui::Begin("Simulation Results", &open, ImGuiWindowFlags_NoDocking);
		if (!open) {
			_showResultsWindow = false;
			ImGui::End();
			ImGui::PopStyleColor(2);
			ImGui::PopStyleVar(3);
			ImGui::PopStyleColor();
			return;
		}

		// --- Header ---
		const bool runningNow = _sim->isSimRunning();
		if (runningNow) {
			ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.4f, 1.0f), "Running...");
		}
		else if (_comparisonReady && !runningNow) {
			ImGui::TextColored(ImVec4(1.0f, 0.749f, 0.0f, 1.0f), "Comparison Ready");
		}
		else {
			ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "Comparison Complete");
		}

		// Show robot name if available
		if (!_requestedRobot.empty()) {
			ImGui::SameLine();
			ImGui::TextDisabled("  Robot: %s", _requestedRobot.c_str());
		}

		const auto& last = ring.at(ring.size() - 1);

		ImGui::TextDisabled("Total Time: %.3f s  |  Samples: %d", last.timeSec, (int)ring.size());
		ImGui::Separator();
		ImGui::Spacing();

		// Track the window rect for glReadPixels
		static ImVec2 capturePos = { 0, 0 };
		static ImVec2 captureSize = { 0, 0 };

		if (ImGui::Button("Export as CSV")) {
			auto now = std::chrono::system_clock::now();
			auto epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

			std::string filename = _requestedRobot.empty() ? "results" : _requestedRobot;
			filename += "_results_" + std::to_string(epoch) + ".csv";

			auto outPath = paths::runs() / filename;
			std::filesystem::create_directories(paths::runs());

			exportTelemetryCSV(outPath.string().c_str());
		}

		ImGui::SameLine();
		ImGui::TextDisabled("Saves to: LocalAppData/DSFE/runs/");

		ImGui::Spacing();
		ImGui::Separator();

		// --- Rebuild series from ring (reuse the same helpers) ---
		static std::vector<float> rX, rRms, rMax, rCs;
		static std::vector<std::vector<float>> rY;

		const int sampleCount = (int)ring.size();
		const int jointCount = (int)last.j.size();

		rX.resize(sampleCount);
		rRms.resize(sampleCount);
		rMax.resize(sampleCount);
		rCs.resize(sampleCount);

		if ((int)rY.size() != jointCount) rY.resize(jointCount);
		for (int j = 0; j < jointCount; ++j) rY[j].resize(sampleCount);

		for (int k = 0; k < sampleCount; ++k) {
			const auto& s = ring.at(k);
			rX[k]   = (float)s.timeSec;
			rRms[k] = (float) s.err_rms;
			rMax[k] = (float)s.err_max;
			rCs[k]  = (float)s.clamp_sum;

			const int m = std::min(jointCount, (int)s.j.size());
			for (int j = 0; j < m; ++j) {
				rY[j][k] = (float)(s.j[j].thetaRefRad - s.j[j].thetaRad);
			}
			for (int j = m; j < jointCount; ++j) {
				rY[j][k] = 0.0f;
			}
		}

		float totalAvail = ImGui::GetWindowHeight() - 200.0f;
		float splitterThickness = 4.0f;

		float reserved = 2.0f * splitterThickness;
		float usable = totalAvail - reserved;
		float minH = 150.0f;

		_plotH1 = ImClamp(_plotH1, minH, usable - minH);
		_plotH2 = ImClamp(_plotH2, minH, usable - _plotH1 - minH);

		float _plotH3 = usable - _plotH1 - _plotH2;

		float _plotWidth = -1;

		// --- Error Plot ---
		const ImVec2 plotSz1(_plotWidth, _plotH1);
		if (ImPlot::BeginPlot("Error (RMS, Max)##results", plotSz1)) {
			ImPlot::SetupAxes("t (s)", "error (rad)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
			ImPlot::SetupLegend(ImPlotLocation_NorthEast);
			ImPlot::PlotLine("RMS", rX.data(), rRms.data(), sampleCount);
			ImPlot::PlotLine("Max", rX.data(), rMax.data(), sampleCount);
			ImPlot::EndPlot();
		}

		// --- Splitter A ---
		ImGui::hSplitter("##split1", &_plotH1, minH, usable - _plotH2 - minH, splitterThickness);


		// --- Clamp Events ---
		const ImVec2 tPlotSz2(_plotWidth, _plotH2);
		if (ImPlot::BeginPlot("Clamp Events##results", tPlotSz2)) {
			ImPlot::SetupAxes("t (s)", "Clamp Sum", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
			ImPlot::SetupLegend(ImPlotLocation_NorthEast);
			ImPlot::PlotStairs("Sum", rX.data(), rCs.data(), sampleCount);
			ImPlot::EndPlot();
		}

		// --- Splitter B ---
		ImGui::hSplitter("##split2", &_plotH2, minH, usable - _plotH1 - minH, splitterThickness);

		// --- Joint Error Overlay ---
		const ImVec2 plotSz3(_plotWidth, _plotH3);
		if (ImPlot::BeginPlot("Joint Error Overlay##results", plotSz3)) {
			ImPlot::SetupAxes("t (s)", "e (rad)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
			ImPlot::SetupLegend(ImPlotLocation_NorthEast);
			for (int j = 0; j < jointCount; ++j) {
				char label[16];
				snprintf(label, sizeof(label), "J%02d", j + 1);
				ImPlot::PlotLine(label, rX.data(), rY[j].data(), sampleCount);
			}
			ImPlot::EndPlot();
		}

		// --- Integrator Comparison Section ---
		ImGui::SectionDivider();
		ImGui::SectionHeader("Integrator Comparison:", ImVec4(0.4f, 0.8f, 1.0f, 1.0f));

		const bool hasScript = !_sim->lastScriptText().empty();
		ImGui::BeginDisabled(!hasScript || _sim->isSimRunning());
		if (ImGui::Button("Compare All Integrators")) {
			runComparisonAllIntegrators();
		}
		ImGui::EndDisabled();
		ImGui::SameLine();
		ImGui::TextDisabled("Runs Euler/Midpoint/Heun/Ralston/RK4/RK45 sequentially");
		if (!hasScript) {
			ImGui::SameLine();
			ImGui::TextDisabled("(run a script first)");
		}

		if (_comparisonReady) {
			ImGui::Spacing();
			drawComparisonPlots();
		}

		// Capture window rect for next-frame export
		capturePos = ImGui::GetWindowPos();
		captureSize = ImGui::GetWindowSize();

		ImGui::PopStyleColor(2);
		ImGui::PopStyleVar(3);

		ImGui::End();
		ImGui::PopStyleColor();
	}
}