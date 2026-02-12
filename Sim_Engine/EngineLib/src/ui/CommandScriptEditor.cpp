#include "pch.h"
// File:   CommandScriptEditor.cpp
// GitHub: SaltyJoss
#include <imgui.h>
#include "ui/CommandScriptEditor.h"
#include "Interpreter/StoredProgram.h"
#include "Platform/Paths.h"
#include <io.h>

#include "EngineLib/LogMacros.h"

namespace gui {
	CommandScriptEditor::CommandScriptEditor(gui::SimManager* sim) : _sim(sim), _parser(nullptr), _program(nullptr), _wrapper(nullptr) {
		_script = std::vector<std::string>();
		_sim->setScriptRunning(false);

		// Preallocate script text buffer
		_scriptText.reserve(8192);

		_currentScriptPath = (paths::assets() / "DSLScripts").string();
		_currentScriptFile = "<...>";

		_load.SetTitle("Load Command Script");
		_load.SetDirectory(_currentScriptPath);
		_load.SetTypeFilters({ ".dsl", ".txt" });
		_load.SetCurrentTypeFilterIndex(1); // default to .dsl

		_save.SetTitle("Save Command Script");
		_save.SetDirectory(_currentScriptPath);
		_save.SetTypeFilters({ ".dsl", ".txt" });
		_save.SetCurrentTypeFilterIndex(1); // default to .dsl
	}// .dsl ([Dynamical [S]ystems [L]anguage), a domain-specific language for defining constrained dynamical systems in MGRE

	// Destructor
	CommandScriptEditor::~CommandScriptEditor() {
		_script.clear();
		_sim->setScriptRunning(false);
	}

	static int textResizeCallback(ImGuiInputTextCallbackData* data) {
		if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
			auto* str = static_cast<std::string*>(data->UserData);
			// Resize string callback 
			str->resize(data->BufTextLen + 1);
			data->Buf = str->data();
			// Ensure string size matches buffer length
			str->resize(data->BufTextLen);
		}
		return 0;
	}

	static std::string filenameOnly(const std::string& path) {
		if (path.empty()) return "<...>";
		return std::filesystem::path(path).filename().string();
	}
	
	// Draw the menu items
	void CommandScriptEditor::drawMenus() {
		// File menu for loading/saving scripts
		if (ImGui::BeginMenu("Script")) {
			if (ImGui::MenuItem("Load")) {
				_load.Open();
			}
			if (ImGui::MenuItem("Save")) {
				if (_currentScriptFile == "Engine/assets/scripts") {
					ImGui::OpenPopup("Save Command Script");
				}
				else {
					std::string path = _currentScriptFile;
					trySaveScriptToFile(path);
				}
			}
			if (ImGui::MenuItem("Save As")) {
				_pendingSavePath = _currentScriptPath;
				_pendingSaveName = "unititledScript.dsl";
				_requestSaveAsPopup = true;
			}

			ImGui::EndMenu();
		}

		// AFTER menu closes
		if (_requestSaveAsPopup) {
			ImGui::OpenPopup("Save Command Script");
			_requestSaveAsPopup = false;
		}
		const bool loadedThisFrame = tryLoadFromDialog();
		renderSaveAsPopup();

		if (loadedThisFrame) {
			_script.clear();

			std::string tmp;
			tmp.reserve(_scriptText.size());

			for (char c : _scriptText) {
				if (c == '\0') break;
				if (c == '\n') {
					if (!tmp.empty() && tmp.back() == '\r') { tmp.pop_back(); }
					_script.push_back(tmp);
					tmp.clear();
				}
				else { tmp.push_back(c); }
			}
			if (!tmp.empty()) { _script.push_back(tmp); }

			selectedLines.clear();
			lastClickedLine = -1;

			LOG_INFO("Command script loaded from file: %s", _currentScriptFile.c_str());
			D_INFO("Command script loaded from file: %s", _currentScriptFile.c_str());
		}

		// Run/Stop button for the command script

		const bool wasRunning = _sim->isScriptRunning(); // snapshot

		if (wasRunning) {
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.80f, 0.15f, 0.15f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.90f, 0.20f, 0.20f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.70f, 0.10f, 0.10f, 1.0f));
		}

		if (ImGui::Button(_sim->isScriptRunning() ? "Stop" : "Run")) {
			_sim->setScriptRunning(!_sim->isScriptRunning());
			LOG_INFO("Command script %s.", _sim->isScriptRunning() ? "started" : "stopped");
			D_INFO("Command script %s.", _sim->isScriptRunning() ? "started" : "stopped");

			if (!_sim->isScriptRunning()) {
				if (_program) _program->stop();
				_sim->setActiveProgram(nullptr);
				_sim->stopSimulation();
				_sim->setScriptRunning(false);
			}
			else {
				_sim->setActiveProgram(nullptr);
				delete _wrapper; _wrapper = nullptr;
				delete _parser;  _parser = nullptr;
				delete _program; _program = nullptr;

				_program = new interpreter::StoredProgram(_sim);
				_program->setDefaultObject(_sim->getObject());
				_parser = new interpreter::Parser(_program);
				_wrapper = new interpreter::RunWrapper(_parser, _program);

				_sim->setActiveProgram(_program);
					_sim->setScriptRunning(true);
					_sim->setLastScriptText(_scriptText);

					// Remove trailing null character if present
				std::string code = _scriptText;
				if (!code.empty() && code.back() == '\0') code.pop_back();

				_wrapper->runProgram(_scriptText);
			}
		}

		if (wasRunning) { ImGui::PopStyleColor(3); }

		if (_sim->isScriptRunning()) {
			auto* prog = _sim->activeProgram();
			if (!prog) { terminateScript("Command script stopped -> active program is null.", true); return; }

			if (prog->isEmpty()) { terminateScript("Command script stopped -> program is empty.", true); return; }
			if (prog->isFaulted()) { terminateScript("Command script stopped due to fault.", true); return; }
			if (prog->isCompleted()) { terminateScript("Command script completed.", false); return; }
			if (prog->isStopped() && !prog->isCompleted()) { terminateScript("Command script stopped.", true); return; }
		}
	}

	void CommandScriptEditor::terminateScript(const char* reason, bool fault) {
		_sim->setActiveProgram(nullptr);
		_sim->stopSimulation();
		_sim->setScriptRunning(false);

		if (fault) { LOG_WARN("%s", reason); D_FAIL("%s", reason); }
		else { LOG_INFO("%s", reason); D_SUCCESS("%s", reason); }

		delete _wrapper; _wrapper = nullptr;
		delete _parser;  _parser = nullptr;
		delete _program; _program = nullptr;
	}

	void CommandScriptEditor::render() {
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 310, ImGui::GetIO().DisplaySize.y - 200), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_FirstUseEver);

		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.129f, 0.129f, 0.129f, 0.8f));
		ImGui::Begin("Command Code Editor", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoTitleBar);

		beginEditorPanel("ScriptEditorPanel");

		if (ImGui::BeginTabBar("Debug Tabs")) {
			if (ImGui::BeginTabItem("Code Editor")) {
				renderEnvironment();
				ImGui::EndTabItem();
			}

			if (ImGui::BeginTabItem("Syntax List")) {
				renderCmdInstructions();
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
		}

		endEditorPanel();
		


		ImGui::End();
		ImGui::PopStyleColor();
	}

	void CommandScriptEditor::renderEnvironment() {
		const std::string fileLabel = filenameOnly(_currentScriptFile);

		// Header info
		ImGui::AlignTextToFramePadding();
		ImGui::Text("Script:");
		ImGui::SameLine();
		ImGui::TextDisabled("%s", fileLabel.c_str());

		ImGui::Separator();

		const float statusH = ImGui::GetFrameHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
		ImGui::BeginChild("EditorScroll", ImVec2(0, -statusH), false,
			ImGuiWindowFlags_HorizontalScrollbar);

		ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.1f, 0.1f, 0.1f, 0.925f));
		// Ensure _scriptText has at least 1 char so data() is valid for ImGui
		if (_scriptText.empty()) _scriptText.push_back('\0');


		ImGuiInputTextFlags flags =
			ImGuiInputTextFlags_AllowTabInput |
			ImGuiInputTextFlags_CallbackResize;

		if (_scriptText.empty() || _scriptText.back() != '\0') { _scriptText.push_back('\0'); }

		ImGui::InputTextMultiline(
			"##editor",
			_scriptText.data(),
			_scriptText.capacity() + 1,
			ImVec2(-FLT_MIN, -FLT_MIN),
			flags,
			textResizeCallback,
			&_scriptText
		);

		ImGui::PopStyleColor();
		ImGui::EndChild();

		// Footer status
		ImGui::Separator();

		// Count lines
		int lineCount = 0;
		for (char c : _scriptText) {
			if (c == '\0') break;
			if (c == '\n') ++lineCount;
		}
		// If there's any content, lines = newlines + 1
		if (!_scriptText.empty() && _scriptText[0] != '\0') ++lineCount;


		// Render status info
		ImGui::TextDisabled("Lines: %d", lineCount);
		ImGui::SameLine(); ImGui::TextDisabled(" | ");
		ImGui::SameLine(); ImGui::TextDisabled("Chars: %d", (int)_scriptText.size());
		ImGui::SameLine(); ImGui::TextDisabled(" | ");
		ImGui::SameLine(); ImGui::TextDisabled("State: %s", _sim->isScriptRunning() ? "Running" : "Idle");
	}

	bool CommandScriptEditor::tryLoadFromDialog() {
		_load.Display();
		if (!_load.HasSelected()) { return false; }

		const std::string filePath = _load.GetSelected().string();
		_load.ClearSelected();

		FILE* file = nullptr;
		if (fopen_s(&file, filePath.c_str(), "r") != 0 || !file) {
			LOG_ERROR("Failed to open script file: %s", filePath.c_str());
			return false;
		}

		_scriptText.clear();

		char lineBuf[1024];
		while (fgets(lineBuf, sizeof(lineBuf), file)) {
			_scriptText += lineBuf;
		}
		fclose(file);

		// Ensure null-termination
		if (_scriptText.empty() || _scriptText.back() != '\0') {
			_scriptText.push_back('\0');
		}

		_currentScriptFile = filePath;
		_currentScriptPath = filePath.substr(0, filePath.find_last_of("/\\"));

		_pendingSavePath = _currentScriptPath;

		LOG_INFO("Command script loaded from file: %s", _currentScriptFile.c_str());
		D_SUCCESS("Command script loaded from file: %s", _currentScriptFile.c_str());

		return true;
	}

	bool CommandScriptEditor::trySaveScriptToFile(const std::string& filepath) {
		FILE* file = nullptr;
		if (fopen_s(&file, filepath.c_str(), "w") != 0 || !file) {
			LOG_ERROR("Failed to save to script file for writing: %s", filepath.c_str());
			return false;
		}

		size_t n = _scriptText.size();
		if (n > 0 && _scriptText.back() == '\0') n -= 1;
		fwrite(_scriptText.data(), 1, n, file);
		fclose(file);

		LOG_INFO("Command script saved to file: %s", filepath.c_str());
		D_SUCCESS("Command script saved to file: %s", filepath.c_str());

		return true;
	}

	void CommandScriptEditor::renderSaveAsPopup() {
		// Window Padding
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 12.0f));

		// Save As popup
		if (!ImGui::BeginPopupModal("Save Command Script", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) { ImGui::PopStyleVar(); return; }

		// Display current directory
		ImGui::Text("Directory: ");
		ImGui::SameLine();
		ImGui::TextUnformatted(_pendingSavePath.c_str());

		ImGui::Spacing();

		if (ImGui::Button("Choose Folder...")) { _save.Open(); }

		_save.Display();
		if (_save.HasSelected()) {
			_pendingSavePath = _save.GetSelected().string();
			_save.ClearSelected();
		}

		ImGui::Spacing();

		// Filename input
		static char filenameBuf[256] = {};
		if (_pendingSaveName.empty()) {
			_pendingSaveName = "script.scl";
			std::snprintf(filenameBuf, sizeof(filenameBuf), "%s", _pendingSaveName.c_str());
		}

		if (ImGui::InputText("Filename", filenameBuf, sizeof(filenameBuf))) { _pendingSaveName = filenameBuf; }

		ImGui::SectionDivider();

		if (ImGui::Button("Save")) {
			std::string fullPath = _pendingSavePath;

			if (!fullPath.empty() && fullPath.back() != '/' && fullPath.back() != '\\') { fullPath += "/"; }

			fullPath += _pendingSaveName;

			if (trySaveScriptToFile(fullPath)) {
				_currentScriptFile = fullPath;
				_currentScriptPath = _pendingSavePath;
				LOG_INFO("Command script saved to file: %s", _currentScriptFile.c_str());
				D_SUCCESS("Command script saved to file: %s", _currentScriptFile.c_str());

				ImGui::CloseCurrentPopup();
			}
			else { LOG_ERROR("Failed to save command script to file: %s", fullPath.c_str()); }
		}

		ImGui::SameLine();
		if (ImGui::Button("Cancel")) { ImGui::CloseCurrentPopup(); }

		ImGui::EndPopup();
		ImGui::PopStyleVar();

	}

	// Helper function to render inline colored text
	static void TextInlineColored(const ImVec4& color, const char* text) {
		ImGui::SameLine(0.0f, 0.0f);
		ImGui::TextColored(color, "%s", text);
	}

	// Render the command syntax instructions
	void CommandScriptEditor::renderCmdInstructions() {
		ImGui::BeginChild("CmdInstructions", ImVec2(0, -30), true, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar);
		
		ImGui::SectionHeader("DSL Command Format:");
		ImGui::Spacing();

		ImGui::TextColored(CMD_COL, "command");
		TextInlineColored(ARG_COL, "(identifier, arg1, arg2, ...)");
		TextInlineColored(DESC_COL, " \'#\' Denotes a comment");

		ImGui::Spacing();

		ImGui::TextDisabled("Notes:");
		ImGui::BulletText("Commands and identifiers are case-insensitive (parser lowercases).");
		ImGui::BulletText("Strings can be quoted: \"...\" to allow spaces in paths.");
		ImGui::BulletText("Inline comments use '#': rotate(obj, 30, 0, 90) # quarter turn");

		ImGui::Spacing();
		ImGui::SectionHeader("DSL Command List:");
		ImGui::Spacing();
		
		// --- TRAJSET ---
		ImGui::TextColored(CMD_COL, "trajSet");
		TextInlineColored(ARG_COL, "(<J_i>, <type>, <params...>)");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Sets a trajectory for robotic arm joint (J_i) of some type with parameters");
			ImGui::TextDisabled("Types and parameters:");
			ImGui::TextDisabled("	• <type>: TRAP, SINE, MSINE");
			ImGui::TextDisabled("	• TRAP, <params...>: q1(°), vmax(°/s), amax(°/s²)");
			ImGui::TextDisabled("	• SINE, <params...>:  duration(s), center(°), amp(°), freq(Hz) [, phase(°)]");
			ImGui::TextDisabled("	• MSINE, <params...>: duration(s), center(°) amp1(°), f1(Hz), ph1(°), amp2, f2, ph2, ...");
			ImGui::EndTooltip();
		}

		ImGui::SectionDivider();

		// --- TRAJCLEAR ---
		ImGui::TextColored(CMD_COL, "trajClear");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			ImGui::TextDisabled("No arguments.");
			TextInlineColored(DESC_COL, "# Clears any trajectory set for robotic arm joints");
			ImGui::EndTooltip();
		}
		TextInlineColored(ARG_COL, "()");

		// --- ROTATEJOINTTO ---
		ImGui::TextColored(CMD_COL, "rotateJointTo");
		TextInlineColored(ARG_COL, "(<J_i>, <°w^(-1)>, <theta°>)");

		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Rotates robotic arm joint (J) to some angle (°) by some max angular velocity (°w^(-1))");
			ImGui::EndTooltip();
		}

		ImGui::SectionDivider();

		// --- ROTATEJOINTBY ---
		ImGui::TextColored(CMD_COL, "rotateJointBy");
		TextInlineColored(ARG_COL, "(<J_i>, <°w^(-1)>, <delta°>)");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Rotates robotic arm joint (J_i) by some delta angle (°) by some angular velocity (°w^(-1))");
			ImGui::EndTooltip();
		}

		ImGui::SectionDivider();

		// --- ROTATETO ---
		ImGui::TextColored(CMD_COL, "rotateTo");
		TextInlineColored(ARG_COL, "(<B>, <{x,y,z}>, <°ω⁻¹>, <theta°>)");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Rotates a rigid body (B) to some angle (°) by some max angular velocity (°w^(-1)) across some axis ({x,y,z})");
			ImGui::EndTooltip();
		}

		ImGui::SectionDivider();

		// --- ROTATEBY ---
		ImGui::TextColored(CMD_COL, "rotateBy");
		TextInlineColored(ARG_COL, "(<B>, <{x,y,z}>, <°w^(-1)>, <delta°>)");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Rotates a rigid body (B) by some delta angle (°) at some angular velocity (°w^(-1)) across some axis ({x,y,z})");
			ImGui::EndTooltip();
		}

		ImGui::SectionDivider();

		// --- LOAD ---
		ImGui::TextColored(CMD_COL, "load");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Load some single or chain rigid body model");
			ImGui::EndTooltip();
		}
		TextInlineColored(ARG_COL, "(<id>, <path>)");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			ImGui::TextDisabled("Identifiers & Arguments:");
			ImGui::TextDisabled("	• \"Robot\" -> args: \"Z1\", \"UR5e\", \"Panda\", \"iiwa14\", \"VISPA\", \"H1\", \"some\\path\\to\\robot.json\"");
			ImGui::TextDisabled("	• \"Obj\"	-> args: \"cube\", \"circle\", \"some\\path\\to\\object\\location.fbx\"");
			ImGui::EndTooltip(); 
		}

		ImGui::SectionDivider();

		// --- SET ---
		ImGui::TextColored(CMD_COL, "set");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Set some identifier by a value in its respective format");
			ImGui::EndTooltip();
		}
		TextInlineColored(ARG_COL, "(<id>, <value>)");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			ImGui::TextDisabled("Identifiers & Arguments:");
			ImGui::TextDisabled("	• \"integrator\" -> args: \"euler\", \"midpoint\", \"heun\", \"ralston\", \"rk4\", \"rk45\"");
			ImGui::TextDisabled("	• \"dt\"         -> args: \"numerical val\"");
			ImGui::TextDisabled("	• \"omega\"      -> args: \"numerical val\"");
			ImGui::TextDisabled("	• \"colour\"		-> args: \"{0.0-1.0, 0.0-1.0, 0.0-1.0}\", \"#ffffff\", \"red\"");
			ImGui::EndTooltip();
		}

		ImGui::SectionDivider();

		// --- START ---
		ImGui::TextColored(CMD_COL, "start");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Start the simulation run");
			ImGui::TextDisabled("No arguments.");
			ImGui::TextDisabled("Note: 'start' must be called to begin a simulation run after loading models and setting parameters.");
			ImGui::EndTooltip();
		}
		TextInlineColored(ARG_COL, "()");

		ImGui::SectionDivider();

		// --- STOP ---
		ImGui::TextColored(CMD_COL, "stop");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Stops the simulation");
			ImGui::TextDisabled("No arguments.");
			ImGui::TextDisabled("Note: \'stop\' halts the simulation run but does not reset loaded models or parameters.");
			ImGui::EndTooltip();
		}
		TextInlineColored(ARG_COL, "()");

		ImGui::SectionDivider();

		// --- WAIT ---
		ImGui::TextColored(CMD_COL, "wait");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Pauses script execution for some time (s)");
			ImGui::TextDisabled("Argument: time in seconds (s).");
			ImGui::EndTooltip();
		}
		TextInlineColored(ARG_COL, "(<time_s>)");

		ImGui::SectionDivider();

		// --- SELECT ---
		ImGui::TextColored(CMD_COL, "select");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Selects a rigid body or joint by its identifier");
			ImGui::TextDisabled("Argument: identifier string of the object to select.");
			ImGui::EndTooltip();
		}
		TextInlineColored(ARG_COL, "(<id>)");

		ImGui::Text("DSL Parallel Execution:");

		// --- PARALLEL ---
		ImGui::TextColored(CMD_COL, "parallel");
		if (ImGui::IsItemHovered()) {
			ImGui::BeginTooltip();
			TextInlineColored(DESC_COL, "# Begins a parallel block where commands run concurrently");
			ImGui::TextDisabled("Argument: timeout in seconds (s) for the parallel block to auto-complete.");
			ImGui::EndTooltip();
		}
		TextInlineColored(ARG_COL, "(<timeout>)");

		ImGui::EndChild();
	}

	void CommandScriptEditor::beginEditorPanel(const char* id) {
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 6.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10.0f, 8.0f));
		ImGui::BeginChild(id, ImVec2(0, 0), true, ImGuiWindowFlags_AlwaysUseWindowPadding);
	}

	void CommandScriptEditor::endEditorPanel() {
		ImGui::EndChild();
		ImGui::PopStyleVar(4);
	}
}