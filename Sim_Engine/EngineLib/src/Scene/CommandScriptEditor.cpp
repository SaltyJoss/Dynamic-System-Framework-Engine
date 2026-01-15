#include "pch.h"
#include <imgui.h>
#include "Scene/CommandScriptEditor.h"
#include "Interpreter/StoredProgram.h"
#include <io.h>

#include "EngineLib/LogMacros.h"

namespace gui {
	CommandScriptEditor::CommandScriptEditor(gui::simManager* sim) : _sim(sim), _parser(nullptr), _program(nullptr), _wrapper(nullptr) {
		_script = std::vector<std::string>();
		_isRunning = false;

		// Preallocate script text buffer
		_scriptText.reserve(8192);

		_currentScriptPath = "Engine/assets/scripts";
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
		_isRunning = false;
	}

	static int textResizeCallback(ImGuiInputTextCallbackData* data) {
		if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
			auto* str = static_cast<std::string*>(data->UserData);
			str->resize(data->BufTextLen);
			data->Buf = str->data();
		}
		return 0;
	}

	static std::string filenameOnly(const std::string& path) {
		if (path.empty()) return "<...>";
		return std::filesystem::path(path).filename().string();
	}
	
	// Draw the menu items
	void CommandScriptEditor::drawMenus() {
		double dt = ImGui::GetIO().DeltaTime;

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
		}

		// Run/Stop button for the command script

		const bool wasRunning = _isRunning; // snapshot

		if (wasRunning) {
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.80f, 0.15f, 0.15f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.90f, 0.20f, 0.20f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.70f, 0.10f, 0.10f, 1.0f));
		}

		if (ImGui::Button(_isRunning ? "Stop Script" : "Run Script")) {
			_isRunning = !_isRunning;

			if (!_isRunning) {
				// stopping
				if (_program) _program->stop();

				LOG_INFO("Command script stopped.");
				D_INFO("Command script stopped.");

				_isRunning = false;
			}
			else {
				delete _wrapper; _wrapper = nullptr;
				delete _parser;  _parser = nullptr;
				delete _program; _program = nullptr;

				_program = new interpreter::StoredProgram(_sim);
				_program->setDefaultObject(_sim->getObject());
				_parser = new interpreter::Parser(_program);
				_wrapper = new interpreter::RunWrapper(_parser, _program);

				// Remove trailing null character if present
				std::string code = _scriptText;
				if (!code.empty() && code.back() == '\0') code.pop_back();

				_wrapper->runProgram(_scriptText);

				LOG_INFO("Command script started.");
				D_INFO("Command script started.");
			}

			LOG_INFO("Command script %s.", _isRunning ? "started" : "stopped");
			D_INFO("Command script %s.", _isRunning ? "started" : "stopped");
		}

		if (wasRunning) {
			ImGui::PopStyleColor(3);
		}

		if (_isRunning && _program) {
			_program->step(dt);

			if (_program->isStopped()) {
				_isRunning = false;

				delete _wrapper; _wrapper = nullptr;
				delete _parser;  _parser = nullptr;
				delete _program; _program = nullptr;

				LOG_INFO("Command script completed.");
				D_INFO("Command script completed.");
			}
		}
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
		ImGui::TextUnformatted("Script:");
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

		ImGui::InputTextMultiline(
			"##editor",
			_scriptText.data(),
			_scriptText.size() + 1,
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


		// basic status: character count + quick hint
		ImGui::TextDisabled("Lines: %d", lineCount);
		ImGui::SameLine(); ImGui::TextDisabled(" | ");
		ImGui::SameLine(); ImGui::TextDisabled("Chars: %d", (int)_scriptText.size());
		ImGui::SameLine();  ImGui::TextDisabled("|");
		ImGui::SameLine(); ImGui::TextDisabled("State: %s", _isRunning ? "Running" : "Idle");
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

		fwrite(_scriptText.data(), 1, _scriptText.size(), file);
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

		if (ImGui::Button("Choose Folder...")) {
			_save.Open();
		}

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

		if (ImGui::InputText("Filename", filenameBuf, sizeof(filenameBuf))) {
			_pendingSaveName = filenameBuf;
		}

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		if (ImGui::Button("Save")) {
			std::string fullPath = _pendingSavePath;

			if (!fullPath.empty() && fullPath.back() != '/' && fullPath.back() != '\\') {
				fullPath += "/";
			}

			if (trySaveScriptToFile(fullPath)) {
				_currentScriptFile = fullPath;
				_currentScriptPath = _pendingSavePath;
				LOG_INFO("Command script saved to file: %s", _currentScriptFile.c_str());
				D_SUCCESS("Command script saved to file: %s", _currentScriptFile.c_str());

				ImGui::CloseCurrentPopup();
			}
			else {
				LOG_ERROR("Failed to save command script to file: %s", fullPath.c_str());
			}
		}

		ImGui::SameLine();

		if (ImGui::Button("Cancel")) {
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
		if (!ImGui::BeginPopupModal("Save Command Script", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) { ImGui::PopStyleVar(); return; }

	}

	// Helper function to render inline colored text
	static void TextInlineColored(const ImVec4& color, const char* text) {
		ImGui::SameLine(0.0f, 0.0f);
		ImGui::TextColored(color, "%s", text);
	}

	void CommandScriptEditor::renderCmdInstructions() {
		ImGui::BeginChild(
			"CmdInstructions",
			ImVec2(0, -30),
			true,
			ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar
		);
		ImGui::Spacing();

		ImGui::SeparatorText("DSL Command Format:");
		
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();


		// New format: command(identifier, arg1, arg2, ...) # Description
		ImGui::TextColored(CMD_COL, "command");
		TextInlineColored(ARG_COL, "(identifier, arg1, arg2, ...)");
		TextInlineColored(DESC_COL, " \'#\' Denotes a comment");

		ImGui::Spacing();

		ImGui::TextDisabled("Notes:");
		ImGui::BulletText("Commands and identifiers are case-insensitive (parser lowercases).");
		ImGui::BulletText("Strings can be quoted: \"...\" to allow spaces in paths.");
		ImGui::BulletText("Inline comments use '#': rotate(obj, 30, 0, 90) # quarter turn");

		ImGui::Spacing();
		ImGui::SeparatorText("DSL Command List:");
		ImGui::Spacing();

		// --- TRANSLATE ---
		// translate(obj, x, y, z, vel) or translate({x,y,z}, vel, dt) depending on your actual command design
		ImGui::TextColored(CMD_COL, "translate");
		TextInlineColored(ARG_COL, "(obj, ");
		TextInlineColored(ARG_COL, "x, y, z, ");
		TextInlineColored(ARG_COL, "distance, velocity)");
		TextInlineColored(DESC_COL, " # Translate default object");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// --- ROTATE (axis mask) ---
		ImGui::TextColored(CMD_COL, "rotate");
		TextInlineColored(ARG_COL, "({x,y,z}, omega, startDeg, endDeg)");
		TextInlineColored(DESC_COL, " # Rotate about axes (mask)");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// --- ROTATE (default object) ---
		ImGui::TextColored(CMD_COL, "rotate");
		TextInlineColored(ARG_COL, "(obj, omega, startDeg, endDeg)");
		TextInlineColored(DESC_COL, " # Rotate default object (mask defaults e.g. z)");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// --- ROTATE (link/joint) ---
		ImGui::TextColored(CMD_COL, "rotate");
		TextInlineColored(ARG_COL, "(linkName, omega, startDeg, endDeg)");
		TextInlineColored(DESC_COL, " # Rotate robot joint/link");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// --- SET (integrator) ---
		ImGui::TextColored(CMD_COL, "set");
		TextInlineColored(ARG_COL, "(integrator, euler|midpoint|heun|ralston|rk4)");
		TextInlineColored(DESC_COL, " # Set integration method");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// --- COLOUR ---
		ImGui::TextColored(CMD_COL, "colour");
		TextInlineColored(ARG_COL, "(obj, r, g, b)");
		TextInlineColored(DESC_COL, " # Set colour (RGB 0..1 or 0..255 depending on your design)");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// --- LOAD ---
		ImGui::TextColored(CMD_COL, "load");
		TextInlineColored(ARG_COL, "(obj, \"path/to/model.obj\")");
		TextInlineColored(DESC_COL, " # Load object asset");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::TextColored(CMD_COL, "load");
		TextInlineColored(ARG_COL, "(robot, \"RobotName\" | \"path/to/robot.json\")");
		TextInlineColored(DESC_COL, " # Load robot model");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::TextColored(CMD_COL, "load");
		TextInlineColored(ARG_COL, "(tex, \"path/to/texture.png\")");
		TextInlineColored(DESC_COL, " # Load texture");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

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