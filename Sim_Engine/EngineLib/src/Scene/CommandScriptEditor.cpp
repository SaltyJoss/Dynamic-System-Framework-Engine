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
		_load.SetTypeFilters({ ".txt", ".scl" });

		_save.SetTitle("Save Command Script");
		_save.SetDirectory(_currentScriptPath);
		_save.SetTypeFilters({ ".txt", ".scl" });
	}// .dsl ([Dynamical [S]ystems [L]anguage), a domain-specific language for defining constrained dynamical systems in MGRE

	CommandScriptEditor::~CommandScriptEditor() {
		_script.clear();
		_isRunning = false;
	}

	void CommandScriptEditor::drawMenus() {
		double dt = ImGui::GetIO().DeltaTime;

		if (ImGui::BeginMenu("Script")) {
			if (ImGui::MenuItem("Load")) {
				_load.Open();
				tryLoadFromDialog();
			}
			if (ImGui::MenuItem("Save")) {
				if (_currentScriptFile == "<...>") {
					ImGui::OpenPopup("Save Command Script");
				}
				else {
					std::string path = _currentScriptFile;
					trySaveScriptToFile(path);
				}
			}
			if (ImGui::MenuItem("Save As")) {
				ImGui::OpenPopup("Save Command Script");
			}
			ImGui::EndMenu();
		}

		saveAsPopup();

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

		ImGui::End();
		ImGui::PopStyleColor();
	}

	// Callback to handle dynamic resizing of the text buffer
	static int TextResizeCallback(ImGuiInputTextCallbackData* data) {
		if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
			auto* str = static_cast<std::string*>(data->UserData);
			str->resize(data->BufTextLen);
			data->Buf = str->data();
		}
		return 0;
	}

	void CommandScriptEditor::renderEnvironment() {
		ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar
			| ImGuiWindowFlags_AlwaysVerticalScrollbar;

		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.925f));
		ImGui::BeginChild("ScriptEditor", ImVec2(0, -30), true, window_flags);

		ImGui::Text("Enter command script below:");

		ImGui::Separator();

		ImGui::InputTextMultiline(
			"##editor",
			_scriptText.data(),
			_scriptText.capacity() + 1,
			ImVec2(-FLT_MIN, -FLT_MIN),
			ImGuiInputTextFlags_CallbackResize,
			TextResizeCallback,
			&_scriptText
		);

		// Display script lines with selection
		for (int i = 0; i < (int)_script.size(); ++i) {
			const auto& line = _script[i];
			bool selected = selectedLines.count(i) > 0;
			// Handle line selection
			if (ImGui::Selectable(line.c_str(), selected, ImGuiSelectableFlags_AllowDoubleClick)) {
				if (ImGui::GetIO().KeyShift && lastClickedLine != -1) {
					// Range selection
					int start = std::min(lastClickedLine, i);
					int end = std::max(lastClickedLine, i);
					for (int j = start; j <= end; ++j) {
						selectedLines.insert(j);
					}
				}
				else if (ImGui::GetIO().KeyCtrl) {
					// Toggle selection
					if (selected) {
						selectedLines.erase(i);
					}
					else {
						selectedLines.insert(i);
					}
					lastClickedLine = i;
				}
				else {
					// Single selection
					selectedLines.clear();
					selectedLines.insert(i);
					lastClickedLine = i;
				}
			}
		}

		// Copy selected lines to clipboard
		if (ImGui::IsWindowFocused() && ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C)) {
			std::string clip;
			for (int idx : selectedLines) {
				if (idx >= 0 && idx < (int)_script.size()) {
					clip += _script[idx] + "\n";
				}
			}
			ImGui::SetClipboardText(clip.c_str());
		}

		ImGui::EndChild();
		ImGui::PopStyleColor();
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

		ImGui::Separator();
		ImGui::NewLine();

		ImGui::Text("Format:");
		ImGui::NewLine();
		ImGui::Separator();

		// New format: command(identifier, arg1, arg2, ...) # Description
		ImGui::TextColored(CMD_COL, "command");
		TextInlineColored(ARG_COL, "(identifier, arg1, arg2, ...)");
		TextInlineColored(DESC_COL, " \'#\' Denotes the Description");

		ImGui::Separator();
		ImGui::NewLine();

		ImGui::Text("Examples:");
		ImGui::NewLine();
		ImGui::Separator();

		// --- TRANSLATE ---
		// translate(obj, x, y, z, vel) or translate({x,y,z}, vel, dt) depending on your actual command design
		ImGui::TextColored(CMD_COL, "translate");
		TextInlineColored(ARG_COL, "(obj, ");
		TextInlineColored(ARG_COL, "x, y, z, ");
		TextInlineColored(ARG_COL, "distance, velocity)");
		TextInlineColored(DESC_COL, " # Translate default object");

		ImGui::Separator();

		// --- ROTATE (axis mask) ---
		ImGui::TextColored(CMD_COL, "rotate");
		TextInlineColored(ARG_COL, "({x,y,z}, omega, startDeg, endDeg)");
		TextInlineColored(DESC_COL, " # Rotate about axes (mask)");

		ImGui::Separator();

		// --- ROTATE (default object) ---
		ImGui::TextColored(CMD_COL, "rotate");
		TextInlineColored(ARG_COL, "(obj, omega, startDeg, endDeg)");
		TextInlineColored(DESC_COL, " # Rotate default object (mask defaults e.g. z)");

		ImGui::Separator();

		// --- ROTATE (link/joint) ---
		ImGui::TextColored(CMD_COL, "rotate");
		TextInlineColored(ARG_COL, "(linkName, omega, startDeg, endDeg)");
		TextInlineColored(DESC_COL, " # Rotate robot joint/link");

		ImGui::Separator();

		// --- SET (integrator) ---
		ImGui::TextColored(CMD_COL, "set");
		TextInlineColored(ARG_COL, "(integrator, euler|midpoint|heun|ralston|rk4)");
		TextInlineColored(DESC_COL, " # Set integration method");

		ImGui::Separator();

		// --- COLOUR ---
		ImGui::TextColored(CMD_COL, "colour");
		TextInlineColored(ARG_COL, "(obj, r, g, b)");
		TextInlineColored(DESC_COL, " # Set colour (RGB 0..1 or 0..255 depending on your design)");

		ImGui::Separator();

		// --- LOAD ---
		ImGui::TextColored(CMD_COL, "load");
		TextInlineColored(ARG_COL, "(obj, \"path/to/model.obj\")");
		TextInlineColored(DESC_COL, " # Load object asset");

		ImGui::Separator();

		ImGui::TextColored(CMD_COL, "load");
		TextInlineColored(ARG_COL, "(robot, \"RobotName\" | \"path/to/robot.json\")");
		TextInlineColored(DESC_COL, " # Load robot model");

		ImGui::Separator();

		ImGui::TextColored(CMD_COL, "load");
		TextInlineColored(ARG_COL, "(tex, \"path/to/texture.png\")");
		TextInlineColored(DESC_COL, " # Load texture");

		ImGui::Separator();

		ImGui::NewLine();
		ImGui::TextDisabled("Notes:");
		ImGui::BulletText("Commands and identifiers are case-insensitive (parser lowercases).");
		ImGui::BulletText("Strings can be quoted: \"...\" to allow spaces in paths.");
		ImGui::BulletText("Inline comments use '#': rotate(obj, 30, 0, 90) # quarter turn");

		ImGui::EndChild();
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
		char line[1024];
		while (fgets(line, sizeof(line), file)) {
			_scriptText += line; // keep original newlines
		}

		fclose(file);

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
			LOG_ERROR("Failed to open script file for writing: %s", filepath.c_str());
			return false;
		}

		fwrite(_scriptText.data(), 1, _scriptText.size(), file);
		fclose(file);

		LOG_INFO("Command script saved to file: %s", filepath.c_str());
		D_SUCCESS("Command script saved to file: %s", filepath.c_str());

		return true;
	}

	bool CommandScriptEditor::saveAsPopup() {
		if (ImGui::BeginPopupModal("Save Command Script", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {

			ImGui::Text("Directory: ");
			ImGui::SameLine();
			ImGui::TextUnformatted(_pendingSavePath.c_str());

			if (ImGui::Button("Choose Folder...")) {
				_save.Open();
			}

			_save.Display();
			if (_save.HasSelected()) {
				_pendingSavePath = _save.GetSelected().string();
				_save.ClearSelected();
			}

			char buf[256];
			std::snprintf(buf, sizeof(buf), "%s", _pendingSaveName.c_str());
			if (ImGui::InputText("Filename", buf, sizeof(buf))) {
				_pendingSaveName = buf;
			}

			auto ensureExt = [](const std::string& name, const char* ext) {
				if (name.size() < strlen(ext) || name.substr(name.size() - strlen(ext)) != ext) {
					return name + ext;
				}
				return name; // already has extension
			};

			if (ImGui::Button("Save")) {
				std::string finalName = ensureExt(_pendingSaveName, ".scl");
				std::string fullPath = _pendingSavePath;

				if (!fullPath.empty() && fullPath.back() != '/' && fullPath.back() != '\\') {
					fullPath += "/";
				}

				fullPath += finalName;

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
			return true;
		}

		return false;
	}
}