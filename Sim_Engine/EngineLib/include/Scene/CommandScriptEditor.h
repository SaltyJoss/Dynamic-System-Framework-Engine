#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <unordered_set>
#include "Scene/SimulationManager.h"

#include "Interpreter/RunWrapper.h"

#include <imgui.h>
#include "Platform/imguiWidgets.h"
#include <imfilebrowser.h>

#include "Platform/Logger.h"


namespace gui {
	class ENGINE_API CommandScriptEditor {
	public:
		CommandScriptEditor(gui::simManager* sims);
		~CommandScriptEditor();

		// Menu Render
		void drawMenus();
		// Main render function
		void render();

		// Run/Stop button handler
		void runButtonHandler();

	private:
		// Simulation manager reference
		gui::simManager* _sim;

		// Interpreter components
		interpreter::Parser* _parser;
		interpreter::IStoredProgram* _program;
		interpreter::RunWrapper* _wrapper;

		// File browser for loading/saving scripts
		ImGui::FileBrowser _load;
		ImGui::FileBrowser _save;

		// Current script file directory path
		std::string _currentScriptPath;
		// Current script file path
		std::string _currentScriptFile;

		std::string _pendingSaveName = "Untitled.scl";
		std::string _pendingSavePath = "Engine/assets/scripts";
		bool _requestSaveAsPopup = false;

		// Renders the command script editor window
		void renderEnvironment();
		// Renders a command instruction syntax helper
		void renderCmdInstructions();

		// Load command script from a file
		bool tryLoadFromDialog();
		// Save command script to the current file
		bool trySaveScriptToFile(const std::string& filepath);
		// Save command script via "Save As" dialog
		void renderSaveAsPopup();

		// Set the command script from a single string
		void setScript(const std::string& s) { _scriptText = s; }
		// Get the current script as a single string
		std::string getScript() const { return _scriptText; }

		void beginEditorPanel(const char* id);
		void endEditorPanel();

		// Command script contents as lines
		std::vector<std::string> _script;
		// Full script text
		std::string _scriptText;

		// Command script running state
		bool _isRunning;

		// For log selection
		std::unordered_set<int> selectedLines;
		// Last clicked line index
		int lastClickedLine = -1;

		// Colours
		const ImVec4 CMD_COL = ImVec4(0.95f, 0.85f, 0.25f, 1.0f); // yellow
		const ImVec4 VEC_COL = ImVec4(0.40f, 0.65f, 0.95f, 1.0f); // blue
		const ImVec4 ARG_COL = ImVec4(0.95f, 0.55f, 0.25f, 1.0f); // orange
		const ImVec4 DESC_COL = ImVec4(0.60f, 0.60f, 0.60f, 1.0f); // grey

	};
}