#pragma once
// File:    CommandScriptEditor.h
// GitHub:  SaltyJoss
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
		CommandScriptEditor(gui::SimManager* sims);
		~CommandScriptEditor();

		// Menu Render
		void drawMenus();
		// Main render function
		void render();

	private:
		// Simulation manager reference
		gui::SimManager* _sim;

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

		void renderEnvironment();
		void renderCmdInstructions();

		void terminateScript(const char* reason, bool fault);

		bool tryLoadFromDialog();
		bool trySaveScriptToFile(const std::string& filepath);

		void renderSaveAsPopup();

		void beginEditorPanel(const char* id);
		void endEditorPanel();

		void setScript(const std::string& s) { _scriptText = s; }
		std::string getScript() const { return _scriptText; }

		// Command script contents as lines
		std::vector<std::string> _script;
		// Full script text
		std::string _scriptText;

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
} // namespace gui