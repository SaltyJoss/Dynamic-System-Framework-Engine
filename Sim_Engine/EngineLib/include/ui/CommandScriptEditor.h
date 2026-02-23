#pragma once
// File:    CommandScriptEditor.h
// GitHub:  SaltyJoss
#include "EngineCore.h"

#include <future>
#include <mutex>
#include <vector>
#include "Platform/StudyRunner.h"

#include <MathLibAPI.h>
#include <core/Types.h>
#include <unordered_set>
#include "Scene/SimulationManager.h"

#include "Interpreter/RunWrapper.h"

#include <imgui.h>
#include "Platform/imguiWidgets.h"
#include <imfilebrowser.h>

#include "Platform/Logger.h"

struct ENGINE_API StudyResult; // forward declaration to avoid circular dependency

namespace gui {
	struct ActiveRun {
		std::future<StudyResult> fut;
		std::string tag;
	};

	class ENGINE_API CommandScriptEditor {
	public:
		CommandScriptEditor(gui::SimManager* sims);
		~CommandScriptEditor();

		// Menu Render
		void drawMenus();
		// Main render function
		void render();

	private:
		// Active runs management
		std::mutex _activeRunsMutex; // Mutex for synchronizing access to active runs
		std::vector<ActiveRun> _activeRuns; // Vector to hold active runs and their futures

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

		void runButtonHandler();

		void renderEnvironment();
		void renderCmdInstructions();

		void terminateScript(const char* reason, bool fault);

		bool tryLoadFromDialog();
		bool trySaveScriptToFile(const std::string& filepath);

		void pollRuns();
		void launchBackgroundRun(const std::string& code, const std::string& tag);

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