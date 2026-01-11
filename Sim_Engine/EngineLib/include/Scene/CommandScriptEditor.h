#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <unordered_set>

#include <imgui.h>
#include "Platform/imguiWidgets.h"
#include <imfilebrowser.h>

#include "Platform/Logger.h"


namespace gui {
	class ENGINE_API CommandScriptEditor {
	public:
		CommandScriptEditor();
		~CommandScriptEditor();

		// Menu Render
		void drawMenus();
		// Main render function
		void render();

		// Renders the command script editor window
		void renderEnvironment();
		// Renders a command instruction syntax helper
		void renderCmdInstructions();

		// Load command script from a file
		bool tryLoadFromDialog();
		// Save command script to the current file
		bool trySaveScriptToFile(const std::string& filepath);
		// Save command script via "Save As" dialog
		bool saveAsPopup();

		// Set the command script from a single string
		void setScript(const std::string& s) { _scriptText = s; }
		// Get the current script as a single string
		std::string getScript() const { return _scriptText; }

	private:
		// File browser for loading/saving scripts
		ImGui::FileBrowser _load;
		ImGui::FileBrowser _save;

		// Current script file directory path
		std::string _currentScriptPath;
		// Current script file path
		std::string _currentScriptFile;

		std::string _pendingSaveName = "Untitled.scl";
		std::string _pendingSavePath = "Engine/assets/scripts";

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
	};
}