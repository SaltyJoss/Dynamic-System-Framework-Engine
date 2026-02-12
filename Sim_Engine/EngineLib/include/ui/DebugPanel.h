#pragma once
// File:    DebugPanel.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include "imgui.h"
#include "Scene/SimulationManager.h"
#include "Scene/Camera.h"
#include <unordered_set>

#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace gui {
	struct debugLogEntry {
		std::string level;
		std::string desc;
		bool isError = false;
	};

	struct TerminalLine {
		enum class Level { Info, Warn, Error, Debug };
		Level level = Level::Info;
		std::string text;
	};

	class ENGINE_API DebugPanel {
	public:
		void render();
		void clearSimLog() { gLog.Instance().clearSimLog(); simSelectedLines.clear(); }

	private:
		bool autoScroll = true;
		std::vector<debugLogEntry> _entries;
		std::vector<debugLogEntry> _simEnteries;

		void renderLog();
		void renderSimLog();
		void renderErrorTable();

		void beginDebugPanel(const char* id, ImVec2 size = ImVec2(0, 0));
		void endDebugPanel();

		std::unordered_set<int> selectedLines;
		std::unordered_set<int> simSelectedLines;
		int lastClickedLine = -1;

		// Colour Keys (I am colour deficient so GPT has been used to generate the colour values)
		const ImVec4 traceCol	  = { 0.6275f, 0.6275f, 0.6275f, 1.0f };
		const ImVec4 debugCol	  = { 0.3137f, 0.6667f, 0.9412f, 1.0f };
		const ImVec4 infoCol	  = { 0.3137f, 0.6275f,    1.0f, 1.0f };
		const ImVec4 warnCol	  = {	 1.0f, 0.7059f,    0.0f, 1.0f };
		const ImVec4 errorCol	  = { 0.8627f, 0.2353f, 0.2745f, 1.0f };
		const ImVec4 okCol		  = {    0.0f, 0.7451f, 0.3922f, 1.0f };
		const ImVec4 failCol	  = { 0.6667f,    0.0f, 0.1961f, 1.0f };
		const ImVec4 runtimeCol	  = { 0.7451f, 0.4706f,    1.0f, 1.0f };
		const ImVec4 outputCol	  = { 0.8627f, 0.8627f, 0.8627f, 1.0f };
		const ImVec4 rotateCol    = { 0.4666f,    1.0f,    0.0f, 1.0f };
		const ImVec4 translateCol = { 0.2745f, 0.5098f, 0.7059f, 1.0f };
	};
} // namespace gui