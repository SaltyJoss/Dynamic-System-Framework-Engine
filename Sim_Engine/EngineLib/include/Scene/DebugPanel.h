#pragma once

// =============================================
//			File: DebugPanel.h
// =============================================
// GUI Debug Panel for displaying logs and errors.
//
// Summary:
// =============================================
//
// public:
// --------------------------------------------
// DebugPanel()
//      -> Constructor for the DebugPanel class.
// void render()
//      -> Renders the debug panel GUI scene.
// --------------------------------------------
//
// private:
// --------------------------------------------
// void renderLog()
//      -> Renders the log section of the debug panel.
// void renderErrorTable()
//      -> Renders the error table section of the debug panel.
// --------------------------------------------
//
// Internal State Variables:
// --------------------------------------------
// bool autoScroll
//      -> Indicates whether auto-scrolling is enabled for the log.
// std::vector<debugLogEntry> _entries
//      -> Vector storing the log entries.
// --------------------------------------------
//
// Colour keys for different log levels:
// --------------------------------------------
// const ImVec4 traceCol
//		-> Grey
// 		-> Colour for trace log entries.
// const ImVec4 debugCol
//		-> Light blue
//      -> Colour for debug log entries.
// const ImVec4 infoCol
//		-> Blue
//      -> Colour for info log entries.
// const ImVec4 warnCol
// 		-> Orange
//      -> Colour for warning log entries.
// const ImVec4 errorCol
//		-> Red
// 		-> Colour for error log entries.
// const ImVec4 okCol
//		-> Green
//      -> Colour for success log entries.
// const ImVec4 failCol
// 		-> Dark red
//      -> Colour for failure log entries.
// const ImVec4 runtimeCol
// 		-> Purple
//      -> Colour for runtime log entries.
// const ImVec4 outputCol
// 		-> Light grey
//      -> Colour for general output log entries.
// --------------------------------------------
//
// ============================================
//              GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"

#include "imgui.h"
#include "Scene/SimulationManager.h"
#include "Camera.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace gui {
	struct debugLogEntry {
		std::string level;
		std::string desc;
		bool isError = false;
	};

	class ENGINE_API DebugPanel {
	public:
		void render();

	private:
		bool autoScroll = true;
		std::vector<debugLogEntry> _entries;

		void renderLog();
		void renderErrorTable();

		std::unordered_set<int> selectedLines;
		int lastClickedLine = -1;

		// Colour Keys (I am colour deficient so GPT has been used to generate the colour values)
		const ImVec4 traceCol	= { 0.6275f, 0.6275f, 0.6275f, 1.0f };
		const ImVec4 debugCol	= { 0.3137f, 0.6667f, 0.9412f, 1.0f };
		const ImVec4 infoCol	= { 0.3137f, 0.6275f,    1.0f, 1.0f };
		const ImVec4 warnCol	= {	   1.0f, 0.7059f,    0.0f, 1.0f };
		const ImVec4 errorCol	= { 0.8627f, 0.2353f, 0.2745f, 1.0f };
		const ImVec4 okCol		= {    0.0f, 0.7451f, 0.3922f, 1.0f };
		const ImVec4 failCol	= { 0.6667f,    0.0f, 0.1961f, 1.0f };
		const ImVec4 runtimeCol	= { 0.7451f, 0.4706f,    1.0f, 1.0f };
		const ImVec4 outputCol	= { 0.8627f, 0.8627f, 0.8627f, 1.0f };
	};
}