#ifndef DEBUG_PANEL_H
#define DEBUG_PANEL_H

#include "ch.h"
namespace gui {
	class DebugPanel {
	public:
		void Render(ImVec2 winSize, ImVec2 padding, float debugHeight, float ctrlPanelWidth);
		void AddLog(const std::string& msg, bool error = false);

	private:
		struct LogEntry { std::string text; bool isError; };
		std::vector<LogEntry> entries;
		bool autoScroll = true;

		ImVec4 GetColour(bool error) const;
		void DebugLog();
		void ErrorList();
	};
}


#endif