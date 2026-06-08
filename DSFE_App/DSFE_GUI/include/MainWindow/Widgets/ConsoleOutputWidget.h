// DSFE_GUI ConsoleOutputWidget.h
#pragma once

#include <QWidget>
#include <string>

#include "Platform/Logger.h"

class QTextEdit;
class QTabWidget;
class QPushButton;

// Colours for log levels (Will eventually move to a separate file for global UI constants)
constexpr QColor TRACE_COLOUR(160, 160, 160);
constexpr QColor DEBUG_COLOUR(80, 170, 240);
constexpr QColor INFO_COLOUR(80, 160, 255);
constexpr QColor WARN_COLOUR(255, 180, 0);
constexpr QColor ERROR_COLOUR(220, 60,  70);
constexpr QColor OK_COLOUR(0, 190, 100);
constexpr QColor FAIL_COLOUR(170, 0, 50);
constexpr QColor RUNTIME_COLOUR(190, 120, 225);
constexpr QColor OUTPUT_COLOUR(220, 220, 220);
constexpr QColor ROTATE_COLOUR(120, 180, 255);
constexpr QColor TRANSLATE_COLOUR(120, 255, 180);

constexpr size_t MAX_TERMINAL_ENTRIES = 10000; // Maximum number of terminal entries to keep in memory for display

namespace widgets {
	class ConsoleOutputWidget : public QWidget {
	public:
		explicit ConsoleOutputWidget(QWidget* parent = nullptr);
		void clearSimLog();
		void updateLog();

	private:
		QTabWidget* _tabs = nullptr;
		QTextEdit* _terminalLog = nullptr;
		QTextEdit* _simLog = nullptr;

		size_t _lastTerminalCount = 0;
		size_t _lastSimCount = 0;

		bool autoScroll = true;
	};
} // namespace widgets