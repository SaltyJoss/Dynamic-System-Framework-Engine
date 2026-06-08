// DSFE_GUI ConsoleOutputWidget.h
#pragma once

#include <QWidget>
#include <string>

#include "Platform/Logger.h"

class QTextEdit;
class QTabWidget;
class QPushButton;

// Colours for log levels (Will eventually move to a separate file for global UI constants)
#define TRACE_COLOUR   QColor(160, 160, 160)
#define DEBUG_COLOUR   QColor(80,  170, 240)
#define INFO_COLOUR    QColor(80,  160, 255)
#define WARN_COLOUR    QColor(255, 180, 0)
#define ERROR_COLOUR   QColor(220, 60,  70)
#define OK_COLOUR	   QColor(0,   190, 100)
#define FAIL_COLOUR	   QColor(170, 0,   50)
#define RUNTIME_COLOUR QColor(190, 120, 225)
#define OUTPUT_COLOUR  QColor(220, 220, 220)

#define ROTATE_COLOUR    QColor(120, 180, 255)
#define TRANSLATE_COLOUR QColor(120, 255, 180)

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
		QPushButton* _clearTerminalButton = nullptr;

		size_t _lastTerminalCount = 0;
		size_t _lastSimCount = 0;

		bool autoScroll = true;
	};
} // namespace widgets