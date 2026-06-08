// DSFE_GUI ConsoleOutputWidget.cpp
#include "Widgets/ConsoleOutputWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QTimer>
#include <QScrollBar>
#include <QTextCursor>
#include <QTextCharFormat>

#include "EngineLib/LogMacros.h"

namespace widgets {
	QColor levelColour(LogLevel level) {
		switch (level) {
			case LogLevel::Trace:   return TRACE_COLOUR;
			case LogLevel::Debug:   return DEBUG_COLOUR;
			case LogLevel::Info:    return INFO_COLOUR;
			case LogLevel::Warning: return WARN_COLOUR;
			case LogLevel::Error:   return ERROR_COLOUR;
			case LogLevel::Success: return OK_COLOUR;
			case LogLevel::Fail:    return FAIL_COLOUR;
			case LogLevel::Runtime: return RUNTIME_COLOUR;
			case LogLevel::Output:
			default:				return OUTPUT_COLOUR;
		}
	}
	QString levelString(LogLevel level) {
		switch (level) {
			case LogLevel::Trace:   return "TRACE";
			case LogLevel::Debug:   return "DEBUG";
			case LogLevel::Info:    return "INFO";
			case LogLevel::Warning: return "WARN";
			case LogLevel::Error:   return "ERROR";
			case LogLevel::Success: return "OK";
			case LogLevel::Fail:    return "FAIL";
			case LogLevel::Runtime: return "RUNTIME";
			case LogLevel::Output:
			default:				return "OUTPUT";
		}
	}
	QColor simColour(simLogLevel level) {
		switch (level) {
			case simLogLevel::Rotate:	 return ROTATE_COLOUR;
			case simLogLevel::Translate: return TRANSLATE_COLOUR;
			case simLogLevel::Fail:		 return FAIL_COLOUR;
			case simLogLevel::Error:	 return ERROR_COLOUR;
			case simLogLevel::Success:	 return OK_COLOUR;
			case simLogLevel::Runtime:
			default:					 return RUNTIME_COLOUR;
		}
	}

	QString simString(simLogLevel level) {
		switch (level) {
			case simLogLevel::Rotate:	 return "ROTATE";
			case simLogLevel::Translate: return "TRANSLATE";
			case simLogLevel::Fail:		 return "FAIL";
			case simLogLevel::Error:	 return "ERROR";
			case simLogLevel::Success:	 return "SUCCESS";
			case simLogLevel::Runtime:
			default:					 return "RUNTIME";
		}
	}

	ConsoleOutputWidget::ConsoleOutputWidget(QWidget* parent) : QWidget(parent) {
		auto* rootLayout = new QVBoxLayout(this);

		_tabs = new QTabWidget(this);
		_terminalLog = new QTextEdit(this);
		_terminalLog->setReadOnly(true);
		_simLog = new QTextEdit(this);
		_simLog->setReadOnly(true);
		_tabs->addTab(_terminalLog, "Terminal");
		_tabs->addTab(_simLog, "Simulation");
		rootLayout->addWidget(_tabs);

		auto* buttonLayout = new QHBoxLayout();
		_clearTerminalButton = new QPushButton("Clear Terminal Log", this);
		buttonLayout->addWidget(_clearTerminalButton);
		rootLayout->addLayout(buttonLayout);

		connect(_clearTerminalButton, &QPushButton::clicked, this, [this]() {
			gLog.Instance().clear();
			_terminalLog->clear();
			_lastTerminalCount = 0;
		});

		auto* timer = new QTimer(this);
		connect(timer, &QTimer::timeout, this, &ConsoleOutputWidget::updateLog);
		timer->start(100); // Update every 100 ms
	}

	void ConsoleOutputWidget::clearSimLog() {
		gLog.Instance().clearSimLog();
		_simLog->clear();
		_lastSimCount = 0;
	}

	void ConsoleOutputWidget::updateLog() {
		const auto& entries = gLog.Instance().Entries();
		while (_lastTerminalCount < entries.size()) {
			const auto& e = entries[_lastTerminalCount];
			QTextCursor cursor = _terminalLog->textCursor();
			cursor.movePosition(QTextCursor::End);
			QTextCharFormat lvlFmt;
			lvlFmt.setForeground(levelColour(e.level));
			cursor.insertText(QString("[%1] ").arg(levelString(e.level)), lvlFmt);
			QTextCharFormat msgFmt;
			msgFmt.setForeground(QColor(220, 220, 220));
			cursor.insertText(QString::fromStdString(e.message), msgFmt);
			cursor.insertBlock();
			++_lastTerminalCount;
		}
		
		const auto& simEntries = gLog.Instance().SimEntries();
		while (_lastSimCount < simEntries.size()) {
			const auto& e = simEntries[_lastSimCount];
			QTextCursor cursor = _simLog->textCursor();
			cursor.movePosition(QTextCursor::End);
			QTextCharFormat lvlFmt;
			lvlFmt.setForeground(simColour(e.level));
			cursor.insertText(QString("[%1] ").arg(simString(e.level)), lvlFmt);
			QTextCharFormat msgFmt;
			msgFmt.setForeground(QColor(220, 220, 220));
			cursor.insertText(QString::fromStdString(e.message), msgFmt);
			cursor.insertBlock();
			++_lastSimCount;
		}

		if (autoScroll) {
			auto* tBar = _terminalLog->verticalScrollBar();
			tBar->setValue(tBar->maximum());
			auto* sBar = _simLog->verticalScrollBar();
			sBar->setValue(sBar->maximum());
		}
	}
} // namespace widgets