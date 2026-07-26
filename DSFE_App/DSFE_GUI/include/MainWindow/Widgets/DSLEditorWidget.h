// DSFE_GUI DSLEditorWidget.h
#pragma once

#include <QWidget>

#include <future>
#include <mutex>
#include <vector>
#include <functional>
#include <string>
#include "Platform/StudyRunner.h"

#include <core/Types.h>
#include <unordered_set>
#include "Interpreter/RunWrapper.h"

class QTextEdit;
class QLabel;
class QPushButton;
class QTabWidget;
class QTimer;

namespace gui { class SimulationManager; }
namespace interpreter {
	class Parser;
	class IStoredProgram;
}

namespace runs {
	struct ActiveRun {
		std::future<StudyResult> fut;
		std::string tag;
	};
}

namespace widgets {
	class ConsoleOutputWidget;
	class DSLSyntaxHighlighter;

	class DSLEditorWidget : public QWidget {
	public:
		explicit DSLEditorWidget(gui::SimulationManager* sim, ConsoleOutputWidget* _log, QWidget* parent = nullptr);

		bool loadScript(const QString& fileName);
		bool saveScript(const QString& fileName);
		void runButtonHandler();

		QString scriptText() const;
        void setScriptText(const QString& text);
        QString currentScriptPath() const { return _currentScriptPath; }
        void stopScript();

		std::function<void()> onContentChanged;

	private:
		std::mutex _activeRunsMutex; // Mutex for synchronizing access to active runs
		std::vector<runs::ActiveRun> _activeRuns; // Vector to hold active runs and their futures

		gui::SimulationManager* _sim = nullptr;
		interpreter::Parser* _parser = nullptr;
		interpreter::IStoredProgram* _program = nullptr;
		interpreter::RunWrapper* _wrapper = nullptr;

		ConsoleOutputWidget* _log = nullptr;
		DSLSyntaxHighlighter* _highlighter = nullptr;

		void runScript();

		void buildEditorTab();
		void buildHelpTab();

		void terminateScript(const char* reason, bool fault);
		void pollScriptState();
		void updateButtonState(bool running);
		void setStatusMessage(const QString& message);

		void setScript(const std::string& s) { _scriptText = s; }
		std::string getScript() const { return _scriptText; }

		bool _loadedFromFile = false;

		std::string _scriptText;
		std::string _scriptWorkingDir;
		std::vector<std::string> _script;
		int lastClickedLine = -1;

		QTabWidget* _tabs = nullptr;
		QWidget* _editorTab = nullptr;
		QWidget* _helpTab = nullptr;
		QTextEdit* _scriptEditor = nullptr;
		QTextEdit* _dslHelp = nullptr;
		QPushButton* _runStopButton = nullptr;
		QLabel* _statusLabel = nullptr;
		QLabel* _scriptLinesLabel = nullptr;
		QLabel* _scriptCharsLabel = nullptr;
		QString _currentScriptPath;
		QTimer* _stateTimer = nullptr;
	};
} // namespace widgets