// DSFE_GUI DSLEditorWidget.h
#pragma once

#include <QWidget>

#include <future>
#include <mutex>
#include <vector>
#include <string>

class QTextEdit;
class QLabel;
class QPushButton;
class QTabWidget;
class QTimer;

namespace gui { class SimManager; }
namespace interpreter {
	class Parser;
	class IStoredProgram;
	class RunWrapper;
}

namespace widgets {
	class DSLEditorWidget : public QWidget {
	public:
		explicit DSLEditorWidget(gui::SimManager* sim, QWidget* parent = nullptr);

		bool loadScript(const QString& fileName);
		bool saveScript(const QString& fileName);

	private:
		void runScript();
		void stopScript();

		void buildEditorTab();
		void buildHelpTab();

		void terminateScript(const char* reason, bool fault);
		void pollScriptState();
		void updateButtonState(bool running);
		void setStatusMessage(const QString& message);

		void setScript(const std::string& s) { _scriptText = s; }
		std::string getScript() const { return _scriptText; }

		std::string _scriptText;
		
		gui::SimManager* _sim = nullptr;
		interpreter::Parser* _parser;
		interpreter::IStoredProgram* _program;
		interpreter::RunWrapper* _wrapper;

		QTabWidget* _tabs = nullptr;
		QWidget* _editorTab = nullptr;
		QWidget* _helpTab = nullptr;
		QTextEdit* _scriptEditor = nullptr;
		QTextEdit* _dslHelp = nullptr;
		QPushButton* _runStopButton = nullptr;
		QLabel* _statusLabel = nullptr;
		QString _currentScriptPath;
		QTimer* _stateTimer = nullptr;
	};
} // namespace widgets