// DSFE_GUI DSLEditorWidget.cpp
#include "Widgets/DSLEditorWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QFile>
#include <QTimer>
#include <QSignalBlocker>

#include "Simulation/SimulationManager.h"
#include "Platform/Paths.h"

#include "Numerics/IntegrationMethods.h"
#include "DSL/StoredProgram.h"

#include "Widgets/ConsoleOutputWidget.h"
#include "DSL/DSLSyntaxHighlighter.h"

#include "EngineLib/LogMacros.h"

namespace widgets {
	DSLEditorWidget::DSLEditorWidget(gui::SimulationManager* sim, ConsoleOutputWidget* log, QWidget* parent)
		: QWidget(parent), _sim(sim), _log(log), _scriptWorkingDir((paths::assets() / "DSLScripts").string())
	{
		auto* rootLayout = new QVBoxLayout(this);
		auto* scriptStateLayout = new QHBoxLayout();

		_tabs = new QTabWidget(this);
		rootLayout->addWidget(_tabs);

		buildEditorTab();
		buildHelpTab();

		_statusLabel = new QLabel("<b>State:</b> Idle", this);
		auto* pipe1 = new QLabel("|", this);
		auto* pipe2 = new QLabel("|", this);
		_scriptLinesLabel = new QLabel("<b>Lines:</b> 0", this);
		_scriptCharsLabel = new QLabel("<b>Chars:</b> 0", this);
		_runStopButton = new QPushButton("Run", this);

		scriptStateLayout->addWidget(_statusLabel);
		scriptStateLayout->addWidget(pipe1);
		scriptStateLayout->addWidget(_scriptLinesLabel);
		scriptStateLayout->addWidget(pipe2);
		scriptStateLayout->addWidget(_scriptCharsLabel);
		scriptStateLayout->addStretch();
		scriptStateLayout->addWidget(_runStopButton);

		rootLayout->addLayout(scriptStateLayout);

		scriptStateLayout->setContentsMargins(6, 6, 6, 6);
		scriptStateLayout->setSpacing(8);
		rootLayout->setContentsMargins(6, 6, 6, 6);
		rootLayout->setSpacing(6);

		_stateTimer = new QTimer(this);
		connect(_stateTimer, &QTimer::timeout, this, [this]() { pollScriptState(); });
		_stateTimer->start(100); // Poll every 100 ms

		connect(_runStopButton, &QPushButton::clicked, this, [this]() { _sim->isScriptRunning() ? stopScript() : runScript(); });
		pollScriptState();
	}

	static std::string filenameFromPath(const std::string& path) {
		size_t lastSlash = path.find_last_of("/\\");
		if (lastSlash == std::string::npos) { return path; }
		return path.substr(lastSlash + 1);
	}

	bool DSLEditorWidget::loadScript(const QString& fileName) {
		QFile file(fileName);
		std::string nameStr = filenameFromPath(fileName.toStdString()).c_str();
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
			LOG_ERROR("Failed to open script file: %s", nameStr);
			return false;
		}
		_scriptEditor->setPlainText(file.readAll());
		file.close();
		_currentScriptPath = fileName;

		_scriptLinesLabel->setText(QString("<b>Lines:</b> %1").arg(_scriptEditor->toPlainText().split('\n').size()));
		_scriptCharsLabel->setText(QString("<b>Chars:</b> %1").arg(_scriptEditor->toPlainText().size()));

		LOG_INFO("DSL script loaded from file: %s", nameStr);
		D_INFO("DSL script loaded from file: %s", nameStr);

		return true;
	}

	bool DSLEditorWidget::saveScript(const QString& fileName) {
		QFile file(fileName);
		std::string nameStr = filenameFromPath(fileName.toStdString()).c_str();
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
			LOG_ERROR("Failed to open script file for writing: %s", nameStr);
			return false;
		}
		file.write(_scriptEditor->toPlainText().toUtf8());
		file.close();
		_currentScriptPath = fileName;

		LOG_INFO("DSL script saved to file: %s", nameStr);
		D_INFO("DSL script saved to file: %s", nameStr);

		return true;
	}

	void DSLEditorWidget::setStatusMessage(const QString& message) { if (_statusLabel) { _statusLabel->setText("<b>State:</b> " + message); } }

	void DSLEditorWidget::buildEditorTab() {
		_editorTab = new QWidget(this);
		auto* layout = new QVBoxLayout(_editorTab);
		_scriptEditor = new QTextEdit(_editorTab);
		_highlighter = new DSLSyntaxHighlighter(_scriptEditor->document());
		connect(_scriptEditor, &QTextEdit::textChanged, this, [this]() { if (onContentChanged) { onContentChanged(); } });
		layout->addWidget(_scriptEditor);
		_tabs->addTab(_editorTab, "Script Editor");
	}

	void DSLEditorWidget::buildHelpTab() {
		_helpTab = new QWidget(this);
		auto* layout = new QVBoxLayout(_helpTab);
		_dslHelp = new QTextEdit(_helpTab);
		_dslHelp->setReadOnly(true);
	
		// TODO: Use a markdown viewer or similar to format and display this help content in an accessible way, with sections, examples, etc.
		//  * (also means I can make a website with the same content)
		_dslHelp->setPlainText(
			"trajSet(...)\n"
			"trajClear()\n"
			"rotateJointTo(...)\n"
			"rotateJointBy(...)\n"
			"load(...)\n"
			"set(...)\n"
			"start()\n"
			"stop()\n"
			"wait(...)\n"
		);

		layout->addWidget(_dslHelp);
		_tabs->addTab(_helpTab, "DSL Help");
	}

	void DSLEditorWidget::runScript() {
		if (_sim->isScriptRunning()) { return; }
		if (_scriptEditor->toPlainText().isEmpty()) { setStatusMessage("Empty script"); return; }

		_sim->setActiveProgram(nullptr);
		delete _wrapper; _wrapper = nullptr;
		delete _parser; _parser = nullptr;
		delete _program; _program = nullptr;

		_program = new dsl::StoredProgram(_sim->simCore());
		_parser = new dsl::Parser(_program);
		_wrapper = new dsl::RunWrapper(_parser, _program);

		_scriptText = _scriptEditor->toPlainText().toStdString();

		if (_log) { _log->clearSimLog(); }
		_sim->setActiveProgram(_program);
		_sim->setScriptRunning(true);
		_sim->setLastScriptText(_scriptText);
		LOG_INFO("DSL script started."); D_INFO("DSL script started.");

		std::string code = _scriptText;
		if (!code.empty() && code.back() == '\0') { code.pop_back(); }
		_wrapper->runProgram(_scriptText);

		updateButtonState(true);
	}

	void DSLEditorWidget::runButtonHandler() { _sim->isScriptRunning() ? stopScript() : runScript(); }

	QString DSLEditorWidget::scriptText() const {
        return _scriptEditor ? _scriptEditor->toPlainText() : QString();
    }

    void DSLEditorWidget::setScriptText(const QString& text) {
        if (_scriptEditor) {
			QSignalBlocker block(_scriptEditor); // Block signals to prevent triggering onContentChanged
			_scriptEditor->setPlainText(text);
		}
        _scriptText = text.toStdString();
        _currentScriptPath.clear();          // embedded text, no file identity
        _loadedFromFile = false;
    }

	void DSLEditorWidget::stopScript() {
		if (!_sim->isScriptRunning()) { return; }
		if (_sim->activeProgram()) {
			_sim->activeProgram()->stop();
			_sim->activeProgram()->reset();
		}
		_sim->setActiveProgram(nullptr);
		_sim->stopSimulation();
		_sim->setScriptRunning(false);
		LOG_INFO("DSL script stopped."); D_INFO("DSL script stopped.");

		updateButtonState(false);
	}

	void DSLEditorWidget::terminateScript(const char* reason, bool fault) {
		_sim->setActiveProgram(nullptr);
		_sim->stopSimulation();
		_sim->setScriptRunning(false);

		if (fault) { LOG_WARN("%s", reason); D_FAIL("%s", reason); }
		else { LOG_INFO("%s", reason); D_SUCCESS("%s", reason); }

		delete _wrapper; _wrapper = nullptr;
		delete _parser;  _parser = nullptr;
		delete _program; _program = nullptr;
		updateButtonState(false);
		setStatusMessage(reason);
		_statusLabel->setStyleSheet(fault ? "color: rgb(180,40,40);" : "");
	}

	void DSLEditorWidget::pollScriptState() {
		updateButtonState(_sim->isScriptRunning());
		if (!_sim->isScriptRunning()) { return; }
		auto* prog = _sim->activeProgram();
		if (!prog) { terminateScript("No active program found in simulation manager.", true); return; }
		if (prog->isEmpty()) { terminateScript("Command script stopped -> program is empty.", true); return; }
		if (prog->isFaulted()) { terminateScript("Command script stopped due to fault.", true); return; }
		if (prog->isCompleted()) { terminateScript("Command script completed.", false); return; }
		if (prog->isStopped() && !prog->isCompleted()) { terminateScript("Command script stopped.", true); return; }
	}

	void DSLEditorWidget::updateButtonState(bool running) {
		_runStopButton->setText(running ? "Stop" : "Run");
		_runStopButton->setStyleSheet(running ? "background-color: rgb(180,40,40);" : "");
	}

} // namespace widgets