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

#include "Platform/StudyRunner.h"
#include "Interpreter/RunWrapper.h"

#include <MathLibAPI.h>
#include <core/Types.h>
#include <unordered_set>

#include "Scene/SimulationManager.h"
#include "Platform/Paths.h"

#include "Numerics/IntegrationMethods.h"
#include "Interpreter/StoredProgram.h"

#include "EngineLib/LogMacros.h"

namespace widgets {
	DSLEditorWidget::DSLEditorWidget(gui::SimManager* sim, QWidget* parent) 
		: QWidget(parent), _sim(sim), _parser(nullptr), _program(nullptr), _wrapper(nullptr)
	{
		auto* rootLayout = new QVBoxLayout(this);
		auto* buttonLayout = new QHBoxLayout();

		_runStopButton = new QPushButton("Run", this); // May also move to the menu, but I want to test the logic first

		buttonLayout->addStretch();
		buttonLayout->addWidget(_runStopButton);

		rootLayout->addLayout(buttonLayout);

		_tabs = new QTabWidget(this);
		rootLayout->addWidget(_tabs);

		buildEditorTab();
		buildHelpTab();

		_statusLabel = new QLabel("Idle", this);
		rootLayout->addWidget(_statusLabel);

		_stateTimer = new QTimer(this);
		connect(_stateTimer, &QTimer::timeout, this, [this]() { pollScriptState(); });
		_stateTimer->start(100); // Poll every 100 ms

		connect(_runStopButton, &QPushButton::clicked, this, [this]() {
			if (_sim->isScriptRunning()) { stopScript(); }
			else { runScript(); }
		});

		pollScriptState();
	}

	bool DSLEditorWidget::loadScript(const QString& fileName) {
		QFile file(fileName);
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
			LOG_ERROR("Failed to open script file: %s", fileName.toStdString().c_str());
			return false;
		}
		_scriptEditor->setPlainText(file.readAll());
		file.close();
		_currentScriptPath = fileName;

		LOG_INFO("DSL script loaded from file: %s", fileName.toStdString().c_str());
		D_INFO("DSL script loaded from file: %s", fileName.toStdString().c_str());

		return true;
	}

	bool DSLEditorWidget::saveScript(const QString& fileName) {
		QFile file(fileName);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
			LOG_ERROR("Failed to open script file for writing: %s", fileName.toStdString().c_str());
			return false;
		}
		file.write(_scriptEditor->toPlainText().toUtf8());
		file.close();
		_currentScriptPath = fileName;

		LOG_INFO("DSL script saved to file: %s", fileName.toStdString().c_str());
		D_INFO("DSL script saved to file: %s", fileName.toStdString().c_str());

		return true;
	}

	void DSLEditorWidget::setStatusMessage(const QString& message) { if (_statusLabel) { _statusLabel->setText(message); } }

	void DSLEditorWidget::buildEditorTab() {
		_editorTab = new QWidget(this);
		auto* layout = new QVBoxLayout(_editorTab);
		_scriptEditor = new QTextEdit(_editorTab);
		layout->addWidget(_scriptEditor);
		_tabs->addTab(_editorTab, "Script Editor");
	}

	void DSLEditorWidget::buildHelpTab() {
		_helpTab = new QWidget(this);
		auto* layout = new QVBoxLayout(_helpTab);
		_dslHelp = new QTextEdit(_helpTab);
		_dslHelp->setReadOnly(true);


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
		LOG_INFO("DSL script started."); D_INFO("DSL script started.");

		_sim->setActiveProgram(nullptr);
		delete _wrapper; _wrapper = nullptr;
		delete _parser; _parser = nullptr;
		delete _program; _program = nullptr;

		_program = new interpreter::StoredProgram(_sim->simCoreInterface());
		_parser = new interpreter::Parser(_program);
		_wrapper = new interpreter::RunWrapper(_parser, _program);

		_scriptText = _scriptEditor->toPlainText().toStdString();

		_sim->setActiveProgram(_program);
		_sim->setScriptRunning(true);
		_sim->setLastScriptText(_scriptText);

		std::string code = _scriptText;
		if (!code.empty() && code.back() == '\0') { code.pop_back(); }

		_wrapper->runProgram(_scriptText);

		updateButtonState(true);
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
		_statusLabel->setStyleSheet(fault ? "color: rgb(180,40,40);" : "color: rgb(40,180,40);");
	}

	void DSLEditorWidget::pollScriptState() {
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
		_runStopButton->setStyleSheet(running ? "background-color: rgb(180,40,40);" : "background-color: rgb(40,180,40);");
	}

} // namespace widgets