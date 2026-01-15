#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include "ICommand.h"
#include "Interpreter/MainContext.h"
#include <string>
#include <vector>

using namespace commands;

namespace interpreter {
	// Class representing a stored program in the interpreter.
	class ENGINE_API StoredProgram : public IStoredProgram {
	public:
		// Constructor
		StoredProgram(gui::simManager* sim);

		// Add a command to the program
		void add(commands::ICommand* cmd) override;

		// Load program data
		void load(ProgramData program) override;
		// Reset program to initial state
		void reset() override;
		// Clear all stored instructions
		void clear() override;
		// Start program execution
		void start() override;
		// Stop program execution
		void stop() override;
		// Puase program execution
		void pause() override;
		// Step the program by dt
		void step(double dt) override;
		// Get current program status
		ProgramStatus status() const override;

		// State checkers
		bool isRunning() const override { return _state == ProgramState::Running; }
		bool isPaused() const override { return _state == ProgramState::Paused; }
		bool isStopped() const override { return _state == ProgramState::Stopped; }

		// Get the current instruction
		const Command* getCurrentInstruction() const;

		// Get Current line number
		int getCurrentLineNumber() const override { return _currentLineNumber; }
		void setCurrentLineNumber(int lineNumber) { _currentLineNumber = lineNumber; }

		// Set default object
		void setDefaultObject(scene::Object* obj) override { _defaultObj = obj; }
		scene::Object* defaultObject() const override { return _defaultObj; }

		// Set & Get Integrator Method
		void setIntegratorMethod(IntegratorMethod method) override;
		IntegratorMethod getIntegratorMethod() const override;

	private:
		gui::simManager* _sim = nullptr;
		commands::MainContext _cntx;
		scene::Object* _defaultObj = nullptr;

		// Bool for tracking if the program has reached the end
		bool atEnd() const;
		// Bool for tracking if there are commands left to execute
		bool commandsLeft() const;

		ProgramState _state = ProgramState::Stopped;
		bool _stopRequested = false;

		CmdResult updateState() override;
		int _currentLineNumber = 0;
		int PC = 0; // Program Counter

		std::vector<commands::ICommand*> _commands;
		IntegratorMethod _integratorMethod = IntegratorMethod::Euler; // Default integrator method
	};
} // namespace interpreter
