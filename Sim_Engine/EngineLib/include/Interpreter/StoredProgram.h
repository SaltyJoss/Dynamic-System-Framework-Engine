#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include "ICommand.h"
#include "Scene/SimulationManager.h"
#include "Interpreter/CommandContextMotion.h"
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
		
		// Run the program
		void run() override;

		// Get the current instruction
		const Command* getCurrentInstruction() const;

		// Get Current line number
		int getCurrentLineNumber() const override { return _currentLineNumber; }
		void setCurrentLineNumber(int lineNumber) { _currentLineNumber = lineNumber; }

	private:
		// Bool for tracking if the program has reached the end
		bool atEnd() const;

		// Bool for tracking if there are commands left to execute
		bool commandsLeft() const;

		CmdResult updateState() override;
		int _currentLineNumber = 0;
		int PC = 0; // Program Counter

		std::vector<commands::ICommand*> _commands;

		gui::simManager* _sim = nullptr;
	};
} // namespace interpreter
