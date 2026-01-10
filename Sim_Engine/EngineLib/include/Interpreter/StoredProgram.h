#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include "CommandFactory.h"
#include "CommandContext.h"
#include <string>
#include <vector>

using namespace commands;

namespace interpreter {
	// Class representing a stored program in the interpreter.
	class ENGINE_API StoredProgram : IStoredProgram {
	public:
		// Constructor
		StoredProgram(CommandFactory& factory, CommandContext& cntx);

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
	private:
		// Bool for tracking if the program is running
		bool _hasActiveCommand() const;
		// Bool for tracking if the program has reached the end
		bool _atEnd() const;
		// Get the current instruction
		const Instruction* _currentInstruction() const;
		
		// Method for handling faults (Not necessary yet, but WILL BE)
		void fault(const std::string& message);
		// Method for completing the program
		void spawnNextCommand();
		// Method for clearing the active command
		void clearActiveCommand();

	};
} // namespace interpreter
