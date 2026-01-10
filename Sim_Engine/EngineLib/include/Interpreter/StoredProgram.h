#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include <string>
#include <vector>

namespace interpreter {
	// Class representing a stored program in the interpreter.
	class ENGINE_API StoredProgram : IStoredProgram {
	public:
		// Get the current program counter
		int getPC() const { return _pc; }
		// Set the program counter
		void setPC(int pc) { _pc = pc; }

		// Add instruction to the program
		void addInstruction(ICommand cmd);
		// Get instruction at specified index
		std::string getInstruction(int index) const;
		// Get total number of stored instructions
		int getInstructionCount() const;

		// Control program execution
		void run();
		// Stop program execution
		void stop();
		// Reset program to initial state
		void reset();
		// Clear all stored instructions
		void clear();

		// Evaluate Expression
		std::string evaluateExpression(const std::string& expr);
	private:
		std::vector<std::string> _instructions; // Vector storing the instructions
		int _pc = 0;                 // Current program counter
		bool _isRunning = false;                 // Flag indicating if the program is running
	};
} // namespace interpreter
