#pragma once

#include "EngineCore.h"
#include <string>
#include <vector>
#include "ICommand.h"

namespace interpreter {
	class ENGINE_API IStoredProgram {
		const std::vector<ICommand>& commands() const;

		// Get the current program counter
		virtual int getPC() const = 0;
		// Set the program counter
		virtual void setPC(int pc) = 0;
		// Add instruction to the program
		virtual void addInstruction(ICommand cmd) = 0;
		// Get instruction at specified index
		virtual std::string getInstruction(int index) const = 0;
		// Get total number of stored instructions
		virtual int getInstructionCount() const = 0;

		// Control program execution
		virtual void run() = 0;
		// Stop program execution
		virtual void stop() = 0;
		// Reset program to initial state
		virtual void reset() = 0;
		// Clear all stored instructions
		virtual void clear() = 0;

		// Evaluate Expression
		virtual std::string evaluateExpression(const std::string& expr) = 0;
	};

} // namespace interpreter