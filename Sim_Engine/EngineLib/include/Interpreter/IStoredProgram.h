#pragma once

#include "EngineCore.h"
#include "ProgramData.h"
#include <string>
#include <vector>

namespace interpreter {
	// Enum representing the state of the program
	enum ProgramState {
		Empty,
		Loaded,
		Running,
		Puased,
		Stopped,
		Completed,
		Faulted
	};

	// Struct for program status
	struct ProgramStatus {
		ProgramState state;
		size_t pc;
	};

	// IStoredProgram interface
	class ENGINE_API IStoredProgram {
		// Virtual destructor
		virtual ~IStoredProgram() = default;

		// Load program data
		virtual void load(ProgramData program) = 0;
		// Reset program to initial state
		virtual void reset() = 0;
		// Clear all stored instructions
		virtual void clear() = 0;
		// Start program execution
		virtual void start() = 0;
		// Stop program execution
		virtual void stop() = 0;
		// Puase program execution
		virtual void pause() = 0;
		// Step the program by dt
		virtual void step(double dt) = 0;
		// Get current program status
		virtual ProgramStatus status() const = 0;
	};
} // namespace interpreter