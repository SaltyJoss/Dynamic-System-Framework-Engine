#pragma once

#include "EngineCore.h"
#include "ProgramData.h"
#include <string>
#include <vector>

namespace commands {
	class ENGINE_API ICommand;
}

namespace scene {
	class Object;
}

namespace interpreter {
	// Enum representing the state of the program
	enum ProgramState {
		Empty,
		Loaded,
		Running,
		Paused,
		Stopped,
		Completed,
		Faulted
	};

	// Struct for program status
	struct ProgramStatus {
		ProgramState state = ProgramState::Empty;
		size_t pc = 0;
	};

	// Command states
	enum CmdState {
		NotStarted,
		Executing,
		Executed,
		Failed
	};

	// Command signals (not used yet, but will be)
	enum CmdSignalType {
		CmdSignal_None,
		CmdSignal_Start,
		CmdSignal_Stop,
		CmdSignal_Pause,
		CmdSignal_Resume,
		CmdSignal_Jump
	};

	// Command signal data struct
	struct CmdSignalData {
		CmdSignalType signal = CmdSignal_None;
		size_t jumpTarget = 0; // for jump signals
	};
	// Command result struct
	struct CmdResult {
		CmdState state = CmdState::NotStarted;
		CmdSignalData signalData;
		std::string message;
	};

	// IStoredProgram interface
	class ENGINE_API IStoredProgram { 
	public:
		// Virtual destructor
		virtual ~IStoredProgram() = default;

		// Add a command to the program
		virtual void add(commands::ICommand* cmd) = 0;

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

		// State checkers
		virtual bool isRunning() const = 0;
		virtual bool isPaused() const = 0;
		virtual bool isStopped() const = 0;

		// Update the command state
		virtual CmdResult updateState() = 0;

		// Get Current line number
		virtual int getCurrentLineNumber() const = 0;
		// Set Current line number
		virtual void setCurrentLineNumber(int lineNumber) = 0;

		// Set default object
		virtual void setDefaultObject(scene::Object* obj) = 0;
		// Get default object
		virtual scene::Object* defaultObject() const = 0;
	};
} // namespace interpreter