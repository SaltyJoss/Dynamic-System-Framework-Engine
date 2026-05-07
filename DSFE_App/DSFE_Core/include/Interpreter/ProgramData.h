// DSFE_Core ProgramData.h
#pragma once
#pragma warning(disable : 4251)

#include "EngineCore.h"

#include <MathLibAPI.h>
#include <core/Types.h>
#include <core/constants.h>

#include <string>
#include <vector>

namespace program_data {
	// Struct representing source location
	struct DSFE_API SrcLocation {
		std::string filename; // Name of the source file
		int line = 0;             // Line number in the source file
		int column = 0;           // Column number in the source file
	};

	// Struct representing a single instruction
	struct DSFE_API Command {
		std::string rawLine;				// The original line of code
		std::string cmdName;				// The command name
		std::string identifier;				// The command identifier
		std::vector<std::string> tokens;	// The command arguments/tokens
		int lineNumber = 0;					// Line number in the source code

		// For parallel blocks
		bool isParallelBlock = false;
		double timeoutSec = 0.0;
		std::vector<Command> inner;
	};

	// Struct representing program data
	struct DSFE_API ProgramData {
		std::vector<Command> cmd; // Vector storing the instructions
	};

	// Numerical integrator methods
	enum class IntegratorMethod {
		Euler,
		Midpoint,
		Heun,
		Ralston,
		RK4,
		RK45,
		ImplicitEuler,
		ImplicitMidpoint,
		GLRK2,
		GLRK3
	};

	// Enum for preset colours
	enum class BlockColour {
		Red,
		Green,
		Blue,
		Yellow,
		Cyan,
		Magenta,
		White,
		Grey,
		DarkGrey,
		Black,
		Custom
	};

	// Struct for colour mapping
	struct DSFE_API Colour {
		BlockColour col = BlockColour::Red;
		mathlib::Vec3 rgb = { 1.0f, 0.0f, 0.0f };
	};

	// Enum representing the state of the program
	enum ProgramState {
		Empty,
		Running,
		Paused,
		Stopped,
		Completed,
		Faulted
	};

	// Struct for program status
	struct DSFE_API ProgramStatus{
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
	struct DSFE_API CmdSignalData {
		CmdSignalType signal = CmdSignal_None;
		size_t jumpTarget = 0; // for jump signals
	};

	// Command result struct
	struct DSFE_API CmdResult {
		CmdState state = CmdState::NotStarted;
		CmdSignalData signalData;
		std::string message;
	};
} // namespace interpreter