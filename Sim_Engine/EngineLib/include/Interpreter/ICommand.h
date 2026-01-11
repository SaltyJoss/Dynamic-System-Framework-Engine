#pragma once

#include "EngineCore.h"
#include "CommandContextMotion.h"
#include <memory>
#include <string>

namespace commands {
	// Command states
	enum CmdState {
		NotStarted,
		Running,
		Completed,
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

	// ICommand interface
	class ENGINE_API ICommand {
		// Get the command name
		virtual std::string_view name() const = 0;
		// Start the command
		virtual void start(CommandContextMotion& cntx) = 0;
		// Update the command
		virtual CmdResult update(CommandContextMotion& cntx, double dt) = 0;
		// Stop the command
		virtual void stop(CommandContextMotion& cntx) = 0;
	};
	
} // namespace interpreter