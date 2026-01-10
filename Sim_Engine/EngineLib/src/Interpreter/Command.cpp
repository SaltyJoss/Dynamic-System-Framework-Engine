#include "pch.h"
#include "Interpreter/Command.h"

namespace commands {
	std::string_view Command::name() const {
		return "GenericCommand"; // Placeholder name
	}

	void Command::start(CommandContext& cntx) {
		// Default implementation does nothing
	}

	CmdResult Command::update(CommandContext& cntx, double dt) {
		// Default implementation does nothing and returns NotStarted
		return CmdResult{ CmdState::NotStarted, CmdSignalData{}, "" }; // Placeholder
	}

	void Command::stop(CommandContext& cntx) {
		// Default implementation does nothing
	}

	void Command::markFailed(const std::string& message) {
		// Implementation to mark the command as failed
	}

	void Command::markCompleted() {
		// Implementation to mark the command as completed
	}

	bool Command::hasStarted() const {
		if (CmdState::Running) {
			return true;
		}
		return false;
	}
} // namespace commands