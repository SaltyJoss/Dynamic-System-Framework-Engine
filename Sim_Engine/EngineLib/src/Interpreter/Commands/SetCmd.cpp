#include "pch.h"
#include "Interpreter/Commands/SetCmd.h"
#include "Interpreter/IStoredProgram.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;

namespace commands {
	// --- SetCmd Constructor ---
	SetCmd::SetCmd(const std::string& id, const std::vector<std::string>& tokens)
		: _id(id), _tokens(tokens) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// Helper function to parse LoadTarget from string
	// Expected formats: "set(integrator,<method>)"
	static std::optional<IntegratorMethod> parseMethod(const std::string& tokens) {
		if (startsWith(tokens, "integrator,")) {
			std::string s = tokens.substr(11);
			auto t = utils::toLower(s);
			if (t == "euler")    return IntegratorMethod::Euler;
			if (t == "midpoint") return IntegratorMethod::Midpoint;
			if (t == "heun")     return IntegratorMethod::Heun;
			if (t == "ralston")  return IntegratorMethod::Ralston;
			if (t == "rk4")      return IntegratorMethod::RK4;
			return std::nullopt;
		}
		return std::nullopt;
	}

	// --- SetCmd Method Implementations ---
	void SetCmd::markFailed(const std::string& message) {
		setResult({ CmdState::Failed, {}, message });
		// Implementation to mark the command as failed
	}
	void SetCmd::markCompleted() {
		// Implementation to mark the command as completed
	}
	bool SetCmd::hasStarted() const {
		return getResult().state != CmdState::NotStarted;
	}
	void SetCmd::execute() {
		if (!getProgram()) {
			std::string errMsg = "set() command has no program context.";
			markFailed(errMsg);
			D_FAIL("%s", errMsg.c_str());
			return;
		}
		if (_id == "integrator") {
			if (_tokens.size() != 1) {
				std::string errMsg = "set(integrator, <method>) expects exactly 1 argument.";
				markFailed(errMsg);
				D_FAIL("%s", errMsg.c_str());
				return;
			}
			auto m = parseMethod(_tokens[0]);
			if (!m) {
				std::string errMsg = "Unknown integrator method: " + _tokens[0];
				markFailed(errMsg);
				D_FAIL("%s", errMsg.c_str());
				return;
			}
			getProgram()->setIntegratorMethod(*m);
			markCompleted();
			D_SUCCESS("set() command executed: Integrator method set.");
			return;
		}

		markFailed("Unknown SET target: " + _id);
	}

	// --- Free Function to Create SetCmd ---
	std::unique_ptr<ICommand> CreateSetCmd(const std::string& id, const std::vector<std::string>& tokens) {
		return std::make_unique<SetCmd>(id, tokens);
	}
} // namespace commands