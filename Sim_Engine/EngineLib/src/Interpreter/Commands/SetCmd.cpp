#include "pch.h"
#include "Interpreter/Commands/SetCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;

namespace commands {
	// Helper function to parse LoadTarget from string
	// Expected formats: "INTEGRATOR <method>"
	static std::optional<SetTarget> parseType(const std::string& tokens) {
		if (startsWith(tokens, "integrator,")) {
			std::string methodStr = tokens.substr(11);
			IntegratorMethod method;

			// Determine the integrator method
			switch (methodStr[0]) {
				case 'M': case 'm':
					method = IntegratorMethod::Midpoint;
					break;
				case 'H': case 'h':
					method = IntegratorMethod::Heun;
					break;
				case 'R': case 'r':
					if (methodStr.size() > 1 && (methodStr[1] == 'K' || methodStr[1] == 'k')) {
						method = IntegratorMethod::RK4;
						break;
					}
					method = IntegratorMethod::Ralston;
					break;
				case 'E': case 'e':
				default:
					method = IntegratorMethod::Euler;
					break;
			}

			return SetTarget{SetTargetType::IntegratorMethod, method};
		}
		if (startsWith(tokens, "func")) {
			// Future implementation for function definition
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
		// Implementation of the SET command execution
		auto targetOpt = parseType(_args);
		if (!targetOpt.has_value()) {
			std::string errMsg = "Invalid set( argument: " + _args + " (expected INTEGRATOR <method>)";
			markFailed(errMsg);
			D_FAIL("%s", errMsg.c_str());
			return;
		}
		_target = *targetOpt;

	}

	// --- Free Function to Create SetCmd ---
	std::unique_ptr<ICommand> CreateSetCmd(const std::string& id, const std::string& arg) {

		if (arg.empty()) {
			D_FAIL("SET command requires an argument.");
			return nullptr;
		}

		return std::make_unique<SetCmd>(id, arg);
	}
} // namespace commands