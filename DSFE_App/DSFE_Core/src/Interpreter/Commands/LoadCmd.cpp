// DSFE_Core LoadCmd.cpp
#include "pch.h"

#include "Interpreter/Commands/LoadCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;

namespace commands {
	// --- LoadCmd Method Implementations ---
	void LoadCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void LoadCmd::markCompleted() { setResult({ CmdState::Executed, {}, "load() ran successfully" }); }
	bool LoadCmd::hasStarted() const { return getResult().state != CmdState::NotStarted; }

	// constructor
	LoadCmd::LoadCmd(const std::string& id, const std::vector<std::string>& tokens) {
		if (id == "robot") { _target.type = LoadTargetType::MultiBody; }
		else {
			std::string errMsg = "Invalid load(<target>,...) identifier -> " + id;
			markFailed(errMsg);
			D_FAIL(errMsg.c_str());
			return;
		}

		if (tokens.empty()) {
			std::string errMsg = "load() command requires a path argument.";
			markFailed(errMsg); 
			D_FAIL(errMsg.c_str());
			return;
		}

		if (!tokens.empty()) { _path = tokens[0]; _target.path = tokens[0]; }
	}

	// Execute the command
	void LoadCmd::execute() {
		if (_cntx == nullptr) {
			std::string errMsg = "Command context is not set for load() command";
			markFailed(errMsg);
			D_FAIL(errMsg.c_str());
			return;
		}

		switch (_target.type) {
		case LoadTargetType::SingleBody: _cntx->loadSingleBody(_target.path); break;
		case LoadTargetType::MultiBody:	 _cntx->loadMultibody(_target.path); break;
		default:
			{
				std::string errMsg = "Invalid load target type.";
				markFailed(errMsg);
				D_FAIL(errMsg.c_str());
				return;
			}
		}

		markCompleted();
		D_SUCCESS("load() command executed successfully.");
	}

	// Factory function to create a LoadCmd from arguments
	std::unique_ptr<ICommand> CreateLoadCmd(const std::string& id, const std::vector<std::string>& tokens) {
		if (tokens.empty()) return nullptr;
		return std::make_unique<LoadCmd>(id, tokens);
	}
} // namespace commands