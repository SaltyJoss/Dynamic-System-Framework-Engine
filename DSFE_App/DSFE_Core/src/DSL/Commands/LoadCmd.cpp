/*
 * File: DSL/LoadCmd.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/Commands/LoadCmd.h"
#include "DSL/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;

namespace commands {
	// --- LoadCmd Method Implementations ---
	void LoadCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void LoadCmd::markCompleted() { setResult({ CmdState::Executed, {}, "load() ran successfully" }); }
	bool LoadCmd::hasStarted() const { return getResult().state != CmdState::NotStarted; }

	// constructor
	LoadCmd::LoadCmd(const std::string& id, const std::vector<std::string>& tokens) {
		if (id == "rigidbody" || id == "robot") {
			_target.type = LoadTargetType::RigidBody;
			std::string name = toLower(tokens[0]);
			if (tokens.size() == 1) {
				LOG_WARN("load() command called with single token, assuming .urdf in rigidbody_models/%s", name.c_str());
				_path = "rigidbody_models/" + name + "/" + name + ".urdf";
				_target.path = _path;
				return;
			}
			_path = "rigidbody_models/" + name + "/" + toLower(tokens[1]);
			_target.path = _path;
			return;
		}
		else {
			std::string errMsg = "Invalid load(<target>,...) identifier -> " + id;
			markFailed(errMsg);
			D_FAIL(errMsg.c_str());
			return;
		}
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
		case LoadTargetType::RigidBody:	 _cntx->loadRigidBody(_target.path); break;
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
		if (tokens.empty()) { 
			std::string errMsg = "load() command requires a path argument.";
			D_FAIL(errMsg.c_str());
			return nullptr;
		}
		std::string id_lower = toLower(id);
		return std::make_unique<LoadCmd>(id_lower, tokens);
	}
} // namespace commands