#include "pch.h"
#include "Interpreter/Commands/LoadCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;

namespace commands {
	// constructor
	LoadCmd::LoadCmd(const std::string& id, const std::vector<std::string>& tokens) {
		if (id == "obj") { _target.type = LoadTargetType::Object; }
		else if (id == "robot") { _target.type = LoadTargetType::Robot; }
		else if (id == "tex") { _target.type = LoadTargetType::Texture; }
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

	// --- LoadCmd Method Implementations ---
	void LoadCmd::markFailed(const std::string& message) {
		setResult({ CmdState::Failed, {}, message });
		// Implementation to mark the command as failed
	}

	void LoadCmd::markCompleted() {
		// Implementation to mark the command as completed
		setResult({ CmdState::Executed, {}, "load() ran successfully" });
	}

	bool LoadCmd::hasStarted() const {
		return getResult().state != CmdState::NotStarted;
	}

	void LoadCmd::execute() {
		if (_cntxUI == nullptr) {
			std::string errMsg = "UI context is not set for load() command";
			markFailed(errMsg);
			D_FAIL(errMsg.c_str());
			return;
		}

		switch (_target.type) {
		case LoadTargetType::Object:	_cntxUI->loadObject(_target.path); break;
		case LoadTargetType::Robot:		_cntxUI->loadRobot(_target.path); break;
		case LoadTargetType::Texture:	_cntxUI->loadTexture(_target.path); break;
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

	// --- Free Function to Create LoadCmd ---
	std::unique_ptr<ICommand> CreateLoadCmd(const std::string& id, const std::vector<std::string>& tokens) {
		if (tokens.empty()) return nullptr;
		return std::make_unique<LoadCmd>(id, tokens);
	}
} // namespace commands