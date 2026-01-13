#include "pch.h"
#include "Interpreter/Commands/LoadCmd.h"
#include "EngineLib/LogMacros.h"

namespace commands {
	// Helper function to check if a string starts with a prefix
	static bool startsWith(const std::string& str, const std::string& prefix) {
		return str.size() >= prefix.size() && str.substr(0, prefix.size()) == prefix;
	}

	// Helper function to parse LoadTarget from string
	// Expected formats: "OBJECT:path", "ROBOT:path", "TEXTURE:path"
	static std::optional<LoadTarget> parseLoadTarget(const std::string& arg) {
		if (startsWith(arg, "OBJECT:")) {
			std::string path = arg.substr(7);
			return LoadTarget{ LoadTargetType::Object, path };
		}
		if (startsWith(arg, "ROBOT:")) {
			std::string path = arg.substr(6);
			return LoadTarget{ LoadTargetType::Robot, path };
		}
		if (startsWith(arg, "TEXTURE:")) {
			std::string path = arg.substr(8);
			return LoadTarget{ LoadTargetType::Texture, path };
		}
		return std::nullopt;
	}

	// --- LoadCmd Method Implementations ---
	void LoadCmd::markFailed(const std::string& message) {
		setResult({ CmdState::Failed, {}, message });
		// Implementation to mark the command as failed
	}

	void LoadCmd::markCompleted() {
		// Implementation to mark the command as completed
	}

	bool LoadCmd::hasStarted() const {
		return getResult().state != CmdState::NotStarted;
	}

	void LoadCmd::execute() {
		if (_uiCntx == nullptr) {
			markFailed("UI context is not set.");
			return;
		}
		switch (_target.type) {
		case LoadTargetType::Object:
			_uiCntx->loadObject(_target.path);
			markCompleted();
			break;
		case LoadTargetType::Robot:
			_uiCntx->loadRobot(_target.path);
			markCompleted();
			break;
		case LoadTargetType::Texture:
			_uiCntx->loadTexture(_target.path);
			markCompleted();
			break;
		default:
			markFailed("Unknown load target type.");
			break;
		}
	}

	// --- Free Function to Create LoadCmd ---
	std::unique_ptr<ICommand> CreateLoadCmd(const std::string& id, const std::string& path) {
		auto targetOpt = parseLoadTarget(path);
		if (!targetOpt.has_value()) {
			D_FAIL("LOAD requires: OBJECT:<path>, ROBOT:<name>, or TEXTURE:<path>");
			return nullptr;
		}
		return std::make_unique<LoadCmd>(targetOpt.value(), targetOpt->path);
	}
} // namespace commands