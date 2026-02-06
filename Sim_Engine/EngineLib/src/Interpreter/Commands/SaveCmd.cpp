#include "pch.h"
#include "Interpreter/Commands/SaveCmd.h"
#include "Robots/RobotSystem.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

using namespace utils;

namespace commands {
	// --- SaveCmd Mark Methods ---
	void SaveCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void SaveCmd::markCompleted() { setResult({ CmdState::Executed, {}, "save() ran successfully" }); }
	bool SaveCmd::hasStarted() const { return getResult().state != CmdState::NotStarted; }

	// --- Helper Functions ---
	static bool tryParseSaveIsIntegratorName(const std::string& str, bool& out) {
		std::string s = toLower(str);
		if (s == "true" || s == "t" || s == "yes" || s == "1") { out = true; return true; }
		if (s == "false" || s == "f" || s == "no" || s == "0") { out = false; return true; }
		return false;
	}

	// --- SaveCmd Implementation ---
	SaveCmd::SaveCmd(const std::string& id, const std::vector<std::string>& tokens) {
		std::string type = toLower(id);
		if (type == "simdata") { _target.type = eSaveType::SimData; }
		else if (type == "plots")   { _target.type = eSaveType::Plots; }
		else {
			D_FAIL("save: unknown type '%s'", id.c_str());
			markFailed("save: unknown type (use 'simdata' or 'plots').");
			return;
		}

		_result = { CmdState::NotStarted, {}, "" };

		if (tokens.size() >= 1) _target.filename = tokens[0];
		if (tokens.size() >= 2) _target.isIntegratorName = tokens[1];
	}

	// --- SaveCmd Method Implementations ---
	void SaveCmd::execute() {
		if (_uiCntx == nullptr) { markFailed("save: UI context is not set."); return; }
		
		auto* robot = _uiCntx->Robot();
		if (robot == nullptr) { markFailed("save: no robot loaded in context."); return; }
		std::string integratorName = robot->getIntegratorName();

		if (_target.filename.empty()) {
			markFailed("save: filename argument is required.");
			D_FAIL("save: filename argument is required.");
			return;
		}

		switch (_target.type) {
			case eSaveType::SimData:
				if (_target.isIntegratorName.empty()) {
					markFailed("save: isIntegratorName argument is required for simdata type.");
					D_FAIL("save: isIntegratorName argument is required for simdata type.");
					return;
				}

				_filename = _target.filename;

				if (!tryParseSaveIsIntegratorName(_target.isIntegratorName, _integratorName)) {
					markFailed("save: invalid value for isIntegratorName argument (use true/false type).");
					D_FAIL("save: invalid value for isIntegratorName argument (use true/false type).");
					return;
				}

				HDF5_SAVE_DATA(_filename, _integratorName);
				SIM_SUCCESS("Saved simulation data to '%s'", _filename.c_str());
				break;

			case eSaveType::Plots:
				_filename = _target.filename;
				SIM_ERROR("Save command for plots currently does not exist");
				break;
			default:
				{
					markFailed("save: unhandled save type.");
					D_FAIL("save: unhandled save type.");
					return;
				}
		}

		markCompleted();
		D_SUCCESS("save() command executed successfully.");
	}

	std::unique_ptr<ICommand> CreateSaveCmd(const std::string& id, const std::vector<std::string>& args) {
		if (id.size() <= 0 || args.empty()) { D_FAIL("save() requires at least two arguments:  <type>, <filename>"); return nullptr; }
		return std::make_unique<SaveCmd>(id, args);
	}
} // namespace commands