#include "pch.h"
// File:   SelectCmd.cpp
// GitHub: SaltyJoss
#include "Interpreter/Commands/SelectCmd.h"
#include "Scene/SimulationManager.h"
#include "Scene/ObjectID.h"
#include "Scene/Object.h"

#include "EngineLib/LogMacros.h"

namespace commands {
	// --- SelectCmd Mark Methods ---
	void SelectCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void SelectCmd::markCompleted() { setResult({ CmdState::Executed, {}, "select() ran successfully" }); }
	bool SelectCmd::hasStarted() const { return getResult().state != CmdState::NotStarted; }

	// Constructor
	SelectCmd::SelectCmd(scene::ObjectID obj) : _objID(obj) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// Execute the command
	void SelectCmd::execute() {
		if (!getProgram()) {
			std::string errMsg = "select() command has no program context.";
			markFailed(errMsg);
			D_FAIL("%s", errMsg.c_str());
			return;
		}
		if (!_uiCntx) {
			std::string errMsg = "select() command has no UI context.";
			markFailed(errMsg);
			D_FAIL("%s", errMsg.c_str());
			return;
		}
		scene::Object* obj = _uiCntx->resolveObject(_objID);
		if (!obj) {
			std::string errMsg = "select() target object not found.";
			markFailed(errMsg);
			D_FAIL("%s", errMsg.c_str());
			return;
		}
		_uiCntx->resolveObject(_objID);
		markCompleted();
		D_SUCCESS("select() command executed: Object %u selected.", static_cast<uint32_t>(_objID));
	}

	// Not needed right now, will revist soon
	void SelectCmd::listObjID() {
		D_RUNTIME("Available Object IDs: ");
		//scene::Object* obj = _uiCntx->resolveObject(_objID);
		//for (const auto& [name, id] : obj->) {
		//	D_RUNTIME(" - Name: '%s', ID: %u", name.c_str(), static_cast<uint32_t>(id));
		//}
	}

	// Factory function to create a SelectCmd from arguments
	std::unique_ptr<ICommand> CreateSelectCmd(const std::string& id, const std::vector<std::string>& args) {
		if (id.empty()) { D_FAIL("select(<objID>) expects exactly 1 argument."); }
		scene::ObjectID objID = static_cast<scene::ObjectID>(std::stoul(id));

		if (objID == scene::ObjectID::INVALID_OBJECT_ID) { D_FAIL("select(<objID>) received invalid object ID."); }

		return std::make_unique<SelectCmd>(objID);
	}
}