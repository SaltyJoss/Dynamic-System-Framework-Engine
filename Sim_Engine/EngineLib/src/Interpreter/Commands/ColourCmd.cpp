#include "pch.h"
#include "Interpreter/Commands/ColourCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	ColourCmd::ColourCmd(const std::string& id, const std::vector<std::string>& tokens)
		: _id(id), _tokens(tokens) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// --- ColourCmd Method Implementations ---
	void ColourCmd::markFailed(const std::string& message) {
		setResult({ CmdState::Failed, {}, message });
		// Implementation to mark the command as failed
	}
	void ColourCmd::markCompleted() {
		// Implementation to mark the command as completed
	}
	bool ColourCmd::hasStarted() const {
		return getResult().state != CmdState::NotStarted;
	}
	void ColourCmd::execute() {
		if (_cntxUI == nullptr) {
			std::string errMsg = "UI context is not set for colour() command";
			markFailed(errMsg);
			D_FAIL(errMsg.c_str());
			return;
		}
	}

	// Convert mathlib::Vec3 to glm::vec3
	inline glm::vec3 toGlm(const mathlib::Vec3& v) {
		return glm::vec3(v.x(), v.y(), v.z());
	}

	// Get colour from parameter
	void ColourCmd::setColour(const Colour& colour, const mathlib::Vec3& rgb) {
		_currentColRGB = rgb;
		auto result = _uiCntx->setColour(toGlm(_currentColRGB));
		markCompleted();
	}
	// Set colour using preset
	void ColourCmd::setColour(const Colour& colour, BlockColour preset) {
		Vec3 rgb;
		switch (preset) {
		case BlockColour::Red:		rgb = { 1.0f, 0.0f, 0.0f };		break;
		case BlockColour::Green:	rgb = { 0.0f, 1.0f, 0.0f };		break;
		case BlockColour::Blue:		rgb = { 0.0f, 0.0f, 1.0f };		break;
		case BlockColour::Yellow:	rgb = { 1.0f, 1.0f, 0.0f };		break;
		case BlockColour::Cyan:		rgb = { 0.0f, 1.0f, 1.0f };		break;
		case BlockColour::Magenta:	rgb = { 1.0f, 0.0f, 1.0f };		break;
		case BlockColour::White:	rgb = { 1.0f, 1.0f, 1.0f };		break;
		case BlockColour::Grey:		rgb = { 0.5f, 0.5f, 0.5f };		break;
		case BlockColour::DarkGrey: rgb = { 0.25f, 0.25f, 0.25f };	break;
		case BlockColour::Black:	rgb = { 0.0f, 0.0f, 0.0f };		break;
		default:
			markFailed("Unknown colour preset.");
			return;
		}
		setColour(colour, rgb);
	}
	void ColourCmd::setColour(const Colour& colour, const std::string& hex) {
		Vec3 rgb = utils::hexToRGB(hex);
		setColour(colour, rgb);
	}

	std::unique_ptr<ICommand> CreateColourCmd(const std::string& id, const std::vector<std::string>& tokens) {
		return std::make_unique<ColourCmd>(id, tokens);
	}
} // namespace commands