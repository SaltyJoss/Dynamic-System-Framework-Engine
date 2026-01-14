#include "pch.h"
#include "Interpreter/Commands/ColourCmd.h"
#include "EngineLib/LogMacros.h"

namespace commands {
	// Helper function to convert hex string to RGB vector (wanted to make my own, so I did)
	static bool hexToRGB(const std::string& hex, Vec3& rgbOut) {
		if (hex.size() != 7 || hex[0] != '#') {
			D_ERROR("Invalid hex colour format: %s", hex.c_str());
			return false; // Invalid format
		}
		try {
			int r = std::stoi(hex.substr(1, 2), nullptr, 16);
			int g = std::stoi(hex.substr(3, 2), nullptr, 16);
			int b = std::stoi(hex.substr(5, 2), nullptr, 16);
			rgbOut = Vec3{ r / 255.0f, g / 255.0f, b / 255.0f };
			return true;
		}
		catch (...) {
			D_ERROR("Failed to convert hex to RGB: %s", hex.c_str());
			return false;
		}
	}
	// --- ColourCmd Method Implementations ---
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
		if (_cntxUI == nullptr) {
			markFailed("UI context is not set.");
			return;
		}
	}
} // namespace commands