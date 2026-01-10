#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/CommandContext.h"

using namespace mathlib;

namespace commands {
	// Enum for rotation target type
	enum class RotateTargetType {
		AxisMask,
		LinkIndex
	};

	// Struct for rotation target
	struct RotateTarget {
		RotateTargetType type = RotateTargetType::AxisMask;
		AxisMask axisMask;
		size_t linkIndex = 0;
	};

	// Class representing the ROTATE command
	class ENGINE_API RotateCmd final : public Command {
	public:
		// Constructor
		RotateCmd(RotateTarget target, double omega, double startDeg = 0.0, double endDeg = 0.0);

		// Rotation target
		std::string_view name() const override;
		// Start the command
		void start(CommandContext& cntx) override;
		// Update the command
		CmdResult update(CommandContext& cntx, double dt) override;
		// Stop the command
		void stop(CommandContext& cntx) override;
	private:
		RotateTarget _target_{};
		double omega_ = 0.0;      // Angular velocity (deg/s)
		double angleDeg_ = 0.0;   // Total rotation angle (deg)
		bool  started_ = false;    // Flag to indicate if rotation has started
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateRotateCmd(const std::vector<std::string>& args);
} // namespace commands