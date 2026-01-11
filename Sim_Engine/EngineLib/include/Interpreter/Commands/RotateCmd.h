#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

using namespace mathlib;

namespace commands {
	// Enum for rotation target type
	enum class RotateTargetType {
		AxisMask,
		linkName
	};

	// Struct for rotation target
	struct RotateTarget {
		RotateTargetType type = RotateTargetType::AxisMask;
		AxisMask axisMask;
		std::string linkName = "";
	};

	// Class representing the ROTATE command
	class ENGINE_API RotateCmd final : public Command {
	public:
		// Constructor
		RotateCmd(RotateTarget target, double omega, double startDeg = 0.0, double endDeg = 0.0);

		// Rotation target
		std::string_view name() const override;
		// Start the command
		void start(CommandContextMotion& cntx) override;
		// Update the command
		CmdResult update(CommandContextMotion& cntx, double dt) override;
		// Stop the command
		void stop(CommandContextMotion& cntx) override;
	private:
		RotateTarget _target{};
		double _omega = 0.0;      // Angular velocity (deg/s)
		double _angleDeg = 0.0;   // Total rotation angle (deg)
		bool  _started = false;    // Flag to indicate if rotation has started

		double _currentAngle = 0.0; // Current angle for rotation commands
		double _totalRotated = 0.0; // Total rotated angle
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateRotateCmd(const std::vector<std::string>& args);
} // namespace commands