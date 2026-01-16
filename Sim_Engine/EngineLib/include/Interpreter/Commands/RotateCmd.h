#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

namespace commands {
	// Enum for rotation target type
	enum class RotateTargetType {
		AxisMask,
		linkName,
		ObjID
	};

	// Struct for rotation target
	struct RotateTarget {
		RotateTargetType type = RotateTargetType::AxisMask;
		utils::AxisMask axisMask;
		std::string linkName = "";
		std::string objID = "";
	};

	// Class representing the ROTATE command
	class ENGINE_API RotateCmd final : public Command {
	public:
		// Constructor
		RotateCmd(RotateTarget target, double omega, double startDeg = 0.0, double endDeg = 0.0);

		std::string_view getName() const { return "ROTATE"; }

		void setContext(CommandContextMotion& cntx) override { _cntxMtn = &cntx; }

		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }

		void execute() override;

		CmdResult update(CommandContextMotion& cntx, double dt) override;
	private:
		RotateTarget _target{};
		double _omega = 0.0;      // Angular velocity (deg/s)
		double _angleDeg = 0.0;   // Total rotation angle (deg)
		bool  _started = false;    // Flag to indicate if rotation has started

		double _currentAngle = 0.0; // Current angle for rotation commands
		double _totalRotated = 0.0; // Total rotated angle

		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		// Mark the command as failed with a message
		void markFailed(const std::string& message) override;
		// Mark the command as completed
		void markCompleted() override;
		// Check if the command has started
		bool hasStarted() const override;
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateRotateCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands