#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/ICommand.h"

using namespace mathlib;

namespace commands {
	// Enum for specifying the type of rotation target
	enum RotateTargetType {
		AxisMask,
		JointIndex
	};
	// Enum for specifying axes
	enum Axis {
		x,
		y,
		z
	};

	// Struct for axis mask
	struct AxisMasks {
		bool x = false;
		bool y = false;
		bool z = false;
	};
	// Struct for rotation target
	struct RotateTarget {
		RotateTargetType type = RotateTargetType::AxisMask;
		AxisMasks axisMask;
		size_t jointIndex = 0;
	};

	// Class representing the ROTATE command
	class ENGINE_API RotateCmd : public ICommand {
		// Constructor
		RotateCmd(const RotateTarget& target, double angleDeg, double vel);

		// Rotation target
		std::string_view name() const override;
		// Start the command
		void start(CommandContext& cntx) override;
		// Update the command
		CmdResult update(CommandContext& cntx, double dt) override;
		// Stop the command
		void stop(CommandContext& cntx) override;
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateRotateCmd(const std::vector<std::string>& args);
} // namespace commands