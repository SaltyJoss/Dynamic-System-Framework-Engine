#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

namespace commands {

	// Class representing the ROTATE command
	class ENGINE_API SpinCmd final : public Command {
	public:
		// Constructor
		SpinCmd(scene::ObjectID obj, utils::AxisMask axes, double omegaDeg, double duration);

		std::string_view getName() const { return "rotateBy"; }
		void setContext(CommandContextMotion& cntx) override { _cntxMtn = &cntx; }
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;
		program_data::CmdResult update(CommandContextMotion& cntx, double dt) override;

		scene::ObjectID _obj;
		utils::AxisMask _axes;
		double _omegaDeg = 0.0;
		double _duration = 0.0;
		double _remainingTime = 0.0;
		bool _started = false;

		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateSpinCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands