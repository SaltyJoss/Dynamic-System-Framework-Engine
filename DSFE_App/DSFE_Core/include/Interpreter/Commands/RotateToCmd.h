#pragma once
// File:    RotateToCmd.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

namespace commands {

	// Class representing the ROTATE command
	class DSFE_API RotateToCmd final : public Command {
	public:
		// Constructor
		RotateToCmd(scene::ObjectID obj, utils::AxisMask axes, double maxOmegaDeg, double angleDeg);

		std::string_view getName() const { return "rotateBy"; }
		void setContext(CommandContextMotion& cntx) override { _cntxMtn = &cntx; }
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;
		program_data::CmdResult update(CommandContextMotion& cntx, double dt) override;

		scene::ObjectID _objID{};
		utils::AxisMask _axes;
		double _angleDeg = 0.0;
		double _maxOmegaDeg = 0.0;
		bool _started = false;

		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateRotateToCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands