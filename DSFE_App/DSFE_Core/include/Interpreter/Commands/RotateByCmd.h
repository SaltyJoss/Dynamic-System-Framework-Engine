#pragma once
// File:    RotateByCmd.h
// GitHub:  SaltyJoss
#pragma warning(disable : 4251)
#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

namespace commands {

	// Class representing the ROTATE command
	class DSFE_API RotateByCmd final : public Command {
	public:
		// Constructor
		RotateByCmd(scene::ObjectID obj, utils::AxisMask axis, double maxOmegaDeg, double deltaDeg);

		std::string_view getName() const { return "rotateBy"; }
		void setContext(CommandContextMotion& cntx) override { _cntxMtn = &cntx; }
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;
		program_data::CmdResult update(CommandContextMotion& cntx, double dt) override;

		scene::ObjectID _objID{};
		utils::AxisMask _axes{};
		double _deltaDeg;
		double _omegaDeg;
		double _totalRotated = 0.0;
		bool _started = false;

		program_data::CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateRotateByCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands