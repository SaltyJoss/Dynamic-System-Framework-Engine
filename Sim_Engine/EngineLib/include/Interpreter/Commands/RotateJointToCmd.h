#pragma once
// File:    RotateJointToCmd.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

namespace commands {
	// Class representing the ROTATE command
	class ENGINE_API RotateJointToCmd final : public Command {
	public:
		// Constructor
		RotateJointToCmd(std::string link, double maxOmegaDeg, double angleDeg);

		std::string_view getName() const { return "rotateJointTo"; }
		void setContext(CommandContextMotion& cntx) override { _cntxMtn = &cntx; }
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;
		program_data::CmdResult update(CommandContextMotion& cntx, double dt) override;
		std::string _link;
		double _angleDeg; // angle relative to the start position
		double _maxOmegaDeg;
		
		bool _started = false;
		double _elapsed = 0.0;
		double _timeoutSec = 10.0;

		double _targetRad = 0.0;
		double _maxOmegaRad = 0.0;

		double _settleT = 0.0;
		double _noProgressT = 0.0;
		double _bestAbsErr = 0.0;

		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateRotateJointToCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands