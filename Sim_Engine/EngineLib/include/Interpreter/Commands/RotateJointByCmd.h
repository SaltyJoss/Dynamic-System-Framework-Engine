#pragma once
// File:    RotateJointByCmd.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

namespace commands {
	class ENGINE_API RotateJointByCmd final : public Command {
	public:
		// Constructor
		RotateJointByCmd(std::string link, double omegaDeg, double deltaDeg);

		std::string_view getName() const { return "rotateJointBy"; }
		void setContext(CommandContextMotion& cntx) override { _cntxMtn = &cntx; }
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;
		program_data::CmdResult update(CommandContextMotion& cntx, double dt) override;

		std::string _link;
		double _deltaDeg = 0.0;
		double _omegaDeg = 0.0;
		double _totalRotated = 0.0;

		bool _started = false;
		double _elapsed = 0.0;
		double _timeoutSec = 10.0;

		double _deltaRad = 0.0;
		double _maxOmegaRad = 0.0;

		double _targetRad = 0.0;
		double _thetaStartRad = 0.0;

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
	std::unique_ptr<ICommand> CreateRotateJointByCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands