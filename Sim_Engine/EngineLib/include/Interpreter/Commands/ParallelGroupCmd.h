#pragma once

#include "EngineCore.h"
#include "Interpreter/Command.h"
#include <memory>
#include <vector>

namespace commands {
	class ENGINE_API ParallelGroupCmd final : public Command {
	public:
		// Any: succeed if any command succeeds; All: succeed only if all commands succeed
		enum class Policy { Any, All };
		// Constructor
		ParallelGroupCmd(Policy policy, std::vector<std::unique_ptr<ICommand>> cmds, double timeout = 0.0);

		// Delete copy constructor and assignment operator
		ParallelGroupCmd(const ParallelGroupCmd&) = delete;
		ParallelGroupCmd& operator=(const ParallelGroupCmd&) = delete;
		// Default move constructor and assignment operator
		ParallelGroupCmd(ParallelGroupCmd&&) noexcept = default;
		ParallelGroupCmd& operator=(ParallelGroupCmd&&) noexcept = default;

		CmdResult update(CommandContextMotion& cntx, double dt) override;
		CmdResult currentResult() const override { return _result; }

	private:
		Policy _policy;
		std::vector<std::unique_ptr<ICommand>> _cmds;
		bool _started = false;
		double _elapsed = 0.0;
		double _timeoutSec = 0.0;

		CmdResult _result = { CmdState::NotStarted, {}, "" };
	};
}