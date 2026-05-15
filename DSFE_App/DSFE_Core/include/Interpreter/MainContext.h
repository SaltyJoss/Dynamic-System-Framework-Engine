#pragma once
// File:    MainContext.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include "SimFwd.h"
#include "Interpreter/CommandContext.h"

namespace commands {
	// Main context to combine multiple command contexts for different subsystems (currently only command context)
	class DSFE_API MainContext {
	public:
        MainContext(core::ISimulationCore* core)
            : _motion(core) {}

        // Accessors
        commands::CommandContext& motion() { return _motion; }
        const commands::CommandContext& motion() const { return _motion; }

    private:
        commands::CommandContext _motion;
	};
} // namespace commands