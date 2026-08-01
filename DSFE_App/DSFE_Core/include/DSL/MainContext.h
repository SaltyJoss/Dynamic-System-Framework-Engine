/*
 * File: DSL/MainContext.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include "DSL/SimFwd.h"
#include "DSL/CommandContext.h"

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