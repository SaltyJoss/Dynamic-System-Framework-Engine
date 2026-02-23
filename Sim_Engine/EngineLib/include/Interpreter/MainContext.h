#pragma once
// File:    MainContext.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include "SimFwd.h"
#include "Interpreter/CommandContextMotion.h"
#include "Interpreter/UIContext.h"

namespace commands {
	// Main context combining motion and UI contexts
	class ENGINE_API MainContext {
	public:
        MainContext(core::ISimulationCore* core)
            : _motion(core), _ui(core) {}

        // Accessors
        commands::CommandContextMotion& motion() { return _motion; }
        const commands::CommandContextMotion& motion() const { return _motion; }

        commands::UIContext& ui() { return _ui; }
        const commands::UIContext& ui() const { return _ui; }

    private:
        commands::CommandContextMotion _motion;
        commands::UIContext _ui;
	};
} // namespace commands