#pragma once

#include "EngineCore.h"
#include "SimFwd.h"
#include "Interpreter/CommandContextMotion.h"
#include "Interpreter/UIContext.h"

namespace commands {
	// Main context combining motion and UI contexts
	class ENGINE_API MainContext {
	public:
        MainContext(gui::simManager* sim, scene::ObjectID objID)
            : _motion(sim, objID), _ui(sim, objID) {}

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