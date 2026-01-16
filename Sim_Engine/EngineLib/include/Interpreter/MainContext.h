#pragma once

#include "Interpreter/CommandContextMotion.h"
#include "Interpreter/UIContext.h"

namespace gui { class simManager; }
namespace scene { class Object; }

namespace commands {
	// Main context combining motion and UI contexts
	class ENGINE_API MainContext {
	public:
        MainContext(gui::simManager* sim, scene::Object* defaultObj)
            : _motion(sim, defaultObj), _ui(sim, defaultObj) {}

        // Accessors
        commands::CommandContextMotion& motion() { return _motion; }
        const commands::CommandContextMotion& motion() const { return _motion; }

        commands::UIContext& ui() { return _ui; }
        const commands::UIContext& ui() const { return _ui; }

        // Optional: convenience wiring if motion needs default object each tick
        void setDefaultObject(scene::Object* obj) { _motion.setDefaultObject(obj); }

    private:
        commands::CommandContextMotion _motion;
        commands::UIContext _ui;
	};
} // namespace commands