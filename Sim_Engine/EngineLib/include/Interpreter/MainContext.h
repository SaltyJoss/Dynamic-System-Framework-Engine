#pragma once

#include "Interpreter/CommandContextMotion.h"
#include "Interpreter/UIContext.h"

namespace gui { class ENGINE_API simManager; }

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

        // Optional: convenience wiring if motion needs default object each tick
        void setDefaultObject(scene::Object* obj) {
            const scene::ObjectID id = obj ? obj->id : scene::INVALID_OBJECT_ID;
            _motion.setDefaultObjectID(id);
        }

    private:
        commands::CommandContextMotion _motion;
        commands::UIContext _ui;
	};
} // namespace commands