#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <cstddef>
#include <string>
#include <unordered_map>
#include "Scene/SimulationManager.h"

#include "Platform/Logger.h"

namespace commands {
	class ENGINE_API CommandContextMotion;
	class ENGINE_API UIContext;
}

namespace gui {
	class ENGINE_API SimContext {
	public:
		SimContext(gui::simManager* sims);
		~SimContext();
		gui::simManager* getSimManager() const { return _sim; }

	private:
		gui::simManager* _sim;
	};
}
