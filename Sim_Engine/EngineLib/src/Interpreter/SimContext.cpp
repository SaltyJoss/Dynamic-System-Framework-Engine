#include "pch.h"
#include "Interpreter/SimContext.h"

#include "EngineLib/LogMacros.h"

namespace gui {
	SimContext::SimContext(gui::simManager* sims) : _sim(sims) {
		LOG_INFO("SimContext created.");
		D_INFO_ONCE("SimContext created.");
	}
	SimContext::~SimContext() {
		LOG_INFO("SimContext destroyed.");
		D_INFO_ONCE("SimContext destroyed.");
	}
}