// DSFE_Core IntegratorState.h
#pragma once

#include <string>

namespace integration {
	// Current simulation backend integration method (e.g., standard numerical integration[explicit, implicit] vs. auto-differentiation for gradients)
	enum class eIntegrationBackend {
		Standard,
		AutoDiff
	};

	struct IntegratorState {
		std::string name;
		eIntegrationBackend backend = eIntegrationBackend::Standard;
		bool adaptive = false;
		bool implicit = false; // false means explicit, TODO adapt this correctly for semi- once integrated
		bool autoDiff = false;

		double last_dt_taken = 0.0;
		double last_dt_sug = 0.0;
	};
}