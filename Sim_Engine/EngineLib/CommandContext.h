#pragma once

#include "EngineCore.h"

namespace commands {
	// Struct for time information
	struct TimeInfo {
		double dt = 0.0;
		double elapsed = 0.0;
	};

	// Log level enum, replicates Platform/Logger.h LogLevel (simplified)
	enum LogLevel {
		Info,
		Warning,
		Error
	};

	// Class representing the command context
	class ENGINE_API CommandContext {
		// Current simulation time
		TimeInfo time() const;
		// Log a message
		void log(const std::string& message, LogLevel level = LogLevel::Info) const;

		// Get joint count
		size_t jointCount() const;
		// Get joint position
		double jointPos(size_t jointIndex) const;
		// Get joint velocity
		double jointVel(size_t jointIndex) const;

		// Set joint target position
		void setJointTargetPos(size_t jointIndex, double qTarget);
		// Set joint velocity
		void setJointVel(size_t jointIndex, double omega);

		// Check if a link exists
		bool hasLink(size_t linkIndex) const;
	};
} // namespace commands