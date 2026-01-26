#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <Control/IJointTrajectory.h>

namespace robots { class ENGINE_API RobotSystem; }

namespace control {
	// Trajectory Manager Class
	class ENGINE_API TrajectoryManager {
	public:
		TrajectoryManager() = default;
		~TrajectoryManager() = default;

		// non-copyable (because unique_ptr)
		TrajectoryManager(const TrajectoryManager&) = delete;
		TrajectoryManager& operator=(const TrajectoryManager&) = delete;

		// movable is fine
		TrajectoryManager(TrajectoryManager&&) noexcept = default;
		TrajectoryManager& operator=(TrajectoryManager&&) noexcept = default;

		// Clear trajectory for a specific robot link
		void clear(const std::string& link);
		// Clears all active trajectories
		void clearAll();

		// Set a trajectory for a specific robot link
		void set(const std::string& link, std::unique_ptr<control::IJointTrajectory> traj);
		// Apply active trajectories to the robot at time t
		void apply(robots::RobotSystem& robot, double t);

		std::size_t activeCount() const { return _active.size(); }

	private:
		std::unordered_map<std::string, std::unique_ptr<control::IJointTrajectory>> _active;
	};
} // namespace control