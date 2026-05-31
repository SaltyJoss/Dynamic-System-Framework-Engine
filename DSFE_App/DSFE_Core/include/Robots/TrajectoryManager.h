#pragma once
// File:   TrajectoryManager.h
// GitHub: SaltyJoss
// -----
// Initially templated off a tutorial:
// GitHub: jayanam/jgl_demos/JGL_MeshLoader
#include "EngineCore.h"
#include <core/MathLib.h>
#include <Control/IJointTrajectory.h>

namespace robots { class DSFE_API RobotSystem; }

namespace control {
	// Trajectory Manager Class
	class DSFE_API TrajectoryManager {
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

		bool empty() const { return _active.empty(); }

		//
		bool tryEval(const std::string& link, double t, control::TrajState<double>& out) const;
		// Check if a trajectory is active for a specific robot link
		bool hasActive(const std::string& link) const;

		// Set a trajectory for a specific robot link
		void set(const std::string& link, std::unique_ptr<control::IJointTrajectory> traj);
		// Apply active trajectories to the robot at time t
		void apply(robots::RobotSystem& robot, double t);

		std::size_t activeCount() const { return _active.size(); }

	private:
#pragma warning(push)
#pragma warning(disable: 4251)
		std::unordered_map<std::string, std::unique_ptr<control::IJointTrajectory>> _active;
#pragma warning(pop)
	};
} // namespace control