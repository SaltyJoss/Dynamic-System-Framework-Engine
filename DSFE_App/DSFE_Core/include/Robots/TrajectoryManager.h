// DSFE_Core TrajectoryManager.h
#pragma once
#include "EngineCore.h"
#include <core/MathLib.h>
#include <control/IJointTrajectory.h>

namespace robots { class DSFE_API RobotSystem; }

namespace control {
	class DSFE_API TrajectoryManager {
	public:
		TrajectoryManager() = default;
		~TrajectoryManager() = default;
		// non-copyable
		TrajectoryManager(const TrajectoryManager&) = delete;
		TrajectoryManager& operator=(const TrajectoryManager&) = delete;
		// movable is fine
		TrajectoryManager(TrajectoryManager&&) noexcept = default;
		TrajectoryManager& operator=(TrajectoryManager&&) noexcept = default;

		void clear(const std::string& link);
		void clearAll();

		bool empty() const { return _active.empty(); }
		bool tryEval(const std::string& link, double t, control::TrajState<double>& out) const;
		bool hasActive(const std::string& link) const;

		void set(const std::string& link, std::unique_ptr<control::IJointTrajectory> traj);
		void apply(robots::RobotSystem& robot, double t);

		std::size_t activeCount() const { return _active.size(); }

	private:
		std::unordered_map<std::string, std::unique_ptr<control::IJointTrajectory>> _active;
	};
} // namespace control