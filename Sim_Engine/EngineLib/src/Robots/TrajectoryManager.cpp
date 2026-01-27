#include "pch.h"
#include "Robots/TrajectoryManager.h"
#include "Robots/RobotSystem.h"
#include "Control/IJointTrajectory.h"

#include <Control/TrapezoidTrajectory.h>
#include <Control/SinusoidalTrajectory.h>
#include <Control/MultisineTrajectory.h>

namespace control {
	// Clear trajectory for a specific robot link
	void TrajectoryManager::clear(const std::string& link) { _active.erase(link); }
	// Clears all active trajectories
	void TrajectoryManager::clearAll() { _active.clear(); }

	// Set a trajectory for a specific robot link
	void TrajectoryManager::set(const std::string& link, std::unique_ptr<control::IJointTrajectory> traj) {
		if (!traj) {
			_active.erase(link);
			return;
		}

		_active.insert_or_assign(link, std::move(traj));
	}

	// Apply active trajectories to the robot at time t
	void TrajectoryManager::apply(robots::RobotSystem& robot, double t) {
		for (auto it = _active.begin(); it != _active.end();) {
			const std::string& link = it->first;
			auto& traj = it->second;

			const auto ref = traj->eval(t);

			robot.trySetJointTargetRad(link, (float)ref.q);
			robot.trySetJointOmegaRefRad(link, (float)ref.qd);
			robot.trySetJointAlphaRefRad(link, (float)ref.qdd);

			if (traj->finished(t)) { it = _active.erase(it); } 
			else { ++it; }
		}
	}
} // namespace control