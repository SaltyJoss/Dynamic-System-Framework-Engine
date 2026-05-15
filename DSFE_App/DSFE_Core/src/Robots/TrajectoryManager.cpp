#include "pch.h"
// File:   TrajectoryManager.cpp
// GitHub: SaltyJoss
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

	// Evaluate the trajectory for a specific robot link at time t, returning the desired state in out
	bool TrajectoryManager::tryEval(const std::string& link, double t, control::TrajState& out) const {
		auto it = _active.find(link);
		if (it == _active.end()) { return false; }
		const auto r = it->second->eval(t);
		out.q = r.q;
		out.qd = r.qd;
		out.qdd = r.qdd;
		return true;
	}

	// Check if a trajectory is active for a specific robot link
	bool TrajectoryManager::hasActive(const std::string& link) const {
		auto it = _active.find(link);
		return (it != _active.end() && it->second);
	}


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

			// Evaluate trajectory at time t
			const auto ref = traj->eval(t);

			// Apply trajectory reference to robot joint
			const bool ok1 = robot.trySetJointTargetRad(link, (float)ref.q);
			if (!ok1) { D_ERROR("Bad link key '%s' (no joint.child match)", link.c_str()); }

			const bool ok2 = robot.trySetJointOmegaRefRad(link, (float)ref.qd);
			if (!ok2) { D_ERROR("Bad link key '%s' (no joint.child match)", link.c_str()); }

			const bool ok3 = robot.trySetJointAlphaRefRad(link, (float)ref.qdd);
			if (!ok3) { D_ERROR("Bad link key '%s' (no joint.child match)", link.c_str()); }

			// Remove finished trajectories
			if (traj->finished(t)) {
				D_WARN("Trajectory finished immediately: link='%s' t=%.6f", link.c_str(), t);
				it = _active.erase(it);
			}
			else {
				++it;
			}

		}
	}
} // namespace control