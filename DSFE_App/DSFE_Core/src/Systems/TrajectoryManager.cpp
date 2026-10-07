/*
 * File: Systems/TrajectoryManager.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"
#include "Systems/TrajectoryManager.h"
#include "Systems/RigidBodySystem.h"

#include <control/TrapezoidTrajectory.h>
#include <control/SinusoidalTrajectory.h>
#include <control/MultisineTrajectory.h>

namespace control {
	// Clear trajectory for a specific rigidBody link
	void TrajectoryManager::clear(const std::string& link) { _active.erase(link); }

	// Clears all active trajectories
	void TrajectoryManager::clearAll() { _active.clear(); }

	// Evaluate the trajectory for a specific rigidBody link at time t, returning the desired state in out
	bool TrajectoryManager::tryEval(const std::string& link, double t, control::TrajState<double>& out) const {
		auto it = _active.find(link);
		if (it == _active.end()) { return false; }
		const auto r = it->second->eval(t);
		out.q = r.q;
		out.qd = r.qd;
		out.qdd = r.qdd;
		return true;
	}

	// Check if a trajectory is active for a specific rigidBody link
	bool TrajectoryManager::hasActive(const std::string& link) const {
		auto it = _active.find(link);
		return (it != _active.end() && it->second);
	}


	// Set a trajectory for a specific rigidBody link
	void TrajectoryManager::set(const std::string& link, std::unique_ptr<control::IJointTrajectory> traj) {
		if (!traj) {
			_active.erase(link);
			return;
		}
		_active.insert_or_assign(link, std::move(traj));
	}

	// Apply active trajectories to the rigidBody at time t
	void TrajectoryManager::apply(systems::RigidBodySystem& rigidBody, double t) {
		for (auto it = _active.begin(); it != _active.end();) {
			const std::string& link = it->first;
			auto& traj = it->second;

			// Evaluate trajectory at time t
			const auto ref = traj->eval(t);

			// Apply trajectory reference to rigidBody joint
			const bool ok1 = rigidBody.trySetJointTargetRad(link, (float)ref.q);
			if (!ok1) { D_ERROR("Bad link key '%s' (no joint.child match)", link.c_str()); }

			const bool ok2 = rigidBody.trySetJointOmegaRefRad(link, (float)ref.qd);
			if (!ok2) { D_ERROR("Bad link key '%s' (no joint.child match)", link.c_str()); }

			const bool ok3 = rigidBody.trySetJointAlphaRefRad(link, (float)ref.qdd);
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