// DSFE_GUI SimRobots.cpp
#include "Scene/SimulationCore.h"
#include "Scene/SimulationManager.h"
#ifdef __gl_h_
#undef __gl_h_
#endif
#include "Manager/SimImplementation.h"
#include "Platform/ScopeGLContext.h"

namespace gui {
	// Load a robot by name from the robot system
	void SimManager::loadRobot(const std::string& name) {
		// Clear any existing robot first
		if (hasRobot()) { clearRobot(); }
		_bodyLoaded = true;

		setSelectedObject(nullptr);
		detachCameraFromObject();
		clearViewFollowTarget(gui::ViewID::Follow);
		_impl->eeFollowBound = false;
		_impl->eeObject = nullptr;

		_core->loadRobot(name);
		auto& rs = _core->robotSystem();

		size_t startIdx = _impl->_objects.size();

		platform::ScopeGLContext guard(_makeCurrentHook, _doneCurrentHook); // Hooks may be empty under Qt: In that case the guard becomes a no-op.
		_impl->buildRobotPresentationFromModel(rs.model());
		_impl->_robotRenderer->applyTransforms(rs.model(), rs.worldTransforms());
		if (auto* simInteg = rs.getIntegrator()) { simInteg->resetAdaptiveState(); }

		// Attempt to find an end-effector candidate among the newly added objects and bind the Follow view to it
		scene::Object* ee = _impl->findEndEffectorFromRange(startIdx);
		if (ee) {
			_impl->eeObject = ee;
			setViewFollowTarget(gui::ViewID::Follow, ee, glm::vec3(0.0f, 0.2f, 0.6f));
			_impl->eeFollowBound = true;
			LOG_INFO("Follow view bound to end-effector candidate: %s", ee->name.c_str());
		}
	}
	// Set the rotation of a specific robot link by name
	void SimManager::setRobotLinkRotation(const std::string& linkName, double angle) { _core->robotSystem().setRobotLinkRotation(linkName, angle); }
	// Set the position and rotation of the robot's root link
	void SimManager::setRobotRootPose(const Vec3& pos, Quat& rot) { _core->robotSystem().setRobotRootPose(pos, rot);}
	// Set the home position and rotation of the robot's root link
	void SimManager::setRobotRootHome(const Vec3& pos, Quat& rot) { _core->robotSystem().setRobotRootHome(pos, rot); }
	// Reset the robot system to its initial state
	void SimManager::resetRobot() { _core->robotSystem().resetRobot(); }
	// Clear the currently loaded robot and its associated scene objects
	void SimManager::clearRobot() {
		setSelectedObject(nullptr); // deselect any selected object
		_impl->clearRobotPresentation();
		_bodyLoaded = false;

		clearViewFollowTarget(gui::ViewID::Follow);
		_impl->eeFollowBound = false;
		_impl->eeObject = nullptr;
	}

	// Conditional accessors for the robot system and single body system
	const bool SimManager::hasRobot() const { return _core->robotSystem().hasRobot(); }
	const bool SimManager::hasBody() const { return _core->singleBodySystem().hasBody(); }

	// Access the robot system (non-const and const versions)
	robots::RobotSystem& SimManager::robotSystem() { return _core->robotSystem(); }
	const robots::RobotSystem& SimManager::robotSystem() const { return _core->robotSystem(); }

	// Access the single body system (non-const and const versions)
	single_body_system::SingleBodySystem& SimManager::singleBodySystem() { return _core->singleBodySystem(); }
	const single_body_system::SingleBodySystem& SimManager::singleBodySystem() const { return _core->singleBodySystem(); }

	// Access the trajectory manager (non-const and const versions)
	control::TrajectoryManager& SimManager::traj() { return _core->trajectoryManager(); }
	const control::TrajectoryManager& SimManager::traj() const { return _core->trajectoryManager(); }
}