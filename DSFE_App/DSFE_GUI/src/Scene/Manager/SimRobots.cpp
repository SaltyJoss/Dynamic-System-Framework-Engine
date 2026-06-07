// DSFE_GUI SimRobots.cpp
#include "Scene/SimulationManager.h"
#include "Scene/SimulationCore.h"
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

		if (!_impl->_robotSystem) { return; }
		_core->loadRobotInternal(name);

		size_t startIdx = _impl->_objects.size();

		platform::ScopeGLContext guard(_makeCurrentHook, _doneCurrentHook); // Hooks may be empty under Qt: In that case the guard becomes a no-op.
		_impl->buildRobotPresentationFromModel(_impl->_robotSystem->model(), *this);

		_impl->_robotRenderer->applyTransforms(
			_impl->_robotSystem->model(),
			_impl->_robotSystem->worldTransforms()
		);

		if (auto* simInteg = _impl->_robotSystem->getIntegrator()) {
			simInteg->resetAdaptiveState();
		}

		// Attempt to find an end-effector candidate among the newly added objects and bind the Follow view to it
		scene::Object* ee = _impl->findEndEffectorFromRange(startIdx);
		if (ee) {
			_impl->eeObject = ee;
			setViewFollowTarget(gui::ViewID::Follow, ee, glm::vec3(0.0f, 0.2f, 0.6f));
			_impl->eeFollowBound = true;
			LOG_INFO("Follow view bound to end-effector candidate: %s", ee->name.c_str());
		}
	}
	void SimManager::setRobotLinkRotation(const std::string& linkName, double angle) {
		if (_impl->_robotSystem) { _impl->_robotSystem->setRobotLinkRotation(linkName, angle); }
	}
	void SimManager::setRobotRootPose(const Vec3& pos, Quat& rot) {
		if (_impl->_robotSystem) { _impl->_robotSystem->setRobotRootPose(pos, rot); }
	}
	void SimManager::setRobotRootHome(const Vec3& pos, Quat& rot) {
		if (_impl->_robotSystem) { _impl->_robotSystem->setRobotRootHome(pos, rot); }
	}
	void SimManager::resetRobot() {
		if (_impl->_robotSystem) { _impl->_robotSystem->resetRobot(); }
	}
	void SimManager::clearRobot() {
		setSelectedObject(nullptr); // deselect any selected object
		_impl->clearRobotPresentation();
		_bodyLoaded = false;

		clearViewFollowTarget(gui::ViewID::Follow);
		_impl->eeFollowBound = false;
		_impl->eeObject = nullptr;
	}
	const bool SimManager::hasRobot() const { return _impl->_robotSystem && _impl->_robotSystem->hasRobot(); }

	// Access the robot system (non-const and const versions)
	robots::RobotSystem* SimManager::robotSystem() { return _core->robotSystem(); }
	const robots::RobotSystem* SimManager::robotSystem() const { return _core->robotSystem(); }

	// Access the trajectory manager (non-const and const versions)
	control::TrajectoryManager* SimManager::traj() { return _core->trajectoryManager(); }
	const control::TrajectoryManager* SimManager::traj() const { return _core->trajectoryManager(); }
}