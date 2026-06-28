// DSFE_GUI SimCamera.cpp
#include "Scene/SimulationManager.h"
#include "Manager/SimImplementation.h"

namespace gui {
	// Get the active view camera
	scene::Camera* SimManager::getCamera() { return _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get(); }
	// Reset the active view camera to default position
	void SimManager::resetView() {
		auto& v = _impl->_views[static_cast<size_t>(_impl->activeView)];

		glm::vec3 pos = { 0.0f, 0.25f, 1.0f };
		float fov = 60.0f;

		switch (_impl->activeView) {
			case gui::ViewID::Top:   pos = { 0, 5, 0 }; fov = 20.0f; break;
			case gui::ViewID::Right: pos = { 5, 0, 0 }; fov = 20.0f; break;
			case gui::ViewID::Front: pos = { 0, 0, 5 }; fov = 20.0f; break;
			case gui::ViewID::Follow: fov = 20.0f; break;
			case gui::ViewID::Manual: fov = 20.0f; break;
			default: break;
		}

		float aspect = (float)std::max(1, v.w) / (float)std::max(1, v.h);
		v.cam = std::make_unique<scene::Camera>(pos, fov, aspect, 0.1f, 5000.0f);

		v.cam->setFocus(glm::vec3(0.0f));

		switch (_impl->activeView) {
			case gui::ViewID::Top:
			v.cam->setYaw(-glm::half_pi<float>());
			v.cam->setPitch(-glm::half_pi<float>() + 0.001f);
			break;
			case gui::ViewID::Right:
			v.cam->setYaw(glm::pi<float>());
			v.cam->setPitch(0.0f);
			break;
			case gui::ViewID::Front:
			v.cam->setYaw(-glm::half_pi<float>());
			v.cam->setPitch(0.0f);
			break;
			default:
			break;
		}

		v.cam->updateViewMatrix();
	}

	// Attach camera to an object and start following it. The camera will maintain a fixed offset from the object's position and orientation.
	void SimManager::attachCameraToObject(scene::Object* obj) {
		if (!obj) return;
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();

		_impl->_cameraFollowTarget = obj;

		glm::vec3 pos = obj->transform.position;
		glm::quat rot = obj->transform.rotQ;

		cam->startFollow(pos, rot, glm::vec3(0, 2, 5));
	}

	// Detach camera from any object and stop following
	void SimManager::detachCameraFromObject() {
		scene::Camera* cam = _impl->_views[static_cast<size_t>(_impl->activeView)].cam.get();
		_impl->_cameraFollowTarget = nullptr;
		cam->clearFollow();
	}

	// Set the follow target for a specific view
	void SimManager::setViewFollowTarget(ViewID view, scene::Object* obj, const glm::vec3& offset) {
		if (view < ViewID::Manual || view >= ViewID::COUNT) { return; }
		auto& v = _impl->_views[static_cast<size_t>(view)];
		v.followTarget = obj;
		v.followOffset = offset;
		v.followEnabled = (obj != nullptr);

		if (v.followEnabled && v.cam && obj) {
			v.cam->startFollow(obj->transform.position, obj->transform.rotQ, offset);
		}
	}

	// Clear the follow target for a specific view
	void SimManager::clearViewFollowTarget(ViewID view) {
		if (view < ViewID::Manual || view >= ViewID::COUNT) { return; }
		auto& v = _impl->_views[static_cast<size_t>(view)];
		v.followTarget = nullptr;
		v.followEnabled = false;
		if (v.cam) { v.cam->clearFollow(); }
	}

	// Convenience for Follow view: set the follow target to the object attached to a robot joint (e.g. end-effector)
	bool SimManager::setViewFollowRobotJoint(ViewID view, const std::string& jointName, const glm::vec3& offset) {
		if (!hasRobot()) {
			LOG_WARN("setViewFollowRobotJoint: no robot loaded");
			return false;
		}

		auto& rs = _core->robotSystem();
		auto& joints = rs.joints();
		auto& links = rs.links();

		// 1) Find joint by name
		const robots::RobotJoint* jPtr = nullptr;
		for (auto& j : joints) {
			if (j.name == jointName) { jPtr = &j; break; }
		}
		if (!jPtr) {
			LOG_WARN("setViewFollowRobotJoint: joint not found: %s", jointName.c_str());
			return false;
		}

		// 2) Find child link -> attached object
		scene::Object* targetObj = nullptr;
		if (auto it = _impl->_primaryLinkObject.find(jPtr->child);
			it != _impl->_primaryLinkObject.end()) {
			targetObj = it->second;
		}

		if (!targetObj) {
			LOG_WARN("setViewFollowRobotJoint: no attached object for joint=%s child=%s",
				jointName.c_str(), jPtr->child.c_str());
			return false;
		}

		// 3) Bind the view follow target
		setViewFollowTarget(view, targetObj, offset);

		/*LOG_INFO("Follow view=%d bound to joint='%s' -> child='%s' -> obj='%s'",
			(int)view, jointName.c_str(), jPtr->child.c_str(), targetObj->name.c_str());*/

		return true;
	}

	// Convenience for Follow view
	bool SimManager::followRobotJoint(const std::string& jointName, const glm::vec3& offset) {
		return setViewFollowRobotJoint(gui::ViewID::Follow, jointName, offset);
	}
}