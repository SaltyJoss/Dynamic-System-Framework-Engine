
#include "pch.h"
#include "Scene/Mesh.h"
#include "Scene/PhysicsSystem.h"
#include "integrators/Integration.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/transform.hpp>

#include "EngineLib/LogMacros.h"

namespace physics {
	PhysicsSystem::PhysicsSystem() {
		_integratorODE = std::make_unique<ODE>();

		LOG_INFO("PhysicsSystem initialized.");
	}

	PhysicsSystem::~PhysicsSystem() {
		LOG_INFO("PhysicsSystem destroyed.");
	}

	void PhysicsSystem::updateRotation(double dt, elements::Object* object) {

		if (!object || !object->getMesh())
			return;

		auto mesh = object->getMesh();

		Eigen::VectorXd x(1);
		x(0) = _angle;

		Eigen::VectorXd dxdt(1);
		dxdt(0) = 1.0f; // rad/s

		Eigen::VectorXd next = _integratorODE->eulerStep(x, dxdt, double(dt));
		_angle = next(0);

		mesh->_rotation.y = float(_angle);
	}
} // namespace physics