#pragma once
#include "EngineCore.h"
#include <MathLibAPI.h>  // your DLL export

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "Scene/Object.h"
#include "integrators/numerical_integrators.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;
using namespace integration;

namespace elements {
	class Mesh;
}

namespace physics {
	class ENGINE_API PhysicsSystem {
	public:
		PhysicsSystem();
		~PhysicsSystem();

		void updateRotation(double dt, elements::Object* object);

	private:
		std::unique_ptr<ODE> _integratorODE;

		double _angle = 0.0f;
	};
} // namespace physics