// DSFE_GUI Object.cpp
#include "Scene/Object.h"


#include <core/Types.h>
#include <core/constants.h>

#include <glm/gtx/quaternion.hpp>

#include "Physics/PhysicsState.h"
#include "Scene/ObjectID.h"
#include "Scene/Mesh.h"

using namespace mathlib;

namespace scene {
	// Convert Transform to a 4x4 matrix for rendering
	glm::mat4 Transform::toMatrix() const {
		glm::mat4 model(1.0f);
		model = glm::translate(model, position);
		model *= glm::toMat4(rotQ); // quaternions use!
		model = glm::scale(model, scale);
		return model;
	}

	// --- Object Implementation ---

	// Constructor initialises transform and physics state
	Object::Object(std::shared_ptr<Mesh> mesh) : _mesh(std::move(mesh)) {
		transform.position = glm::vec3(0.0f);
		transform.rotQ = glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f };
		transform.scale = glm::vec3(0.01f);

		state.q = Quat(1.0, 0.0, 0.0, 0.0);
		state.linearVelocity = Vec3::Zero();
		state.angularVelocity = Vec3::Zero();
		state.mass = 1.0;
		state.damping = 0.0; // no damping by default (for now)
		state.inertia = Mat3::Identity();
		state.forces = Vec3::Zero();
		state.torques = Vec3::Zero();
	}

	// Update method passes material properties to the shader
	void Object::update(shaders::Shader* shader) { if (_mesh) _mesh->update(shader); }
}
