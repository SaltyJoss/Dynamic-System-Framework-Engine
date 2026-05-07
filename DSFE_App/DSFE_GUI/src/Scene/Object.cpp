// DSFE_GUI Object.cpp
#include "Scene/Object.h"

#include <MathLibAPI.h>
#include <core/Types.h>
#include <core/constants.h>

#include <glm/gtx/quaternion.hpp>

#include "Physics/PhysicsState.h"
#include "Scene/ObjectID.h"
#include "Scene/Input.h"
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

	// Constructor initializes transform and physics state
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

	// Handle mouse movement for object manipulation
	void Object::onMouseMove(double x, double y, eInputButton button) {
		glm::vec2 pos2d{ x, y };
		glm::vec2 delta = pos2d - _lastMousePos;
		_lastMousePos = pos2d;

		if (button == eInputButton::Right) {
			delta *= 0.004f;
			float yaw = delta.x;
			float pitch = -delta.y;

			glm::quat qYaw = glm::angleAxis(yaw, glm::vec3(0.0f, 1.0f, 0.0f));
			glm::vec3 right = transform.rotQ * glm::vec3(1.0f, 0.0f, 0.0f);
			glm::quat qPitch = glm::angleAxis(pitch, glm::normalize(right));

		}
		else if (button == eInputButton::Left) {
			delta *= 0.003f;
			transform.position += glm::vec3(delta.x * _distance, -delta.y * _distance, 0.0f);
		}
	}
}
