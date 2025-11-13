
#include "pch.h"

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Scene/Camera.h"

#include "EngineLib/LogMacros.h"

namespace elements {

	void Camera::processKeyboard(int key, float dt) {
		_currentSpeed = glm::mix(_currentSpeed, _targetSpeed, 1.0f - expf(-_accel * dt));
		float velocity = _currentSpeed * dt;

		if (key == GLFW_KEY_W)
			moveForward(velocity);
		if (key == GLFW_KEY_S)
			moveBackward(velocity);
		if (key == GLFW_KEY_A)
			moveLeft(velocity);
		if (key == GLFW_KEY_D)
			moveRight(velocity);
		updateViewMatrix();
	}

	void Camera::processMouseMovement(float xoffset, float yoffset, bool constrainPitch) {
		const double sensitivity = 0.001f;
		xoffset *= sensitivity;
		yoffset *= sensitivity;
		_yaw += xoffset;
		_pitch += yoffset;

		if (constrainPitch) {
			if (_pitch > glm::radians(89.0f))
				_pitch = glm::radians(89.0f);
			if (_pitch < glm::radians(-89.0f))
				_pitch = glm::radians(-89.0f);
		}
		updateViewMatrix();
	}

	void Camera::clampToFloor(float floorY) {
		float minY = floorY + _eyeHeight; // camera's eyes stay above floor

		if (_position.y < minY) {
			_position.y = minY;
			_velocity.y = 0.0f;
			updateViewMatrix();
		}
	}

	void Camera::applyGravity(float dt, float floorY)
	{
		// Only apply gravity if not grounded
		if (!_isGrounded)
		{
			_verticalVelocity += _gravity * dt;
			_position.y += _verticalVelocity * dt;

			// Clamp to floor
			float minY = floorY + _eyeHeight;
			if (_position.y <= minY)
			{
				_position.y = minY;
				_verticalVelocity = 0.0f;
				_isGrounded = true;
			}

			updateViewMatrix();
		}
	}

	void Camera::jump()
	{
		if (_isGrounded)
		{
			_isGrounded = false;
			_verticalVelocity = _jumpStrength;
		}
	}

	void Camera::moveForward(float velocity) {
		glm::vec3 dir = glm::vec3(_forward.x, 0.0f, _forward.z);
		_position += glm::normalize(dir) * velocity;
	}

	void Camera::moveBackward(float velocity) {
		glm::vec3 dir = glm::vec3(_forward.x, 0.0f, _forward.z);
		_position -= glm::normalize(dir) * velocity;
	}

	void Camera::moveLeft(float velocity) {
		glm::vec3 dir = glm::vec3(_right.x, 0.0f, _right.z);
		_position -= glm::normalize(dir) * velocity;
	}

	void Camera::moveRight(float velocity) {
		glm::vec3 dir = glm::vec3(_right.x, 0.0f, _right.z);
		_position += glm::normalize(dir) * velocity;
	}
}