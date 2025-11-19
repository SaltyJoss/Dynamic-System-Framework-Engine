
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
		if (key == GLFW_KEY_SPACE)
			moveUp(velocity);
		if (key == GLFW_KEY_LEFT_SHIFT)
			moveDown(velocity);
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

	std::array<glm::vec4, 8> Camera::getFrustumCornersWorldSpace(float, float) const {
		std::array<glm::vec4, 8> corners;

		float tanHalfFov = tanf(_FOV * 0.5f);
		float nearHeight = tanHalfFov * _near;
		float nearWidth = nearHeight * (_aspect);
		float farHeight = tanHalfFov * _far;
		float farWidth = farHeight * (_aspect);

		glm::vec3 forward = glm::normalize(_forward);
		glm::vec3 right = glm::normalize(_right);
		glm::vec3 up = glm::normalize(_up);

		glm::vec3 nearCenter = _position + forward * _near;
		glm::vec3 farCenter = _position + forward * _far;

		// Near plane
		corners[0] = glm::vec4(nearCenter - right * nearWidth + up * nearHeight, 1.0f); // Top-Left
		corners[1] = glm::vec4(nearCenter + right * nearWidth + up * nearHeight, 1.0f); // Top-Right
		corners[2] = glm::vec4(nearCenter - right * nearWidth - up * nearHeight, 1.0f); // Bottom-Left
		corners[3] = glm::vec4(nearCenter + right * nearWidth - up * nearHeight, 1.0f); // Bottom-Right

		// Far plane
		corners[4] = glm::vec4(farCenter - right * farWidth + up * farHeight, 1.0f); // Top-Left
		corners[5] = glm::vec4(farCenter + right * farWidth + up * farHeight, 1.0f); // Top-Right
		corners[6] = glm::vec4(farCenter - right * farWidth - up * farHeight, 1.0f); // Bottom-Left
		corners[7] = glm::vec4(farCenter + right * farWidth - up * farHeight, 1.0f); // Bottom-Right

		return corners;
	}

	void Camera::fall()
	{
		if (_isGrounded)
		{
			_isGrounded = false;
			_verticalVelocity = _upwardForce;
		}
	}

	void Camera::moveForward(float velocity) {
		_position += glm::normalize(_forward) * velocity;
	}

	void Camera::moveBackward(float velocity) {
		_position -= glm::normalize(_forward) * velocity;
	}

	void Camera::moveLeft(float velocity) {
		_position -= glm::normalize(_right) * velocity;
	}

	void Camera::moveRight(float velocity) {
		_position += glm::normalize(_right) * velocity;
	}

	void Camera::moveUp(float velocity) {
		_position += glm::vec3(0.0f, 1.0f, 0.0f) * velocity;
	}

	void Camera::moveDown(float velocity) {
		_position -= glm::vec3(0.0f, 1.0f, 0.0f) * velocity;
	}
}