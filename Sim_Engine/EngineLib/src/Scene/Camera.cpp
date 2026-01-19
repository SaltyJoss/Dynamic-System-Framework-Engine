
#include "pch.h"

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Scene/Camera.h"

#include "EngineLib/LogMacros.h"

namespace scene {

	void Camera::update(shaders::Shader* shader) {
		updateViewMatrix();

		glm::mat4 model{ 1.0f };
		shader->setMat4(model, "model");
		shader->setMat4(_viewMatrix, "view");
		shader->setMat4(getProjection(), "projection");
		shader->setVec3(_position, "camPos");
	}

	void Camera::processKeyboard(int key, float dt) {
		_currentSpeed = glm::mix(_currentSpeed, _targetSpeed, 1.0f - expf(-_accel * dt));
		float velocity = _currentSpeed * dt;

		switch (key) {
			case GLFW_KEY_W:			moveForward(velocity);	break;
			case GLFW_KEY_S:			moveBackward(velocity); break;
			case GLFW_KEY_A:			moveLeft(velocity);		break;
			case GLFW_KEY_D:			moveRight(velocity);	break;
			case GLFW_KEY_SPACE:		moveUp(velocity);		break;
			case GLFW_KEY_LEFT_SHIFT:	moveDown(velocity);		break;
		}

		updateViewMatrix();
	}

	void Camera::processMouseMovement(float xoffset, float yoffset, bool constrainPitch) {
		const double sensitivity = 0.001f;
		xoffset *= sensitivity;
		yoffset *= sensitivity;
		_yaw += xoffset;
		_pitch += yoffset;

		if (constrainPitch) {
			if (_pitch > glm::radians(89.0f)) { _pitch = glm::radians(89.0f); }
			if (_pitch < glm::radians(-89.0f)) { _pitch = glm::radians(-89.0f); }
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

	void Camera::moveForward(float velocity) { _position += glm::normalize(_forward) * velocity; }
	void Camera::moveBackward(float velocity) { _position -= glm::normalize(_forward) * velocity; }
	void Camera::moveLeft(float velocity) { _position -= glm::normalize(_right) * velocity; }
	void Camera::moveRight(float velocity) { _position += glm::normalize(_right) * velocity; }
	void Camera::moveUp(float velocity) { _position += glm::vec3(0.0f, 1.0f, 0.0f) * velocity; }
	void Camera::moveDown(float velocity) { _position -= glm::vec3(0.0f, 1.0f, 0.0f) * velocity; }

	void Camera::startFollow(const glm::vec3& pos, const glm::quat& rot, const glm::vec3& offset) {
		_following = true;
		_targetPos = pos;
		_targetRot = rot;
		_followOffset = offset;
	}

	void Camera::updateViewMatrix() {

		if (_following) {
			_position = _targetPos + (_targetRot * _followOffset);
			const glm::vec3 up = _targetRot * glm::vec3(0.0f, 1.0f, 0.0f);

			_viewMatrix = glm::lookAt(_position, _targetPos, up);
			return;
		}

		glm::vec3 f;
		f.x = cosf(_yaw) * cosf(_pitch);
		f.y = sinf(_pitch);
		f.z = sinf(_yaw) * cosf(_pitch);
		_forward = glm::normalize(f);

		const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
		_right = glm::normalize(glm::cross(_forward, glm::vec3(0.0f, 1.0f, 0.0f)));
		_up = glm::normalize(glm::cross(_right, _forward));

		_viewMatrix = glm::lookAt(_position, _position + _forward, _up);
	}

	// --- CAMERA MOVEMENT METHOD FOR FIXED POSITION CAMERA ---
	//void updateViewMatrix() {
	//	float yawRad = glm::radians(_yaw);
	//	float pitchRad = glm::radians(_pitch);
	//
	//	glm::vec3 f;
	//	f.x = cosf(yawRad) * cosf(pitchRad);
	//	f.y = sinf(pitchRad);
	//	f.z = sinf(yawRad) * cosf(pitchRad);
	//	_forward = glm::normalize(f);
	//
	//	const glm::vec3 worldUp(0.0f, 1.0f, 0.0f);
	//
	//	_right = glm::normalize(glm::cross(worldUp, _forward));
	//	_up = glm::normalize(glm::cross(_forward, _right));
	//
	//	_viewMatrix = glm::lookAt(_position, _position + _forward, _up);
	//}
}