// DSFE_GUI Camera.cpp
#include <array>
#include "Scene/Camera.h"
#include "Platform/KeyCode.h"
#include "EngineLib/LogMacros.h"

namespace scene {
	void Camera::update() {
		updateViewMatrix();

		glm::mat4 model{ 1.0f };
	}

	void Camera::processKeyboard(int key, float dt) {
		_currentSpeed = glm::mix(_currentSpeed, _targetSpeed, 1.0f - expf(-_accel * dt));
		float velocity = _currentSpeed * dt;

		switch (key) {
			case static_cast<int>(gui::eKeyCode::W):		moveForward(velocity);	break;
			case static_cast<int>(gui::eKeyCode::A):		moveLeft(velocity);		break;
			case static_cast<int>(gui::eKeyCode::S):		moveBackward(velocity);	break;
			case static_cast<int>(gui::eKeyCode::D):		moveRight(velocity);	break;
			case static_cast<int>(gui::eKeyCode::Space):	moveUp(velocity);		break;
			case static_cast<int>(gui::eKeyCode::LShift):	moveDown(velocity);		break;
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
			if (_pitch > glm::radians(89.999f)) { _pitch = glm::radians(89.999f); }
			if (_pitch < glm::radians(-89.999f)) { _pitch = glm::radians(-89.999f); }
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

	std::array<glm::vec4, 8> Camera::getFrustumCornersWorldSpace(float near, float far) const {
		std::array<glm::vec4, 8> corners;

		float tanHalfFov = tanf(_FOV * 0.5f);
		float nearHeight = tanHalfFov * near;
		float nearWidth = nearHeight * (_aspect);
		float farHeight = tanHalfFov * far;
		float farWidth = farHeight * (_aspect);

		glm::vec3 forward = glm::normalize(_forward);
		glm::vec3 right = glm::normalize(_right);
		glm::vec3 up = glm::normalize(_up);

		glm::vec3 nearCenter = _position + forward * near;
		glm::vec3 farCenter = _position + forward * far;

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

	void Camera::fall() {
		if (_isGrounded)  {
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

		rebuildAxesFromFrontUp_(f, worldUp);

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

	// Rebuild the camera axes based on a given front vector and an up hint
	void Camera::rebuildAxesFromFrontUp_(const glm::vec3& front, const glm::vec3& upHint) {
		_forward = glm::normalize(front);

		glm::vec3 up = glm::normalize(upHint);

		// If up is nearly parallel to forward, choose a safe fallback
		if (glm::abs(glm::dot(_forward, up)) > 0.999f) {
			up = (glm::abs(_forward.y) < 0.999f) ? glm::vec3(0, 1, 0) : glm::vec3(0, 0, 1);
		}

		_right = glm::normalize(glm::cross(_forward, up));
		_up = glm::normalize(glm::cross(_right, _forward));
	}

	// Orient the camera to look at a target point with an up hint
	void Camera::lookAt(const glm::vec3& target, const glm::vec3& upHint) {
		// Keep orbit focus consistent
		_focus = target;

		glm::vec3 dir = target - _position;
		float len = glm::length(dir);
		if (len < 1e-6f) return;
		dir /= len;

		// Match yaw/pitch convention:
		// Default: _yaw = -pi/2 gives forward (0,0,-1).
		// That corresponds to:
		// forward.x = cos(yaw)*cos(pitch)
		// forward.y = sin(pitch)
		// forward.z = sin(yaw)*cos(pitch)
		_pitch = std::asin(glm::clamp(dir.y, -1.0f, 1.0f));
		_yaw = std::atan2(dir.z, dir.x);

		rebuildAxesFromFrontUp_(_forward, upHint);
		updateViewMatrix();
	}
}