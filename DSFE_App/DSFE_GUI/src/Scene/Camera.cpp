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
		_cur_spd = glm::mix(_cur_spd, _trgt_spd, 1.0f - expf(-_accel * dt));
		float vel = _cur_spd * dt;

		switch (key) {
			case static_cast<int>(gui::eKeyCode::W):		moveForward(vel);	break;
			case static_cast<int>(gui::eKeyCode::A):		moveLeft(vel);		break;
			case static_cast<int>(gui::eKeyCode::S):		moveBackward(vel);	break;
			case static_cast<int>(gui::eKeyCode::D):		moveRight(vel);	break;
			case static_cast<int>(gui::eKeyCode::Space):	moveUp(vel);		break;
			case static_cast<int>(gui::eKeyCode::LShift):	moveDown(vel);		break;
		}
		updateViewMatrix();
	}

	void Camera::processMouseMovement(float xoffset, float yoffset, bool constrainPitch) {
		const double sns = 0.001f;
		xoffset *= sns;
		yoffset *= sns;
		_yaw += xoffset;
		_pitch += yoffset;

		if (constrainPitch) {
			if (_pitch > glm::radians(89.999f)) { _pitch = glm::radians(89.999f); }
			if (_pitch < glm::radians(-89.999f)) { _pitch = glm::radians(-89.999f); }
		}
		updateViewMatrix();
	}

	std::array<glm::vec4, 8> Camera::getFrustumCornersWorldSpace(float near, float far) const {
		std::array<glm::vec4, 8> corners;

		float tan_half_fov = tanf(_fov * 0.5f);
		float near_h = tan_half_fov * near;
		float near_w = near_h * (_aspect);
		float far_h = tan_half_fov * far;
		float far_w = far_h * (_aspect);

		glm::vec3 forward = glm::normalize(_forward);
		glm::vec3 right = glm::normalize(_right);
		glm::vec3 up = glm::normalize(_up);

		glm::vec3 near_cen = _pos + forward * near;
		glm::vec3 far_cen = _pos + forward * far;

		// Near plane
		corners[0] = glm::vec4(near_cen - right * near_w + up * near_h, 1.0f); // Top-Left
		corners[1] = glm::vec4(near_cen + right * near_w + up * near_h, 1.0f); // Top-Right
		corners[2] = glm::vec4(near_cen - right * near_w - up * near_h, 1.0f); // Bottom-Left
		corners[3] = glm::vec4(near_cen + right * near_w - up * near_h, 1.0f); // Bottom-Right

		// Far plane
		corners[4] = glm::vec4(far_cen - right * far_w + up * far_h, 1.0f); // Top-Left
		corners[5] = glm::vec4(far_cen + right * far_w + up * far_h, 1.0f); // Top-Right
		corners[6] = glm::vec4(far_cen - right * far_w - up * far_h, 1.0f); // Bottom-Left
		corners[7] = glm::vec4(far_cen + right * far_w - up * far_h, 1.0f); // Bottom-Right

		return corners;
	}

	void Camera::moveForward(float vel) { _pos += glm::normalize(_forward) * vel; updateViewMatrix(); }
	void Camera::moveBackward(float vel) { _pos -= glm::normalize(_forward) * vel; updateViewMatrix(); }
	void Camera::moveLeft(float vel) { _pos -= glm::normalize(_right) * vel; updateViewMatrix(); }
	void Camera::moveRight(float vel) { _pos += glm::normalize(_right) * vel; updateViewMatrix(); }
	void Camera::moveUp(float vel) { _pos += glm::vec3(0.0f, 1.0f, 0.0f) * vel; updateViewMatrix(); }
	void Camera::moveDown(float vel) { _pos -= glm::vec3(0.0f, 1.0f, 0.0f) * vel; updateViewMatrix(); }

	void Camera::startFollow(const glm::vec3& pos, const glm::quat& rot, const glm::vec3& offset) {
		_following = true;
		_trgt_pos = pos;
		_trgt_rot = rot;
		_follow_offset = offset;
	}

	void Camera::updateViewMatrix() {
		if (_following) {
			_pos = _trgt_pos + (_trgt_rot * _follow_offset);
			const glm::vec3 up = _trgt_rot * glm::vec3(0.0f, 1.0f, 0.0f);

			_view_matrix = glm::lookAt(_pos, _trgt_pos, up);
			return;
		}
		if (_pos.y < FLOOR_Y + MIN_EYE_OFFSET) { _pos.y = FLOOR_Y + MIN_EYE_OFFSET; }
		glm::vec3 f;
		f.x = cosf(_yaw) * cosf(_pitch);
		f.y = sinf(_pitch);
		f.z = sinf(_yaw) * cosf(_pitch);
		_forward = glm::normalize(f);
		const glm::vec3 world_up(0.0f, 1.0f, 0.0f);
		_right = glm::normalize(glm::cross(_forward, world_up));
        _up    = glm::normalize(glm::cross(_right, _forward));
        _view_matrix = glm::lookAt(_pos, _pos + _forward, world_up);
	}

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
		glm::vec3 dir = target - _pos;
		float len = glm::length(dir);
		if (len < 1e-6f) return;
		dir /= len;
		// Match yaw/pitch convention:
		// Default: _yaw = -pi/2 gives forward (0,0,-1).
		_pitch = std::asin(glm::clamp(dir.y, -1.0f, 1.0f));
		_yaw = std::atan2(dir.z, dir.x);

		rebuildAxesFromFrontUp_(_forward, upHint);
		updateViewMatrix();
	}
}