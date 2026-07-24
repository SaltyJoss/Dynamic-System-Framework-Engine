// DSFE_GUI Camera.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include "Scene/Element.h"
#include "Platform/Logger.h"

namespace scene {
	class Camera : public Element {
	public:
		Camera(const glm::vec3& position, float fov, float aspect, float zNear, float zFar) {
			_position = position;
			_aspect = aspect;
			_near = zNear;
			_far = zFar;
			_FOV = glm::radians(fov);

			setAspect(_aspect);
			updateViewMatrix();
		}

		void update();

		const glm::mat4& getProjection() const { return _projection; }
		float getNear() const { return _near; }
		float getFar() const { return _far; }
		glm::vec2 getCurrentPos2D() const { return _currentPos2D; }
		glm::vec3 getPosition() const { return _position; }
		glm::mat4 getViewProjection() const { return _projection * getViewMatrix(); }
		glm::vec3 getUp() const { return _up; }
		glm::vec3 getRight() const { return _right; }
		glm::vec3 getForward() const { return _forward; }
		glm::quat getDirection() const { return glm::quat(glm::vec3(-_pitch, -_yaw, 0.0f)); }
		glm::mat4 getViewMatrix() const { return _viewMatrix; }
		float getYaw() const { return _yaw; }
        float getPitch() const { return _pitch; }

		void setPosition(const glm::vec3& pos) { _position = pos; updateViewMatrix(); }
		void setViewMatrix(const glm::mat4& view) { _viewMatrix = view; }
		void setAspect(float aspect) {
			_aspect = aspect;
			updateProjectionMatrix();
		}
		void setFocus(const glm::vec3& focus) { _focus = focus; updateViewMatrix(); }
		void setCurrentPos2D(const glm::vec2& pos) { _currentPos2D = pos; }
		void setYaw(float yaw) { _yaw = yaw; updateViewMatrix(); }
		void setPitch(float pitch) { _pitch = pitch; updateViewMatrix(); }
		
		void setDistance(float offset) {
			_distance = glm::max(_minDistance, _distance + offset);
			updateViewMatrix();
		}

		void reset() {
			_focus = { 0.0f, 0.0f, 0.0f };
			//_distance = 5.0f;
			updateViewMatrix();
		}

		void onMouseWheel(double delta) { setDistance((float)(-delta * 0.5f)); }

		void startFollow(const glm::vec3& pos, const glm::quat& rot, const glm::vec3& offset);
		void clearFollow() { _following = false; }
		void setFollowTarget(const glm::vec3& pos, const glm::quat& rot) { 
			_targetPos = pos;
			_targetRot = rot;
		}

		void updateViewMatrix();
		void fall();

		void moveForward(float delta);
		void moveBackward(float delta);
		void moveLeft(float delta);
		void moveRight(float delta);
		void moveUp(float delta);
		void moveDown(float delta);

		void processKeyboard(int key, float delta);
		void processMouseMovement(float xoffset, float yoffset, bool constrainPitch = true);
		void clampToFloor(float floorY);

		void lookAt(const glm::vec3& target, const glm::vec3& upHint = glm::vec3(0, 1, 0));

		std::array<glm::vec4, 8> getFrustumCornersWorldSpace(float near, float far) const;

		float getFOVRadians() const { return _FOV; }
		float getFOVDegrees() const { return glm::degrees(_FOV); }

		void setFOVRadians(float rad) {
			rad = glm::clamp(rad, glm::radians(10.0f), glm::radians(150.0f));
			_FOV = rad;
			updateProjectionMatrix();
		}

		void setFOVDegrees(float deg) {
			deg = glm::clamp(deg, 10.0f, 150.0f);
			_FOV = glm::radians(deg);
			updateProjectionMatrix();
		}

		void setOrbitDistance(float d) {
			_distance = glm::max(_minDistance, d);
			updateViewMatrix();
		}
		float getOrbitDistance() const { return _distance; }

		void setMinDistance(float d) { _minDistance = glm::max(0.05f, d); }

	private:
		enum class eKeyCode {
			W, A, S, D,
			LShift, Ctrl,
			Space,
			Tab,
			Unknown
		};

		void rebuildAxesFromFrontUp_(const glm::vec3& front, const glm::vec3& upHint);
        void updateProjectionMatrix() {
            if (!std::isfinite(_FOV) || _FOV <= 0.001f) { _FOV = glm::radians(70.0f); }
            _projection = glm::perspective(_FOV, _aspect, _near, _far);
            _projection[1][1] *= -1.0f;   // Vulkan clip-space Y flip
        }

		bool _following = false;
		glm::vec3 _targetPos;
		glm::vec3 _followOffset;
		glm::quat _targetRot{ 1.0f, 0.0f, 0.0f, 0.0f };

		glm::mat4 _viewMatrix;
		glm::mat4 _projection{ 1.0f };

		glm::vec3 _position{ 0.0f };
		glm::vec3 _focus{ 0.0f };
		glm::vec3 _velocity{ 0.0f };

		float _distance = 5.0f;
		float _minDistance = 0.5f;
		float _aspect;
		float _FOV;
		float _near;
		float _far;
		float _pitch = 0.0f;
		float _yaw = -glm::half_pi<float>();
		float _currentSpeed = 0.0f;
		float _targetSpeed = 5.0f;
		float _accel = 10.0f;
		float _eyeHeight = 1.8f;

		glm::vec2 _currentPos2D = { 0.0f, 0.0f };
		
		// --- ORIENTATION VECTORS ---
		// --> xyz basis vectors
		glm::vec3 _right = { 1.0f, 0.0f, 0.0f };
		glm::vec3 _up = { 0.0f, 1.0f, 0.0f };
		glm::vec3 _forward = { 0.0f, 0.0f, -1.0f };

		const float _rotationSpeed = 2.0f;

		// --- OBJECTS --- 
		bool _isGrounded = true;
		float _verticalVelocity = 0.0f;
		float _upwardForce = 5.5f;
		float _gravity = -9.81f;

	};
} // namespace scene