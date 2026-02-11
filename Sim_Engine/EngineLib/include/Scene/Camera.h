#pragma once
// File:   Camera.h
// GitHub: SaltyJoss
#include "EngineCore.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

#include "Scene/Element.h"
#include "Rendering/ShaderUtil.h"
#include "Scene/Input.h"
#include "Platform/Logger.h"

namespace scene {
	class ENGINE_API Camera : public Element {
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

		void update(shaders::Shader* shader);

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

		void setAspect(float aspect) {
			_aspect = aspect;
			updateProjectionMatrix();
		}
		void setFocus(const glm::vec3& focus) { _focus = focus; updateViewMatrix(); }
		void setCurrentPos2D(const glm::vec2& pos) { _currentPos2D = pos; }
		void setYaw(float yaw) { _yaw = yaw; updateViewMatrix(); }
		void setPitch(float pitch) { _pitch = pitch; updateViewMatrix(); }
		
		void setDistance(float offset) {
			_distance += offset;
			updateViewMatrix();
		}

		void reset() {
			_focus = { 0.0f, 0.0f, 0.0f };
			//_distance = 5.0f;
			updateViewMatrix();
		}

		void onMouseWheel(double delta) {
			setDistance((float)(delta * 0.5f));
			updateViewMatrix();
		}

		void onMouseMove(double x, double y, eInputButton button) {
			glm::vec2 pos2d{ x, y };

			if (button == eInputButton::Right) {
				glm::vec2 delta = (pos2d - _currentPos2D) * 0.004f;

				float sign = getUp().y < 0 ? -1.0f : 1.0f;

				_yaw += sign * delta.x * _rotationSpeed;
				_pitch += delta.y * _rotationSpeed;

				updateViewMatrix();
			}
			else if (button == eInputButton::Left) {
				glm::vec2 delta = (pos2d - _currentPos2D) * 0.003f;

				_focus += -getRight() * delta.x * _distance;
				_focus += getUp() * delta.y * _distance;

				updateViewMatrix();
			}

			_currentPos2D = pos2d;
		}

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
			_distance = glm::max(0.05f, d);
			updateViewMatrix();
		}
		float getOrbitDistance() const { return _distance; }

	private:
		void rebuildAxesFromFrontUp_(const glm::vec3& front, const glm::vec3& upHint);
		void updateProjectionMatrix() {
			if (!std::isfinite(_FOV) || _FOV <= 0.001f) { _FOV = glm::radians(70.0f); }
			_projection = glm::perspective(_FOV, _aspect, _near, _far);
		}

		bool _following = false;
		glm::vec3 _targetPos;
		glm::vec3 _followOffset;
		glm::quat _targetRot{ 1.0f, 0.0f, 0.0f, 0.0f };

		glm::mat4 _viewMatrix;
		glm::mat4 _projection  = glm::mat4{ 1.0f };

		glm::vec3 _position = { 0.0f, 0.0f, 0.0f };
		glm::vec3 _focus = { 0.0f, 0.0f, 0.0f };
		glm::vec3 _velocity{ 0.0f };

		float _distance = 5.0f;
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