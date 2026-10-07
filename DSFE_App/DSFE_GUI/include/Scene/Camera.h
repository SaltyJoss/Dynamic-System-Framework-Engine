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
			_pos = position;
			_aspect = aspect;
			_near = zNear;
			_far = zFar;
			_fov = glm::radians(fov);

			setAspect(_aspect);
			updateViewMatrix();
		}

		void update();

		const glm::mat4& getProjection() const { return _proj; }
		float getNear() const { return _near; }
		float getFar() const { return _far; }
		glm::vec2 getCurrentPos2D() const { return _cur_pos_2D; }
		glm::vec3 getPosition() const { return _pos; }
		glm::mat4 getViewProjection() const { return _proj * getViewMatrix(); }
		glm::vec3 getUp() const { return _up; }
		glm::vec3 getRight() const { return _right; }
		glm::vec3 getForward() const { return _forward; }
		glm::quat getDirection() const { return glm::quat(glm::vec3(-_pitch, -_yaw, 0.0f)); }
		glm::mat4 getViewMatrix() const { return _view_matrix; }
		float getYaw() const { return _yaw; }
        float getPitch() const { return _pitch; }
		float getFOVRadians() const { return _fov; }
		float getFOVDegrees() const { return glm::degrees(_fov); }
		float getOrbitDistance() const { return _dist; }
		std::array<glm::vec4, 8> getFrustumCornersWorldSpace(float near, float far) const;

		void setPosition(const glm::vec3& pos) { _pos = pos; updateViewMatrix(); }
		void setViewMatrix(const glm::mat4& view) { _view_matrix = view; }
		void setFocus(const glm::vec3& focus) { _focus = focus; updateViewMatrix(); }
		void setCurrentPos2D(const glm::vec2& pos) { _cur_pos_2D = pos; }
		void setYaw(float yaw) { _yaw = yaw; updateViewMatrix(); }
		void setPitch(float pitch) { _pitch = pitch; updateViewMatrix(); }
		void setOrbitDistance(float d) { _dist = glm::max(_min_dist, d); updateViewMatrix(); }
		void setMinDistance(float d) { _min_dist = glm::max(0.05f, d); }
		void setFollowTarget(const glm::vec3& pos, const glm::quat& rot) { _trgt_pos = pos; _trgt_rot = rot; }
		
		void setAspect(float aspect) {
			_aspect = aspect;
			updateProjectionMatrix();
		}

		void setDistance(float offset) { _dist = glm::max(_min_dist, _dist + offset); updateViewMatrix(); }

		void setFOVRadians(float rad) {
			rad = glm::clamp(rad, glm::radians(10.0f), glm::radians(150.0f));
			_fov = rad;
			updateProjectionMatrix();
		}

		void setFOVDegrees(float deg) {
			deg = glm::clamp(deg, 10.0f, 150.0f);
			_fov = glm::radians(deg);
			updateProjectionMatrix();
		}

		void reset() { _focus = { 0.0f, 0.0f, 0.0f }; updateViewMatrix(); }

		void onMouseWheel(double delta) { setDistance((float)(-delta * 0.5f)); }
		void startFollow(const glm::vec3& pos, const glm::quat& rot, const glm::vec3& offset);
		void clearFollow() { _following = false; }

		void updateViewMatrix();

		void moveForward(float delta);
		void moveBackward(float delta);
		void moveLeft(float delta);
		void moveRight(float delta);
		void moveUp(float delta);
		void moveDown(float delta);

		void processKeyboard(int key, float delta);
		void processMouseMovement(float xoffset, float yoffset, bool constrainPitch = true);
		void lookAt(const glm::vec3& target, const glm::vec3& upHint = glm::vec3(0, 1, 0));

	private:
		static constexpr float FLOOR_Y = 0.0f;
		static constexpr float MIN_EYE_OFFSET = 0.05f;

		enum class eKeyCode {
			W, A, S, D,
			LShift, Ctrl,
			Space,
			Tab,
			Unknown
		};

		void rebuildAxesFromFrontUp_(const glm::vec3& front, const glm::vec3& upHint);
        void updateProjectionMatrix() {
            if (!std::isfinite(_fov) || _fov < 0.001f) { _fov = glm::radians(50.0f); }
            _proj = glm::perspective(_fov, _aspect, _near, _far);
            _proj[1][1] *= -1.0f;   // Vulkan clip-space Y flip
        }

		// Target position and rotation for following
		glm::vec3 _trgt_pos;
		glm::vec3 _follow_offset;
		glm::quat _trgt_rot{ 1.0f, 0.0f, 0.0f, 0.0f };
		// Camera matrices
		glm::mat4 _view_matrix;
		glm::mat4 _proj{ 1.0f };
		// Camera state
		glm::vec3 _pos{ 0.0f };
		glm::vec3 _focus{ 0.0f };

		glm::vec2 _cur_pos_2D = { 0.0f, 0.0f };

		// Orientation Vectors (xyz basis vectors)
		glm::vec3 _right = { 1.0f, 0.0f, 0.0f };
		glm::vec3 _up = { 0.0f, 1.0f, 0.0f };
		glm::vec3 _forward = { 0.0f, 0.0f, -1.0f };

		float _dist = 5.0f;
		float _min_dist = 0.5f;
		float _aspect;
		float _fov;
		float _near;
		float _far;
		float _pitch = 0.0f;
		float _yaw = -glm::half_pi<float>();
		float _cur_spd = 0.0f;
		float _trgt_spd = 5.0f;
		float _accel = 10.0f;

		bool _following = false;
	};
} // namespace scene