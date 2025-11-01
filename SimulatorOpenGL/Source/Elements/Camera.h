#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include "Element.h"
#include "Shader/ShaderUtil.h"
#include "Input.h"

namespace elements {
	class Camera : public Element
	{
	public:

		Camera(const glm::vec3& position, float fov, float aspect, float near, float far) {
			_position = position;
			_aspect = aspect;
			_near = near;
			_far = far;
			_FOV = fov;

			setAspect(_aspect);
			updateViewMatrix();
		}

		void update(shaders::Shader* shader) override {
			glm::mat4 model{ 1.0f };
			shader->setMat4(model, "model");
			shader->setMat4(_viewMatrix, "view");
			shader->setMat4(getProjection(), "projection");
			shader->setVec3(_position, "camPos");
		}

		void setAspect(float aspect) { _projection  = glm::perspective(_FOV, aspect, _near, _far); }

		void setDistance(float offset) {
			_distance += offset;
			updateViewMatrix();
		}

		const glm::mat4& getProjection() const { return _projection; }

		glm::vec3 getPosition() const { return _position; }
		glm::mat4 getViewProjection() const { return _projection * getViewMatrix(); }
		glm::vec3 getUp() const { return glm::rotate(getDirection(), _up); }
		glm::vec3 getRight() const { return glm::rotate(getDirection(), _right); }
		glm::vec3 getForward() const { return glm::rotate(getDirection(), _forward); }
		glm::quat getDirection() const { return glm::quat(glm::vec3(-_pitch, -_yaw, 0.0f)); }
		glm::mat4 getViewMatrix() const { return _viewMatrix; }

		void onMouseWheel(double delta) {
			setDistance(delta * 0.5f);
			updateViewMatrix();
		}

		void reset()
		{
			_focus = { 0.0f, 0.0f, 0.0f };
			//_distance = 5.0f;
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
			else if (button == eInputButton::Middle) {
				// TODO: Adjust pan speed for distance
				glm::vec2 delta = (pos2d - _currentPos2D) * 0.003f;

				_focus += -getRight() * delta.x * _distance;
				_focus += getUp() * delta.y * _distance;

				updateViewMatrix();
			}

			_currentPos2D = pos2d;
		}

		void updateViewMatrix() {
			_position = _focus - getForward() * _distance;

			glm::quat orientation = getDirection();
			_viewMatrix = glm::translate(glm::mat4(1.0f), _position) * glm::toMat4(orientation);
			_viewMatrix = glm::inverse(_viewMatrix);
		}

	private:
		glm::mat4 _viewMatrix;
		glm::mat4 _projection  = glm::mat4{ 1.0f };
		glm::vec3 _position = { 0.0f, 0.0f, 0.0f };
		glm::vec3 _focus = { 0.0f, 0.0f, 0.0f };

		float _distance = 5.0f;
		float _aspect;
		float _FOV;
		float _near;
		float _far;
		float _pitch = 0.0f;
		float _yaw = 0.0f;

		glm::vec2 _currentPos2D = { 0.0f, 0.0f };
		const glm::vec3 _right = { 1.0f, 0.0f, 0.0f };
		const glm::vec3 _up = { 0.0f, 1.0f, 0.0f };
		const glm::vec3 _forward = { 0.0f, 0.0f, -1.0f };

		const float _rotationSpeed = 2.0f;

	};
}