#pragma once

#include "EngineCore.h"
#include "Physics/PhysicsState.h"
#include "Scene/Element.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include "Rendering/ShaderUtil.h"
#include "Scene/Input.h"
#include "Scene/Mesh.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace elements {
	struct Transform {
		glm::vec3 position{ 0.0f };
		glm::vec3 rotation{ 0.0f }; // Euler angles
		glm::vec3 scale{ 0.01f, 0.01f, 0.01f };

		Transform()
			: position(0.0f), rotation(0.0f), scale(0.01f, 0.01f, 0.01f) {
		}

		glm::mat4 toMatrix() const;
	};

	class ENGINE_API Object : public Element {
	public:
		Transform transform;
		physics::PhysicsState state;

		explicit Object(std::shared_ptr<Mesh> mesh)
			: _mesh(std::move(mesh))
		{
			transform.position = glm::vec3(0.0f);
			transform.rotation = glm::vec3(0.0f);
			transform.scale = glm::vec3(0.01f);

			state.theta = Eigen::Vector3d::Zero();
			state.linearVelocity = Eigen::Vector3d::Zero();
			state.angularVelocity = Eigen::Vector3d::Zero();
			state.mass = 1.0;
			state.inertia = Eigen::Matrix3d::Identity();
			state.forces = Eigen::Vector3d::Zero();
			state.torques = Eigen::Vector3d::Zero();
		}

		Mesh* getMesh() { return _mesh.get(); }
		const Mesh* getMesh() const { return _mesh.get(); }

		void update(shaders::Shader* shader) override {
			if (_mesh) _mesh->update(shader);
		}

		void reset() {
			transform.position = glm::vec3(0.0f);
			transform.rotation = glm::vec3(0.0f);
			transform.scale = glm::vec3(0.01f);

			state.theta = Eigen::Vector3d::Zero();
			state.linearVelocity = Eigen::Vector3d::Zero();
			state.angularVelocity = Eigen::Vector3d::Zero();
			state.forces = Eigen::Vector3d::Zero();
			state.torques = Eigen::Vector3d::Zero();
		}

		void onMouseWheel(double delta) { _distance += delta * 0.5f; }

		void onMouseMove(double x, double y, eInputButton button) {
			glm::vec2 pos2d{ x, y };
			glm::vec2 delta = pos2d - _lastMousePos;
			_lastMousePos = pos2d;

			if (button == eInputButton::Right) {
				delta *= 0.004f;
				transform.rotation.x += -delta.y;
				transform.rotation.y += delta.x;

			}
			else if (button == eInputButton::Left) {
				delta *= 0.003f;
				transform.position += glm::vec3(delta.x * _distance, -delta.y * _distance, 0.0f);

			}

		}

		void setLastMousePos(const glm::vec2& pos) { _lastMousePos = pos; }
		glm::vec2 getLastMousePos() const { return _lastMousePos; }

	private:
		std::shared_ptr<Mesh> _mesh;
		glm::vec2 _lastMousePos{ 0.0f };
		float _distance = 5.0f;

	};
}