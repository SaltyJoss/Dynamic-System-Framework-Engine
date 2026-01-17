#pragma once

//=============================================
//            File: Object.h
//=============================================
// Class representing a 3D object in the scene with a mesh, transform, and physics state.
// 
// Summary:
// ============================================
// 
// Structs & Enumerations:
// --------------------------------------------
// enum class ObjectCategory
//    -> Enumeration for object categories (General, RobotLink).
// AssetSource
//		-> Represents the source of an asset with name and path.
// Transform
//      -> Represents the position, rotation, and scale of an object in 3D space.
// --------------------------------------------
//
// Built upon code from:
// ============================================
//	 GitHub: jayanam/jgl_demos/JGL_MeshLoader
// ============================================
// 
// ============================================
//              GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include "Physics/PhysicsState.h"
#include "Scene/ObjectID.h"
#include "Scene/Element.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include "Rendering/ShaderUtil.h"
#include "Scene/Input.h"
#include "Scene/Mesh.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace scene {
	enum class ObjectCategory { General, RobotLink };

	struct ENGINE_API AssetSource {
		std::string filename;
		std::string filepath;
	};

	// Represents the position, rotation, and scale of an object in 3D space.
	struct ENGINE_API Transform {
		glm::vec3 position{ 0.0f };
		glm::vec3 rotation{ 0.0f };
		glm::quat rotQ{ 1.0f, 0.0f, 0.0f, 0.0f };
		glm::vec3 scale{ 0.01f, 0.01f, 0.01f };

		Transform()
			: position(0.0f), rotation(0.0f), rotQ{ 1.0f, 0.0f, 0.0f, 0.0f }, scale(0.01f, 0.01f, 0.01f) {
		}

		glm::mat4 toMatrix() const;
	};

	// Represents a 3D object in the scene with a mesh, transform, and physics state.
	class ENGINE_API Object : public Element {
	public:
		// Unique identifier for the object
		ObjectID id = INVALID_OBJECT_ID;
		std::string name;

		// 3D Transform
		Transform transform;
		physics::PhysicsState state;

		// Object Category & Asset Source
		ObjectCategory category = ObjectCategory::General;
		AssetSource source;


		explicit Object(std::shared_ptr<Mesh> mesh)
			: _mesh(std::move(mesh))
		{
			transform.position = glm::vec3(0.0f);
			transform.rotation = glm::vec3(0.0f);
			transform.rotQ = glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f };
			transform.scale = glm::vec3(0.01f);

			state.theta = Vec3::Zero(); // soon to be removed
			state.q = Quat(1.0, 0.0, 0.0, 0.0);
			state.linearVelocity = Vec3::Zero();
			state.angularVelocity = Vec3::Zero();
			state.mass = 1.0;
			state.damping = 0.0; // no damping by default (for now)
			state.inertia = Mat3::Identity();
			state.forces = Vec3::Zero();
			state.torques = Vec3::Zero();
		}

		Mesh* getMesh() { return _mesh.get(); }	// mutable version
		const Mesh* getMesh() const { return _mesh.get(); } // const version

		void update(shaders::Shader* shader) override {
			if (_mesh) _mesh->update(shader);
		}

		void reset() {
			transform.position = glm::vec3(0.0f);
			transform.rotation = glm::vec4(0.0f);

			state.theta = Vec3::Zero(); // soon to be removed
			state.q = Quat(1.0, 0.0, 0.0, 0.0);
			state.linearVelocity = Vec3::Zero();
			state.angularVelocity = Vec3::Zero();
			state.forces = Vec3::Zero();
			state.torques = Vec3::Zero();
		}

		void onMouseWheel(double delta) { _distance += (float)delta * 0.5f; }

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
		glm::vec3 albedo = glm::vec3(1.0f);
		float _distance = 5.0f;
	};
}