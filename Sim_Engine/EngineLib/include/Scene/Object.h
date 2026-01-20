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
#include "Platform/Logger.h"

namespace scene {
	enum class eInputButton;
	class ENGINE_API Input;
	class ENGINE_API Mesh;

	enum class ObjectCategory { General, RobotLink };

	struct ENGINE_API AssetSource {
		std::string filename;
		std::string filepath;
	};

	// Represents the position, rotation, and scale of an object in 3D space.
	struct ENGINE_API Transform {
		glm::vec3 position{ 0.0f };
		glm::quat rotQ{ 1.0f, 0.0f, 0.0f, 0.0f };
		glm::vec3 scale{ 0.01f, 0.01f, 0.01f };

		Transform() : position(0.0f), rotQ{ 1.0f, 0.0f, 0.0f, 0.0f }, scale(0.01f, 0.01f, 0.01f) {}
		glm::mat4 toMatrix() const;
	};

	// Represents a 3D object in the scene with a mesh, transform, and physics state.
	class ENGINE_API Object : public Element {
	public:
		// Unique identifier for the object
		scene::ObjectID id = scene::ObjectID::INVALID_OBJECT_ID;
		std::string name;

		// 3D Transform
		Transform transform;
		physics::PhysicsState state;

		// Object Category & Asset Source
		ObjectCategory category = ObjectCategory::General;
		AssetSource source;


		explicit Object(std::shared_ptr<Mesh> mesh);

		Mesh* getMesh() { return _mesh.get(); }	// mutable version
		const Mesh* getMesh() const { return _mesh.get(); } // const version

		glm::vec3 getAlbedo() const { return albedo; }
		void setAlbedo(const glm::vec3& color) { albedo = color; }

		void update(shaders::Shader* shader) override;

		void reset() {
			transform.position = glm::vec3(0.0f);
			transform.rotQ = glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f };

			state.q = Quat(1.0, 0.0, 0.0, 0.0);
			state.linearVelocity = Vec3::Zero();
			state.angularVelocity = Vec3::Zero();
			state.forces = Vec3::Zero();
			state.torques = Vec3::Zero();
		}

		void onMouseWheel(double delta) { _distance += (float)delta * 0.5f; }
		void onMouseMove(double x, double y, eInputButton button);
		void setLastMousePos(const glm::vec2& pos) { _lastMousePos = pos; }
		glm::vec2 getLastMousePos() const { return _lastMousePos; }

	private:
		std::shared_ptr<Mesh> _mesh;
		glm::vec2 _lastMousePos{ 0.0f };
		glm::vec3 albedo = glm::vec3(0.3f, 0.2f, 0.8f);
		float _distance = 5.0f;
	};
}