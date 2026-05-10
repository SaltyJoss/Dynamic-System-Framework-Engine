// DSFE_GUI Object.h
#pragma once
// -----
// Initially templated off a tutorial:
// GitHub: jayanam/jgl_demos/JGL_MeshLoader
#include "Physics/PhysicsState.h"

#include "Scene/ObjectID.h"
#include "Scene/Element.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "Platform/Logger.h"
#include "EngineLib/LogMacros.h"

namespace scene {
	enum class eInputButton;
	class Input;
	class Mesh;

	enum class ObjectCategory { General, RobotLink };

	//struct DSFE_API ObjLookup {
	//	std::unordered_map<std::string, scene::Object*> objMap;

	//	void incrementObjectID(scene::ObjectID& id) { id = static_cast<scene::ObjectID>(static_cast<std::uint32_t>(id) + 1); }
	//	void resetObjectID(scene::ObjectID& id) { id = scene::ObjectID::INVALID_OBJECT_ID; }

	//	void addObject(scene::Object* obj) { if (obj) { objMap[obj->name] = obj; } }
	//	void addObject(const std::string& name, scene::Object* obj) { if (obj) { objMap[name.c_str()] = obj; } }
	//	void removeObject(const std::string& name) { objMap.erase(name); }
	//	void clear() { objMap.clear(); }

	//	void updateObjectName(const std::string& oldName, const std::string& newName) {
	//		auto it = objMap.find(oldName);
	//		if (it != objMap.end()) {
	//			scene::Object* obj = it->second;
	//			objMap.erase(it);
	//			objMap[newName] = obj;
	//		}
	//	}

	//	scene::Object* getObjectByName(const std::string& name) {
	//		auto it = objMap.find(name);
	//		if (it != objMap.end()) { return it->second; }
	//		return nullptr;
	//	}

	//	scene::Object* getObjectByID(scene::ObjectID id) {
	//		for (const auto& pair : objMap) { if (pair.second && pair.second->id == id) { return pair.second; } }
	//		return nullptr;
	//	}

	//	std::string getNameByObject(scene::Object* obj) {
	//		for (const auto& pair : objMap) { if (pair.second == obj) { return pair.first; } }
	//		return "";
	//	}

	//	void logAllObjects() {
	//		for (const auto& pair : objMap) {
	//			if (pair.second) { D_INFO("Object Name: %s, ID: %u", pair.first.c_str(), static_cast<std::uint32_t>(pair.second->id)); }
	//		}
	//	}

	//	void logObjectCount() { D_INFO("Total Objects in ObjLookup: %zu", objMap.size()); }
	//};

	struct AssetSource {
		std::string filename;
		std::string filepath;
	};

	// Represents the position, rotation, and scale of an object in 3D space.
	struct Transform {
		glm::vec3 position{ 0.0f };
		glm::quat rotQ{ 1.0f, 0.0f, 0.0f, 0.0f };
		glm::vec3 scale{ 0.01f, 0.01f, 0.01f };

		Transform() : position(0.0f), rotQ{ 1.0f, 0.0f, 0.0f, 0.0f }, scale(0.01f, 0.01f, 0.01f) {}
		glm::mat4 toMatrix() const;
	};

	struct RenderMaterial {
		glm::vec3 albedo{ 1.0f, 1.0f, 1.0f };
		float metallic = 0.5f;
		float roughness = 0.5f;
	};

	// Represents a 3D object in the scene with a mesh, transform, and physics state.
	class Object : public Element {
	public:
		// Unique identifier for the object
		scene::ObjectID id = scene::ObjectID::INVALID_OBJECT_ID;
		std::string name;

		// 3D Transform
		Transform transform;
		physics::PhysicsState state;
		RenderMaterial material;

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

			state.q = mathlib::Quat(1.0, 0.0, 0.0, 0.0);
			state.linearVelocity = mathlib::Vec3::Zero();
			state.angularVelocity = mathlib::Vec3::Zero();
			state.forces = mathlib::Vec3::Zero();
			state.torques = mathlib::Vec3::Zero();
		}

		void onMouseWheel(double delta) { _distance += (float)delta * 0.5f; }
		void onMouseMove(double x, double y, eInputButton button);
		void setLastMousePos(const glm::vec2& pos) { _lastMousePos = pos; }
		glm::vec2 getLastMousePos() const { return _lastMousePos; }

		void setFromMatrix(const glm::mat4& mat) {
			transform.position = glm::vec3(mat[3]);
			transform.rotQ = glm::quat_cast(mat);
		}

		bool visible = true;      // rendered + editor-visible
		bool internal = false;    // not part of scene graph / editor

	private:
		std::shared_ptr<Mesh> _mesh;
		glm::vec2 _lastMousePos{ 0.0f };
		glm::vec3 albedo = glm::vec3(0.3f, 0.2f, 0.8f);
		float _distance = 5.0f;
	};
} // namespace scene