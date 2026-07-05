// DSFE_GUI SimObjects.cpp
#include "Scene/SimulationManager.h"
#include "Manager/SimImplementation.h"
#include "Platform/ScopeGLContext.h"

namespace gui {
	// Helper to get the next ObjectID
	static inline scene::ObjectID next(scene::ObjectID id) {
		return static_cast<scene::ObjectID>(static_cast<std::uint32_t>(id) + 1);
	}

	// Load a mesh from file and create one Object per submesh. The last loaded mesh becomes the active selection.
	void SimManager::loadMesh(const std::string& filepath) {
		platform::ScopeGLContext guard(_makeCurrentHook, _doneCurrentHook);

		assets::MeshLoader loader;
		auto meshes = loader.load(filepath);

		if (meshes.empty()) {
			LOG_WARN("No meshes imported from %s", filepath.c_str());
			D_WARN("No meshes imported from %s", filepath.c_str());
			return;
		}

		LOG_INFO("4. Creating Objects for each submesh from %s", filepath.c_str());
		// For now: spawn one Object per submesh
		for (auto& m : meshes) {
			auto obj = std::make_unique<scene::Object>(m);
			obj->id = next(_nextObjectID);
			obj->source.filename = filepath;
			obj->name = m->getName().empty() ? "Object_" + std::to_string(scene::toUInt32(obj->id)) : m->getName();

			// initialise physics state
			obj->state.q = Quat(1.0, 0.0, 0.0, 0.0);
			obj->state.angularVelocity = Vec3::Zero();
			obj->state.linearVelocity = Vec3::Zero();
			obj->state.mass = 1.0;
			obj->state.damping = 0.0;
			obj->state.inertia = Mat3::Identity();
			obj->state.forces = Vec3::Zero();
			obj->state.torques = Vec3::Zero();

			_impl->_selectedObject = obj.get();
			_impl->_objects.push_back(std::move(obj));
		}

		LOG_INFO("Loaded %zu submeshes from %s", meshes.size(), filepath.c_str());
		D_INFO("Loaded %zu submeshes from %s", meshes.size(), filepath.c_str());
	}

	// Returns the loaded objects so they can be used as targets for robot joints in the same frame (e.g. end-effector)
	std::vector<scene::Object*> SimManager::loadMeshReturn(const std::string& filepath) {
		assets::MeshLoader loader;
		auto meshes = loader.load(filepath);
		std::vector<scene::Object*> result;
		for (auto& m : meshes) {
			auto obj = std::make_unique<scene::Object>(m);
			auto raw = obj.get();
			raw->internal = true;
			_impl->_objects.push_back(std::move(obj));
			result.push_back(raw);
		}
		return result;
	}

	// Setter and Getter for the active mesh
	void SimManager::setMesh(std::shared_ptr<scene::Mesh> mesh) { _impl->_mesh = mesh; }
	std::shared_ptr<scene::Mesh> SimManager::getMesh() { return _impl->_mesh; }

	// Set the currently selected object (can be nullptr to deselect)
	void SimManager::setSelectedObject(scene::Object* obj) { _impl->_selectedObject = obj; }
	// Add a new object to the scene and select it
	void SimManager::addObject(std::unique_ptr<scene::Object> obj) { _impl->_objects.push_back(std::move(obj)); } // Cache the unique_ptr

	// Remove an object by index
	void SimManager::deleteObject(int index) {
		if (index < 0 || index >= _impl->_objects.size()) { return; }
		if (_impl->_selectedObject == _impl->_objects[index].get()) { _impl->_selectedObject = nullptr; }
		_impl->_objects.erase(_impl->_objects.begin() + index);
	}

	// Remove an object by pointer
	void SimManager::removeObject(scene::Object* obj) {
		if (!obj) return;

		auto it = std::remove_if(
			_impl->_objects.begin(),
			_impl->_objects.end(),
			[obj](const std::unique_ptr<scene::Object>& o) {
			return o.get() == obj;
		}
		);

		_impl->_objects.erase(it, _impl->_objects.end());
	}

	// Access the objects as raw pointers for use in the rest of the codebase, while maintaining ownership in SimManager
	std::vector<std::unique_ptr<scene::Object>>& SimManager::getObjects() { return _impl->_objects; }
	// Get the currently selected object (can be nullptr)
	scene::Object* SimManager::getObject() { return _impl->_selectedObject; }

	// Helper to find an object by its ID (returns nullptr if not found)
	scene::Object* SimManager::getObjectByID(scene::ObjectID id) {
		for (auto& obj : _impl->_objects) {
			if (obj && obj->id == id) {
				return obj.get();
			}
		}
		return nullptr;
	}
}