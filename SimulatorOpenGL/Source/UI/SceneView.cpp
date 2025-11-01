#include "ch.h"

#include "SceneView.h"
#include <imgui.h>

namespace gui{
	void SceneView::render() {
		_frameBuffer->bind();

		_shader->use();

		_camera->update(_shader.get());
		_camera->setAspect(_size.x / _size.y);

		_light->update(_shader.get());

		// Render checker floor
		if (_checkerPlane) {
			glm::mat4 floorModel(1.0f);
			_shader->setMat4(floorModel, "model");

			// Set checkerboard uniforms
			_shader->setBool(true, "isFloor");
			_shader->setVec3(glm::vec3(1.0f), "colour1");
			_shader->setVec3(glm::vec3(0.0f), "colour2");
			_shader->setFlt1(1.0f, "checkSize");

			_checkerPlane->update(_shader.get());
			_checkerPlane->render();
		}

		// Render other objects normally
		if (_mesh) {
			glm::mat4 model = glm::translate(glm::mat4(1.0f), _mesh->_position);
			_shader->setMat4(model, "model");
			_shader->setBool(false, "isFloor");

			_mesh->update(_shader.get());
			_mesh->render();
		}

		_frameBuffer->unbind();

		ImGui::Begin("Simulation");

		ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
		_size = { viewportPanelSize.x, viewportPanelSize.y };
		uint64_t textureID = _frameBuffer->getTexture();
		ImGui::Image(reinterpret_cast<void*>(static_cast<uintptr_t>(_frameBuffer->getTexture())), ImVec2{ _size.x, _size.y }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

		_isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);

		ImGui::End();
	}

	void SceneView::resize(int32_t width, int32_t height) {
		_size.x = width;
		_size.y = height;

		_frameBuffer->deleteBuffers();
		_frameBuffer->createBuffers((int32_t)_size.x, (int32_t)_size.y);
		LOG_INFO("Framebuffer resized", (int32_t)_size.x, (int32_t)_size.y);
	}

	std::shared_ptr<elements::Mesh> gui::SceneView::createCheckerPlane(float size) {

		auto plane = std::make_shared<elements::Mesh>();

		std::vector<glm::vec3> pos = {
			{-size, planeHeight, -size},
			{ size, planeHeight, -size},
			{ size, planeHeight,  size},
			{-size, planeHeight,  size}
		};

		glm::vec3 normal(0.0f, 1.0f, 0.0f);

		for (auto& p : pos) {
			elements::VertexHolder vh(p, normal);
			plane->addVertex(vh);
		}

		plane->addVertexIndex(0);
		plane->addVertexIndex(1);
		plane->addVertexIndex(2);
		plane->addVertexIndex(2);
		plane->addVertexIndex(3);
		plane->addVertexIndex(0);

		plane->init();
		return plane;
	}

	void SceneView::onMouseWheel(double delta) { 
		if (!_isHovered) return;
		if (_camera) _camera->onMouseWheel(delta);
	}

	void SceneView::onMouseMove(double x, double y, elements::eInputButton button) { 
		if (!_isHovered) return;
		if (_camera) _camera->onMouseMove(x, y, button);
	}

	void SceneView::loadMesh(const std::string& filepath) {
		if (!_mesh) _mesh = std::make_shared<elements::Mesh>();
		else _mesh->clear(); // implement clear() to delete VAO/VBO etc.
		_mesh->load(filepath);
		LOG_INFO("Mesh loaded from %s", filepath.c_str());
	}
}