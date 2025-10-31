#include "ch.h"

#include "SceneView.h"
#include <imgui.h>

namespace gui{
	void SceneView::resize(int32_t width, int32_t height) {
		_size.x = width;
		_size.y = height;
	}

	void SceneView::onMouseMove(double x, double y, elements::eInputButton button) { _camera->onMouseMove(x, y, button); }
	void SceneView::onMouseWheel(double delta) { _camera->onMouseWheel(delta); }

	void SceneView::loadMesh(const std::string& filepath) {
		if (!_mesh) { _mesh = std::make_shared < elements::Mesh>(); }
		_mesh->load(filepath);
	}

	void SceneView::render() {
		_shader->use();
		_light->update(_shader.get());
		_frameBuffer->bind();

		if (_mesh) {
			_mesh->update(_shader.get());
			_mesh->render();
		}

		_frameBuffer->unbind();
		ImGui::Begin("Simulation");

		ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
		_size = { viewportPanelSize.x, viewportPanelSize.y };

		_camera->setAspect(_size.x / _size.y);
		_camera->update(_shader.get());

		uint64_t textureID = _frameBuffer->getTexture();
		ImGui::Image(reinterpret_cast<void*>(textureID), ImVec2{ _size.x, _size.y }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

		ImGui::End();
	}
}