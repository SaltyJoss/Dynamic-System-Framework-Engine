#include "ch.h"

#include "SceneView.h"
#include <imgui.h>

namespace gui{
	void SceneView::resize(int32_t width, int32_t height) {
		_size.x = width;
		_size.y = height;

		_frameBuffer->createBuffers((int32_t)_size.x, (int32_t)_size.y);
	}

	void SceneView::onMouseMove(double x, double y, elements::eInputButton button) { _camera->onMouseMove(x, y, button); }
	void SceneView::onMouseWheel(double delta) { _camera->onMouseWheel(delta); }

	void SceneView::loadMesh(const std::string& filepath) {
		if (!_mesh) { _mesh = std::make_shared<elements::Mesh>(); }
		_mesh->load(filepath);
	}

	void SceneView::render() {
		_frameBuffer->bind();

		_shader->use();
		
		_camera->update(_shader.get());
		_camera->setAspect(_size.x / _size.y);

		_light->update(_shader.get());

		if (_mesh) {
			_mesh->update(_shader.get());

			_mesh->render();
		}

		_frameBuffer->unbind();

		ImGui::Begin("Scene");

		ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
		_size = { viewportPanelSize.x, viewportPanelSize.y };
		uint64_t textureID = _frameBuffer->getTexture();
		ImGui::Image(reinterpret_cast<void*>(static_cast<uintptr_t>(_frameBuffer->getTexture())), ImVec2{ _size.x, _size.y }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

		ImGui::End();
	}
}