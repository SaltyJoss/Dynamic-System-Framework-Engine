#include "ch.h"

#include "SceneView.h"
#include <imgui.h>

namespace gui{
	void SceneView::render() {
		_frameBuffer->bind();
		LOG_INFO("FrameBuffer bound");

		_shader->use();
		LOG_INFO("Shader used");

		_camera->update(_shader.get());
		_camera->setAspect(_size.x / _size.y);
		LOG_INFO("Camera updated");

		_light->update(_shader.get());
		LOG_INFO("Light updated");

		if (_checkerPlane) {
			glm::mat4 floorModel(1.0f);
			_shader->setMat4(floorModel, "model");
			_checkerPlane->update(_shader.get());
			_checkerPlane->render();
			LOG_INFO("CheckerPlane rendered");
		}


		if (_mesh) {
			glm::mat4 model(1.0f);
			_shader->setMat4(model, "model");
			_mesh->update(_shader.get());
			_mesh->render();
			LOG_INFO("Mesh rendered");
		}

		_frameBuffer->unbind();
		LOG_INFO("FrameBuffer unbound");

		ImGui::Begin("Scene");

		ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
		_size = { viewportPanelSize.x, viewportPanelSize.y };
		uint64_t textureID = _frameBuffer->getTexture();
		ImGui::Image(reinterpret_cast<void*>(static_cast<uintptr_t>(_frameBuffer->getTexture())), ImVec2{ _size.x, _size.y }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

		ImGui::End();
	}

	void SceneView::resize(int32_t width, int32_t height) {
		_size.x = width;
		_size.y = height;

		_frameBuffer->createBuffers((int32_t)_size.x, (int32_t)_size.y);
		LOG_INFO("Framebuffer resized", (int32_t)_size.x, (int32_t)_size.y);
	}

	std::shared_ptr<elements::Mesh> gui::SceneView::createCheckerPlane(float size) {
		LOG_INFO("createCheckerPlane() CALLED");

		auto plane = std::make_shared<elements::Mesh>();

		std::vector<glm::vec3> pos = {
			{-size, 0.0f, -size},
			{ size, 0.0f, -size},
			{ size, 0.0f,  size},
			{-size, 0.0f,  size}
		};

		glm::vec3 normal(0.0f, 1.0f, 0.0f);

		for (auto& p : pos) {
			elements::VertexHolder vh;
			vh._pos = p;
			vh._normal = normal;
			plane->addVertex(vh);
		}

		plane->addVertexIndex(0);
		plane->addVertexIndex(1);
		plane->addVertexIndex(2);
		plane->addVertexIndex(2);
		plane->addVertexIndex(3);
		plane->addVertexIndex(0);

		plane->init();
		LOG_INFO("createCheckerPlane() INITIALISED");
		return plane;
	}

	void SceneView::onMouseMove(double x, double y, elements::eInputButton button) { _camera->onMouseMove(x, y, button); }
	void SceneView::onMouseWheel(double delta) { _camera->onMouseWheel(delta); }

	void SceneView::loadMesh(const std::string& filepath) {
		if (!_mesh) { _mesh = std::make_shared<elements::Mesh>(); }
		_mesh->load(filepath);
		LOG_INFO("Mesh loaded from %s", filepath.c_str());
	}
}