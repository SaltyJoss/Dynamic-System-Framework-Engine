#pragma once

#include "Elements/Camera.h"
#include "Elements/Mesh.h"
#include "Elements/Light.h"
#include "shader/shaderUtil.h"
#include "render/openglBufferManager.h"
#include "Elements/Input.h"

namespace gui {
    class SceneView
    {
    public:
        SceneView() :
            _camera(nullptr), _frameBuffer(nullptr), _shader(nullptr),
            _light(nullptr), _size(800, 600)
        {
            _frameBuffer = std::make_unique<render::OpenGLFrameBuffer>();
            _frameBuffer->createBuffers(1280, 720);
            _shader = std::make_unique<shaders::Shader>();
            _shader->load("shaders/vs.shader", "shaders/fs_pbr.shader");
            _light = std::make_unique<elements::Light>();
            _camera = std::make_unique<elements::Camera>(glm::vec3(0, 0, 3), 45.0f, 1.3f, 0.1f, 100.0f);

        }

        ~SceneView()
        {
            _shader->unload();
        }

        elements::Light* get_light() { return _light.get(); }

        void resize(int32_t width, int32_t height);
        void render();
        void loadMesh(const std::string& filepath);
        void setMesh(std::shared_ptr<elements::Mesh> mesh) { _mesh = mesh; }

        std::shared_ptr<elements::Mesh> getMesh() { return _mesh; }

        void onMouseMove(double x, double y, elements::eInputButton button);
        void onMouseWheel(double delta);
        void resetView() { _camera->reset(); }

    private:
        std::unique_ptr<elements::Camera> _camera;
        std::unique_ptr<render::OpenGLFrameBuffer> _frameBuffer;
        std::unique_ptr<shaders::Shader> _shader;
        std::unique_ptr<elements::Light> _light;
        std::shared_ptr<elements::Mesh> _mesh;
        glm::vec2 _size;
    };
}