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
            _Camera(nullptr), _FrameBuffer(nullptr), _Shader(nullptr),
            _Light(nullptr), _Size(800, 600)
        {
            _FrameBuffer = std::make_unique<render::OpenGLFrameBuffer>();
            _FrameBuffer->createBuffers(1280, 720);
            _Shader = std::make_unique<shaders::Shader>();
            _Shader->load("shaders/vs.shader", "shaders/fs_pbr.shader");
            _Light = std::make_unique<elements::Light>();
            _Camera = std::make_unique<elements::Camera>(glm::vec3(0, 0, 3), 45.0f, 1.3f, 0.1f, 100.0f);

        }

        ~SceneView()
        {
            _Shader->unload();
        }

        elements::Light* get_light() { return _Light.get(); }

        void resize(int32_t width, int32_t height);
        void render();
        void loadMesh(const std::string& filepath);
        void setMesh(std::shared_ptr<elements::Mesh> mesh) { _Mesh = mesh; }

        std::shared_ptr<elements::Mesh> get_mesh() { return _Mesh; }

        void onMouseMove(double x, double y, elements::eInputButton button);
        void onMouseWheel(double delta);
        void resetView() { _Camera->reset(); }

    private:
        std::unique_ptr<elements::Camera> _Camera;
        std::unique_ptr<render::OpenGLFrameBuffer> _FrameBuffer;
        std::unique_ptr<shaders::Shader> _Shader;
        std::unique_ptr<elements::Light> _Light;
        std::shared_ptr<elements::Mesh> _Mesh;
        glm::vec2 _Size;
    };
}