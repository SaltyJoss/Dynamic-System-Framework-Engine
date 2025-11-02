#pragma once

#include "Elements/Camera.h"
#include "Elements/Mesh.h"
#include "Elements/Light.h"
#include "shader/shaderUtil.h"
#include "render/openglBufferManager.h"
#include "Elements/Input.h"

extern Debug gLog; // Global Variable for debugging and logs

namespace gui {
    class SceneView{
    public:
        SceneView() :
            _camera(nullptr), _frameBuffer(nullptr), _shader(nullptr),
            _light(nullptr), _size(1280, 720)
        {
            _frameBuffer = std::make_unique<render::OpenGLFrameBuffer>();
            _frameBuffer->createBuffers(1920, 1080);
            _shader = std::make_unique<shaders::Shader>();
            _shader->load("Source/shaders/vs.shader", "Source/shaders/fs_pbr.shader");
            _light = std::make_unique<elements::Light>();
            _camera = std::make_unique<elements::Camera>(glm::vec3(0, 0, 3), 45.0f, 1280.0f / 720.0f, 0.1f, 100.0f);

            if (_checkerPlane) _checkerPlane->clear();
            _checkerPlane = createCheckerPlane(50.0f);
        }

        ~SceneView() {
            _shader->unload();
            if (_frameBuffer) _frameBuffer->deleteBuffers();
            if (_mesh) _mesh->clear();
            if (_checkerPlane) _checkerPlane->clear();
        }

        elements::Light* getLight() { return _light.get(); }

        enum class ControlMode {
            Camera,
            Object
        };

        ControlMode _controlMode = ControlMode::Camera; // Default to Camera Control

        void resize(int32_t width, int32_t height);
        void render();
        void loadMesh(const std::string& filepath);
        void setMesh(std::shared_ptr<elements::Mesh> mesh) { _mesh = mesh; }
        void setControlMode(ControlMode mode) { _controlMode = mode; }
        ControlMode getControlMode() const { return _controlMode; }

        std::shared_ptr<elements::Mesh> getMesh() { return _mesh; }

        void onMouseMove(double x, double y, elements::eInputButton button);
        void onMouseWheel(double delta);
        void resetView() { _camera->reset(); }

    private:       
        std::unique_ptr<render::OpenGLFrameBuffer> _frameBuffer;
        std::unique_ptr<shaders::Shader> _shader;
        std::unique_ptr<elements::Light> _light;
        std::unique_ptr<elements::Camera> _camera;

        std::shared_ptr<elements::Mesh> _checkerPlane;
        std::shared_ptr<elements::Mesh> createCheckerPlane(float size = 50.0f);
        std::shared_ptr<elements::Mesh> _mesh;

        glm::vec2 _size;
        glm::vec2 _lastMousePos;

        float planeHeight = -2.5f;
        bool _isHovered = false;
    };
}