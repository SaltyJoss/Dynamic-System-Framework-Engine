#pragma once

#include "EngineCore.h"
#include "Scene/Object.h"
#include "Rendering/ModelGroup.h"
#include "Scene/Input.h"
#include "FpsCounter.h"
#include "Robots/RobotModel.h"
#include "Platform/Logger.h"

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

extern ENGINE_API Debug gLog;

namespace render {
    class OpenGLFrameBuffer;
	class IBL;
    class SkyboxRenderer;
}
namespace shaders {
    class Shader;
}
namespace elements {
    class Light;
    class Camera;
    class Input;
    class Mesh;
    class Object;
}

namespace physics {
    class PhysicsSystem;
}

namespace gui {
    class ENGINE_API SceneView{
    public:
        SceneView();
        ~SceneView();

        // Light & Skybox
        elements::Light* getLight() { return _light.get(); }
		elements::Light* getSunLight() { return _sunLight.get(); }

        bool isSkyboxEnabled() const { return skyboxEnabled; }
        void setSkyboxEnabled(bool b) { skyboxEnabled = b; }

        void loadNewHDR(const std::string& path);


        // Background & Scene
        void setBackgroundColour(const glm::vec3& c) { _backgroundColour = c; }
        glm::vec3 getBackgroundColour() const { return _backgroundColour; }

        void setBackgroundAlpha(float a) { _backgroundAlpha = a; }
        float getBackgroundAlpha() const { return _backgroundAlpha; }

        float getPlaneHeight() const { return planeHeight; }


        // Control Modes & Camera
        enum class ControlMode {
            Camera,
            Object
        };

        ControlMode ctrlMode = ControlMode::Camera;

        void setControlMode(ControlMode mode) { ctrlMode = mode; }
        ControlMode getControlMode() const { return ctrlMode; }

        elements::Camera* getCamera();
        void resetView();

		void attachCameraToObject(elements::Object* obj);
        void detachCameraFromObject();

		// Mesh loading & Management
        void loadMesh(const std::string& filepath);
        std::vector<elements::Object*> loadMeshReturn(const std::string& filepath);
        void setMesh(std::shared_ptr<elements::Mesh> mesh) { _mesh = mesh; }

        std::shared_ptr<elements::Mesh> getMesh() { return _mesh; }


		// Rendering Entry Points
        void render();
        void resize(int32_t width, int32_t height);

		// Scene Objects Management
		std::vector<std::unique_ptr<elements::Object>>& getObjects() { return _objects; }
        elements::Object* getObject() { return _selectedObject; }
        void setSelectedObject(elements::Object* obj) { _selectedObject = obj; }

        void deleteObject(int index) {
            if (index < 0 || index >= _objects.size()) return;

            if (_selectedObject == _objects[index].get()) {
                _selectedObject = nullptr;
            }

            _objects.erase(_objects.begin() + index);
        }


        // Physics
        void updatePhysics(double dt);

		// Robotic Arm System
        void loadRobot(const std::string& name);
        bool hasRobot() const { return _hasRobot; }


		// Input Handling
        void processMovementKey(int key, float delta);
        void handleContinuousMovement(GLFWwindow* window, float dt);
        void handleMouseLook(GLFWwindow* window, double xpos, double ypos);

        void onMouseMove(double x, double y, elements::eInputButton button);
        void onMouseWheel(double delta);

        void resetMouseDelta();


    private:       
		// Rendering Pipeline Methods
        void MeshRender();
        void WorldGridRender();
        void InitShadowResource();
        void InitIBL();
        void ShadowPass();
        void SkyboxRender();
        glm::mat4 LightSpaceMatrix(float near, float far);


        // Core Rendering State
        std::unique_ptr<render::OpenGLFrameBuffer> _frameBuffer;
        std::unique_ptr<shaders::Shader> _shader;
        std::unique_ptr<shaders::Shader> _worldGridShader;
        std::unique_ptr<shaders::Shader> _shadowShader;


        // Scene Objects
        std::unique_ptr<elements::Camera> _camera;
        std::unique_ptr<elements::Light> _light;
        std::unique_ptr<elements::Light> _sunLight;
        elements::Object* _cameraFollowTarget = nullptr;

        std::shared_ptr<elements::Mesh> _mesh;
	    std::vector<std::unique_ptr<elements::Object>> _objects;
        std::vector<ModelGroup> modelGroups;
        elements::Object* _selectedObject = nullptr;

        std::shared_ptr<elements::Mesh> _checkerPlane;
        std::shared_ptr<elements::Mesh> createCheckerPlane(float size = 50.0f);


		// Environment & Lighting
        std::unique_ptr<render::IBL> _ibl;
        std::unique_ptr<render::SkyboxRenderer> _skybox;


		// Shadow Mapping
        static constexpr int NUM_CASCADES = 2;

        GLuint _cascadeFBO[NUM_CASCADES];
        GLuint _cascadeDepth[NUM_CASCADES];
        glm::mat4 _lightSpaceMatrixCascade[NUM_CASCADES];

        float _cascadeSplits[NUM_CASCADES] = { 0.1f, 0.3f };

        const unsigned int SHADOW_W = 8192;
        const unsigned int SHADOW_H = 8192;


		// Physics System
		std::unique_ptr<physics::PhysicsSystem> _physics;

        // Robotic Arm System
        RobotModel _robot;
        bool _hasRobot = false;
        std::unordered_map<std::string, int> _linkIndex;

        void instantiateRobotLinks();
        void buildLinkIndex();
        void updateRobotKinematics(const glm::mat4& baseTransform);
        void clearRobot();


        // Editor & UI
        gui::FpsCounter _fpsCounter;

        bool _isHovered = false;
        bool skyboxEnabled = true;
        bool _firstMouse = true;
        bool _firstUpdate = true;


        // Camera & Mouse
        glm::vec2 _lastMousePos{ 0.f, 0.f };

		// Misc Settings
        glm::vec2 _size;
        glm::vec3 _backgroundColour{ 1.0f, 1.0f, 1.0f };
        float _backgroundAlpha = 1.0f;

        static constexpr float planeHeight = -2.5f;
        float planeY = planeHeight;
        glm::vec3 planeNormal{ 0.0f, 1.0f, 0.0f };

        unsigned int _worldGridVAO = 0;
    };
}