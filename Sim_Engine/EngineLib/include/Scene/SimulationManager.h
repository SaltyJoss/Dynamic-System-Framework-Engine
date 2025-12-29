#pragma once

//=============================================
//            File: simManager.h
//=============================================
// Class representing a 3D scene view with camera, lighting, and object management.
// 
// Summary:
// ============================================
// 
// structures & enumerations:
// --------------------------------------------
// enum class ControlMode
//      -> Enumeration for control modes (Camera or Object).
// enum class ShaderMode
// 	    -> Enumeration for shader modes (Basic, Lit, PBR).
// enum class robotFocusMode
//      -> Enumeration for robot focus modes (Base, Link, Joint).
// --------------------------------------------
// 
// public:
// --------------------------------------------
// simManager()
// 	    -> Constructor that initializes the simManager.
// ~simManager()
// 	    -> Destructor that cleans up resources.
// scene::Light* getLight()
//      -> Returns a pointer to the main light in the scene.
// scene::Light* getSunLight()
// 	    -> Returns a pointer to the sun light in the scene.
// bool isSkyboxEnabled()
//      -> Checks if the skybox is enabled.
// void setSkyboxEnabled(bool b)
//      -> Sets whether the skybox is enabled.
// void loadNewHDR(const std::string& path)
// 	    -> Loads a new HDR environment map from the specified path.
// void setBackgroundColour(const glm::vec3& c)
//      -> Sets the background colour of the scene.
// glm::vec3 getBackgroundColour()
// 	    -> Returns the background colour of the scene.
// void setBackgroundAlpha(float a)
//      -> Sets the background alpha (transparency) of the scene.
// float getBackgroundAlpha()
// 	    -> Returns the background alpha (transparency) of the scene.
// float getPlaneHeight()
//      -> Returns the height of the ground plane.
// void setControlMode(ControlMode mode)
//      -> Sets the current control mode.
// ControlMode getControlMode()
//  	-> Returns the current control mode.
// scene::Camera* getCamera()
// 	    -> Returns a pointer to the camera in the scene.
// void resetView()
//      -> Resets the camera view to the default position and orientation.
// void attachCameraToObject(scene::Object* obj)
//      -> Attaches the camera to follow the specified object.
// void detachCameraFromObject()
//      -> Detaches the camera from any object it is following.
// void loadMesh(const std::string& filepath)
//      -> Loads a mesh from the specified file path.
// std::vector<scene::Object*> loadMeshReturn(const std::string& filepath)
// 	    -> Loads a mesh and returns a vector of pointers to the created objects.
// void setMesh(std::shared_ptr<scene::Mesh> mesh)
//      -> Sets the current mesh for the scene.
// std::shared_ptr<scene::Mesh> getMesh()
// 	    -> Returns the current mesh for the scene.
// void reloadAllShaders()
//      -> Reloads all shaders used in the scene.
// void render()
//      -> Renders the scene.
// void resize(int32_t width, int32_t height)   
//      -> Resizes the scene view to the specified width and height.
// std::vector<std::unique_ptr<scene::Object>>& getObjects()
//      -> Returns a reference to the vector of scene objects.
// scene::Object* getObject()
// 	    -> Returns a pointer to the currently selected object.
// void setSelectedObject(scene::Object* obj)
// 	    -> Sets the currently selected object.
// void deleteObject(int index)
//      -> Deletes the object at the specified index.
// void updatePhysics(double dt)
//      -> Updates the physics simulation with the given time step.
// void loadRobot(const std::string& name)
//      -> Loads a robotic arm model by name.
// bool hasRobot()
// 	    -> Checks if a robotic arm model is currently loaded.
// void clearRobot()
//      -> Clears the currently loaded robotic arm model.
// RobotModel& getRobotModel()
// 	    -> Returns a reference to the robotic arm model.
// void processMovementKey(int key, float delta)
// 	    -> Processes a movement key input for camera control.
// void handleContinuousMovement(GLFWwindow* window, float dt)
//      -> Handles continuous movement input for the camera.
// void handleMouseLook(GLFWwindow* window, double xpos, double ypos)
//      -> Handles mouse look input for the camera.
// void onMouseMove(double x, double y, scene::eInputButton button)
//      -> Handles mouse movement input for the scene.
// void onMouseWheel(double delta)
//      -> Handles mouse wheel input for zooming.
// void resetMouseDelta()
//      -> Resets the mouse delta values.
// --------------------------------------------
// 
// private:
// --------------------------------------------
// void MeshRender()
//      -> Renders the mesh in the scene.
// void WorldGridRender()
//  	-> Renders the world grid in the scene.
// void InitShadowResource()
// 	    -> Initializes resources for shadow mapping.
// void InitIBL()
//      -> Initializes resources for image-based lighting (IBL).
// void ShadowPass()
// 	    -> Performs the shadow mapping pass.
// void SkyboxRender()
// 	    -> Renders the skybox in the scene.
// void instantiateRobotLinks()
// 	    -> Instantiates Object instances for each link in the robotic arm model.
// void buildLinkIndex()
// 	    -> Builds an index mapping link names to their corresponding Object instances.
// void updateRobotKinematics(const glm::mat4& baseTransform)
//      -> Updates the kinematics of the robotic arm model based on the base transform.
// 
// 
// Internal Pointer Variables:
// -----
// std::unique_ptr<render::OpenGLFrameBuffer> _frameBuffer
//      -> Unique pointer to the framebuffer used for rendering.
// std::shared_ptr<shaders::Shader> _shaderBasic
// 	    -> Shared pointer to the basic shader.
// std::shared_ptr<shaders::Shader> _shaderLit
//      -> Shared pointer to the lit shader.
// std::shared_ptr<shaders::Shader> _shaderPBR
// 	    -> Shared pointer to the PBR shader.
// std::unique_ptr<shaders::Shader> _worldGridShader
// 	    -> Unique pointer to the world grid shader.
// std::unique_ptr<shaders::Shader> _shadowShader
//      -> Unique pointer to the shadow mapping shader.
// std::unique_ptr<scene::Camera> _camera
//      -> Unique pointer to the camera in the scene.
// std::unique_ptr<scene::Light> _light
// 	    -> Unique pointer to the main light in the scene.
// std::unique_ptr<scene::Light> _sunLight
//      -> Unique pointer to the sun light in the scene.
// scene::Object* _cameraFollowTarget
// 	    -> Pointer to the object the camera is following (if any).
// std::unique_ptr<gui::AxisOrientator> _axisOrientator
//      -> Unique pointer to the axis orientator for the scene (general camera navigation aid).
// std::shared_ptr<scene::Mesh> _mesh
//      -> Shared pointer to the current mesh in the scene.
// std::vector<std::unique_ptr<scene::Object>> _objects
// 	    -> Vector of unique pointers to the scene objects.
// std::vector<ModelGroup> modelGroups
//      -> Vector of model groups for rendering.
// scene::Object* _selectedObject
// 	    -> Pointer to the currently selected object.
// std::shared_ptr<scene::Mesh> _checkerPlane
//      -> Shared pointer to the checkerboard ground plane mesh.
// std::shared_ptr<scene::Mesh> createCheckerPlane(float size)
//      -> Creates a checkerboard ground plane mesh of the specified size.
// render::SkyboxRenderer* _skybox
//      -> Pointer to the skybox renderer.
// std::unique_ptr<render::IBL> _ibl
//      -> Unique pointer to the image-based lighting 
// physics::PhysicsSystem* _physics
// 	    -> Pointer to the physics system for simulation.
// RobotModel _robot
//      -> Instance of the robotic arm model.
// std::unordered_map<std::string, int> _linkIndex
// 	    -> Map of link names to their corresponding indices in the robotic arm model.
// gui::FpsCounter _fpsCounter
//      -> FPS counter for performance monitoring.
// -----
// 
// Internal State Variables:
// -----
// glm::mat4 LightSpaceMatrix(float nearPlane, float farPlane)
// 	    -> Computes the light space matrix for shadow mapping.
// int _currentShadowIndex
// 	    -> Current index for shadow mapping cascades.
// int NUM_CASCADES
// 	    -> Number of cascades for shadow mapping.
// GLuint _cascadeFBO[NUM_CASCADES]
//      -> Array of OpenGL framebuffer IDs for cascade shadow maps.
// GLuint _cascadeDepth[NUM_CASCADES]
//      -> Array of OpenGL texture IDs for cascade shadow maps.
// glm::mat4 _lightSpaceMatrixCascade[NUM_CASCADES]
//      -> Array of light space matrices for each cascade.
// float _cascadeSplits[NUM_CASCADES]
// 	    -> Array of split distances for cascade shadow mapping.
// int SHADOW_W
//      -> Width of the shadow map textures.
// int SHADOW_H
// 	    -> Height of the shadow map textures.
// bool _hasRobot
//      -> Indicates whether a robotic arm model is currently loaded.
// bool _isHovered
//      -> Indicates whether the scene view is currently hovered by the mouse.
// bool skyboxEnabled
//      -> Indicates whether the skybox is enabled.
// bool _firstMouse
// 	    -> Indicates whether this is the first mouse input event.
// bool _firstUpdate
//      -> Indicates whether this is the first update call.
// glm::vec2 _lastMousePos
// 	    -> Last recorded mouse position for input handling.
// glm::vec2 _size 
//      -> Size of the scene view.
// glm::vec3 _backgroundColour
//      -> Background colour of the scene.
// float _backgroundAlpha
// 	    -> Background alpha (transparency) of the scene.
// float planeHeight
//      -> Height of the ground plane.
// float planeY
// 	    -> Y-coordinate of the ground plane.
// glm::vec3 planeNormal
//      -> Normal vector of the ground plane.
// int _worldGridVAO
// 	    -> OpenGL Vertex Array Object ID for the world grid.
// -----
// --------------------------------------------
//
// ============================================
//              GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include "Scene/Object.h"
#include "Rendering/ModelGroup.h"
#include "Scene/Input.h"
#include "FpsCounter.h"
#include "Robots/RobotModel.h"
#include "Platform/Logger.h"
#include "Scene/RenderPreset.h"

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

extern ENGINE_API Debug gLog;

// Forward Declarations
namespace render {
    class OpenGLFrameBuffer;
	class IBL;
    class SkyboxRenderer;
}
namespace shaders {
    class Shader;
}
namespace scene {
    class Light;
    class Camera;
    class Input;
    class Mesh;
    class Object;
}
namespace physics {
    class PhysicsSystem;
}

// I want to rename to more appropriate namespace later
namespace gui {
	class AxisOrientator;
	// simManager Class (Plan on renaming later)
    class ENGINE_API simManager {
    public:
        simManager();
        ~simManager();

        // Light & Skybox
        scene::Light* getLight() { return _light.get(); }
		scene::Light* getSunLight() { return _sunLight.get(); }

        bool isSkyboxEnabled() const { return skyboxEnabled; }
        void setSkyboxEnabled(bool b) { skyboxEnabled = b; }

        void loadNewHDR(const std::string& path);
        void loadNewHDR_UI(const std::string& path);
        void loadNewHDR_Preset(const std::string& path);

        // Background & Scene
        void setBackgroundColour(const glm::vec3& c) { _backgroundColour = c; }
        void setBackgroundAlpha(float a) { _backgroundAlpha = a; }

		std::string getDefaultHDR(render::LookPreset p) const;
        glm::vec3 getBackgroundColour() const { return _backgroundColour; }
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

        scene::Camera* getCamera();
        void resetView();

		void attachCameraToObject(scene::Object* obj);
        void detachCameraFromObject();

		// Mesh loading & Management
        void loadMesh(const std::string& filepath);
        std::vector<scene::Object*> loadMeshReturn(const std::string& filepath);
        void setMesh(std::shared_ptr<scene::Mesh> mesh) { _mesh = mesh; }

        std::shared_ptr<scene::Mesh> getMesh() { return _mesh; }

        enum class ShaderMode {
            Basic = 0,
            Lit = 1,
            PBR = 2
        };

        ShaderMode currentShaderMode = ShaderMode::Lit;  // default
        void applyRenderSettings(const render::RenderSettings& s);
        void applyRenderProfile(const render::RenderSettings& s, render::LookPreset l);
		void resetHDRToPreset();
        void reloadAllShaders();

		// Rendering Entry Points
        void render();
        void resize(int32_t width, int32_t height);

		// Scene Objects Management
		std::vector<std::unique_ptr<scene::Object>>& getObjects() { return _objects; }
        scene::Object* getObject() { return _selectedObject; }
        void setSelectedObject(scene::Object* obj) { _selectedObject = obj; }

        void deleteObject(int index);


        // Physics
        void updatePhysics(double dt);
		// Access to Physics System -> my attempt to fix the control panel integrtation method selector issue
		physics::PhysicsSystem& getPhysicsSystem() { return *_physics; } // mutable
		const physics::PhysicsSystem& getPhysicsSystem() const { return *_physics; } // const

		// Robotic Arm System
        void loadRobot(const std::string& name);
        bool hasRobot() const { return _hasRobot; }
		void setRobotLinkRotation(const std::string& linkName, float angle);
        void clearRobot();

        RobotModel& getRobotModel() { return _robot; }

		// Robot Focus Modes - NOT USED YET, KEEPING FOR IDEA I HAVE!
        enum class robotFocusMode {
            Base,
            Link,
            Joint
		};

		// Input Handling
        void processMovementKey(int key, float delta);
        void handleContinuousMovement(GLFWwindow* window, float dt);
        void handleMouseLook(GLFWwindow* window, double xpos, double ypos);
        void onMouseMove(double x, double y, scene::eInputButton button);
        void onMouseWheel(double delta);
        void resetMouseDelta();

    private:       
		// Rendering Pipeline Methods
        void MeshRender();
        void InitWorldGridVAO();
        void WorldGridRender();
        void InitShadowResource(int baseRes);
        void InitIBL();
        void ShadowPass();
        void SkyboxRender();
		void oreintationGizmoRender(); // Not really sure what to call this yet so GizmoRender for now!
        glm::mat4 LightSpaceMatrix(float near, float far);


        // Core Rendering State
        std::unique_ptr<render::OpenGLFrameBuffer> _frameBuffer;
        std::shared_ptr<shaders::Shader> _shaderBasic;
        std::shared_ptr<shaders::Shader> _shaderLit;
        std::shared_ptr<shaders::Shader> _shaderPBR;
        std::unique_ptr<shaders::Shader> _worldGridShader;
        std::unique_ptr<shaders::Shader> _shadowShader;


        // Scene Objects
        std::unique_ptr<scene::Camera> _camera;
        std::unique_ptr<scene::Light> _light;
        std::unique_ptr<scene::Light> _sunLight;
        scene::Object* _cameraFollowTarget = nullptr;
		std::unique_ptr<AxisOrientator> _axisOrientator;


        std::shared_ptr<scene::Mesh> _mesh;
	    std::vector<std::unique_ptr<scene::Object>> _objects;
        std::vector<ModelGroup> modelGroups;
        scene::Object* _selectedObject = nullptr;

        std::shared_ptr<scene::Mesh> _checkerPlane;
        std::shared_ptr<scene::Mesh> createCheckerPlane(float size = 50.0f);


		// Environment & Lighting
        render::RenderSettings _settingsCurrent{};
		render::LookPreset _lookCurrent = render::LookPreset::Studio;
        glm::vec3 _clearColour = glm::vec3(0.02f, 0.02f, 0.03f);
        bool _settingsValid = false;

        std::unique_ptr<render::IBL> _ibl;
        std::unique_ptr<render::SkyboxRenderer> _skybox;

        std::string _activeHDRPath;
		bool _hdrUserOverride = false;

        // current selection
        int _currentShaderIndex = 1; // 1 = lit by default


		// Shadow Mapping
        static constexpr int NUM_CASCADES = 2;

        bool _shadowsInit = false;
        GLuint _cascadeFBO[NUM_CASCADES]{};
        GLuint _cascadeDepth[NUM_CASCADES]{};
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
		float dt = 1.0f / 120.0f; // default to 120 fps

        static constexpr float planeHeight = -2.5f;
        float planeY = planeHeight;
        glm::vec3 planeNormal{ 0.0f, 1.0f, 0.0f };

        unsigned int _worldGridVAO = 0;
    };
}


// NOTES:
// NEED TO MANAGE SHADER RESOURCES (RELOAD ON DEMAND)
// NEED TO MANAGE MESH RESOURCES (RELOAD ON DEMAND)
// NEED TO MANAGE TEXTURE RESOURCES (RELOAD ON DEMAND)
// MAYBE A RESOURCE MANAGER CLASS TO HANDLE ALL OF THE ABOVE?
// MAYBE SPLIT SCENEVIEW INTO RENDERER AND SCENE MANAGER CLASSES?????? *Not sure though*
//
// TODO:
// - Look for decrepated methods and variables to clean up.
// - Consider splitting simManager into smaller, more focused classes if it becomes too large.
// - Explore my initial idea of central scene (efficitly this), then optional 4 sub-views for different camera angles (top, side, front, perspective), static in relation to the robotic arm (like CAD or modelling software), each allows either focus on the entire robot for all 4 angles, or focus on a specific link/joint for all 4 angles. This would be useful for debugging and visualizing the robot's configuration from multiple perspectives simultaneously (Plus user may need this for precise joint adjustments and understanding spatial relationships between links).
// 
// simManager name replacement ideas (Given its current main function of rendering and managing the simulations 3D scene):
// - SceneRenderer
// - SceneManager
// - SimulationView
// - SimulationRenderer
// - SimulationViewport
// - sim3DView
// - sim3DRenderer
// - sim3DManager
// - RenderManager3D
// - simSceneManager
// - simSceneRenderer
// * Some of these were generate with github copilot *
//
// END OF FILE