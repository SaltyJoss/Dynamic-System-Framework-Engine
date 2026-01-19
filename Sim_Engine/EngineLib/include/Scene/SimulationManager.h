#pragma once

//=============================================
//            File: simManager.h
//=============================================
// Class representing a 3D scene view with camera, lighting, and object management.
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
// ============================================
//              GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

#include "Scene/ObjectID.h"
#include "Rendering/ModelGroup.h"
#include "Scene/RenderPreset.h"
#include "FpsCounter.h"

#include "Platform/Logger.h"

// Forward Declarations
namespace render {
    class ENGINE_API OpenGLFrameBuffer;
	class ENGINE_API IBL;
    class SkyboxRenderer;
}
namespace shaders { class ENGINE_API Shader; }

namespace scene {
    enum class eInputButton;
    class ENGINE_API Light;
    class ENGINE_API Camera;
    class ENGINE_API Input;
    class ENGINE_API Mesh;
    class ENGINE_API Object;
}

namespace physics { class PhysicsSystem; }
namespace robots { class ENGINE_API RobotSystem; }

// I want to rename to more appropriate namespace later
namespace gui {
	class AxisOrientator;
	// simManager Class (Plan on renaming later)
    class ENGINE_API simManager {
    public:
        simManager();
        ~simManager();

		// OpenGL Initialisation
        void initGL();

        // Light & Skybox
        scene::Light* getLight();
        scene::Light* getSunLight();
        void setLightColour(const glm::vec3& c);

        bool isSkyboxEnabled() const { return skyboxEnabled; }
        void setSkyboxEnabled(bool b) { skyboxEnabled = b; }

        void loadNewHDR(const std::string& path);
        void loadNewHDR_UI(const std::string& path);
        void loadNewHDR_Preset(const std::string& path);

        // Background & Scene
		void setSize(const glm::vec2& size) { _size = size; }
		glm::vec2 getSize() const { return _size; }
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
        void setMesh(std::shared_ptr<scene::Mesh> mesh);
        std::shared_ptr<scene::Mesh> getMesh();

        enum class ShaderMode {
            Basic = 0,
            Lit = 1,
            PBR = 2
        };

        shaders::Shader* getActiveShader() const;
        ShaderMode currentShaderMode = ShaderMode::Lit;  // default
        void applyRenderSettings(const render::RenderSettings& s);
        void applyRenderProfile(const render::RenderSettings& s, render::LookPreset l);
        void rebuildRenderTargets();
		void resetHDRToPreset();
        void reloadAllShaders();

		// Rendering Entry Points
        void render();
        void resize(int32_t width, int32_t height);

		// Scene Objects Management
        void setSelectedObject(scene::Object* obj);
        void addObject(std::unique_ptr<scene::Object> obj);
        void deleteObject(int index);

		// Scene Objects Lookup
        std::vector<std::unique_ptr<scene::Object>>& getObjects();
        scene::Object* getObject();
		scene::Object* getObjectByID(scene::ObjectID id);
		scene::Object* getObjectByName(const std::string& name);

        // Physics
        void updatePhysics(double dt);
		// Access to Physics System -> my attempt to fix the control panel integrtation method selector issue
        physics::PhysicsSystem& getPhysicsSystem();
        const physics::PhysicsSystem& getPhysicsSystem() const;

		// Robot System
        void loadRobot(const std::string& name);
        void setRobotLinkRotation(const std::string& linkName, float angle);
        void setRobotRootPose(const glm::vec3& pos, const glm::quat& rot);
        void setRobotRootHome(const glm::vec3& pos, const glm::quat& rot);
        void resetRobot();
        void clearRobot();
        bool hasRobot() const;

        robots::RobotSystem* getRobotSystem();
        const robots::RobotSystem* getRobotSystem() const;

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
        void WorldGridRender();
        void InitShadowResource(int baseRes);
        void InitIBL();
        void ShadowPass();
        void SkyboxRender();
		void oreintationGizmoRender(); // Not really sure what to call this yet so GizmoRender for now!
        glm::mat4 LightSpaceMatrix(float near, float far);

        // Misc Settings
        bool _glReady = false;

        glm::vec2 _size;
        glm::vec3 _backgroundColour{ 1.0f, 1.0f, 1.0f };
        float _backgroundAlpha = 1.0f;
        float dt = 1.0f / 120.0f; // default to 120 fps

        static constexpr float planeHeight = -2.5f;
        float planeY = 2.5f;
        glm::vec3 planeNormal{ 0.0f, 1.0f, 0.0f };

		// Members
        struct Impl;
		std::unique_ptr<Impl> _impl;

		// Objects & Scene Management
        std::vector<ModelGroup> modelGroups;
		scene::ObjectID _nextObjectID = scene::FIRST_VALID_OBJECT_ID; // Next available ObjectID
        
		// Name and ID mapping (for easy lookup)
        std::unordered_map<std::string, scene::ObjectID> _nameToId;
        std::unordered_map<scene::ObjectID, scene::Object*> _idToPtr;

		// Environment & Lighting
        render::RenderSettings _settingsCurrent{};
		render::LookPreset _lookCurrent = render::LookPreset::Studio;
        glm::vec3 _clearColour = glm::vec3(0.02f, 0.02f, 0.03f);
        bool _settingsValid = false;

        std::string _activeHDRPath;
		bool _hdrUserOverride = false;

        // current selection
        int _currentShaderIndex = 1; // 1 = lit by default

		// Shadow Mapping
        static constexpr int NUM_CASCADES = 2;

        bool _shadowsInit = false;

        float _cascadeSplits[NUM_CASCADES] = { 0.1f, 0.3f };

        const unsigned int SHADOW_W = 8192;
        const unsigned int SHADOW_H = 8192;

        // Editor & UI
        gui::FpsCounter _fpsCounter;

        bool _isHovered = false;
        bool skyboxEnabled = true;
        bool _firstMouse = true;
        bool _firstUpdate = true;

        // Camera & Mouse
        glm::vec2 _lastMousePos{ 0.f, 0.f };
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