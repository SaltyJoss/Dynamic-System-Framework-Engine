#pragma once
// File:   MeshLoader.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

#include "Rendering/ModelGroup.h"
#include "Scene/ObjectID.h"
#include "ui/RenderPreset.h"
#include "FpsCounter.h"

#include "Analysis/Telemetry.h"

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

namespace interpreter { class ENGINE_API IStoredProgram; }
namespace physics { class ENGINE_API PhysicsSystem; }
namespace robots { class ENGINE_API RobotSystem; }
namespace control { class ENGINE_API TrajectoryManager; }

// I want to rename to more appropriate namespace later
namespace gui {
    // View IDs
    enum class ViewID { Manual = 0, Top, Right, Front, Follow, COUNT };

	class ENGINE_API AxisOrientator;
	// SimManager Class (Plan on renaming later)
    class ENGINE_API SimManager {
    public:
        SimManager();
        ~SimManager();

		// OpenGL Initialisation
        void initGL();

        // Light & Skybox
        scene::Light* getLight();
        void setLightColour(const glm::vec3& c);

        bool isSkyboxEnabled() const { return skyboxEnabled; }
        void setSkyboxEnabled(bool b) { skyboxEnabled = b; }

        void loadNewHDR(const std::string& path);
        void loadNewHDR_UI(const std::string& path);
        void loadNewHDR_Preset(const std::string& path);

        // Background & Scene
        void setInternalSize(const glm::vec2& size) { _internalSize = size; }
        glm::vec2 getInternalSize() const { return _internalSize; }

        void setDisplaySize(const glm::vec2& size) { _displaySize = size; }
        glm::vec2 getDisplaySize() const { return _displaySize; }

        void setSize(const glm::vec2& size) { setInternalSize(size); }
        glm::vec2 getSize() const { return getInternalSize(); }

        void setBackgroundColour(const glm::vec3& c) { _backgroundColour = c; }
        void setBackgroundAlpha(float a) { _backgroundAlpha = a; }

		std::string getDefaultHDR() const;
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

        void setViewFollowTarget(ViewID view, scene::Object* obj, const glm::vec3& offset = glm::vec3(0.0f, 0.25f, 1.0f));
        void clearViewFollowTarget(ViewID view);

        // Follow a robot joint by name (binds the view to that joint's child link object)
        bool setViewFollowRobotJoint(ViewID view, const std::string& jointName, const glm::vec3& offset);

        // Convenience: follow in the Follow view
        bool followRobotJoint(const std::string& jointName, const glm::vec3& offset = glm::vec3(0.0f, 0.2f, 0.6f));

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

        shaders::Shader* getCurrentShader() const;
		void setCurrentShader(shaders::Shader* shader);

        ShaderMode currentShaderMode = ShaderMode::PBR;  // default
        void applyRenderSettings(const render::RenderSettings& s, render::ResolutionPreset r);
        void applyRenderProfile(const render::RenderSettings& s, render::ResolutionPreset r);
        //void rebuildRenderTargets();
		void resetHDRToPreset();
        void reloadAllShaders();

		// Rendering Entry Points
        void render();
        void resize(int32_t width, int32_t height);

		// Scene Objects Management
        void setSelectedObject(scene::Object* obj);
        void addObject(std::unique_ptr<scene::Object> obj);
        void deleteObject(int index);
		void removeObject(scene::Object* obj);

		// Scene Objects Lookup
        std::vector<std::unique_ptr<scene::Object>>& getObjects();
        scene::Object* getObject();
		scene::Object* getObjectByID(scene::ObjectID id);

        // Physics
        void updatePhysics(double dt);
        void tick(double frame_dt);
		void stepFixed(double frame_dt);

		// Access to Physics System -> my attempt to fix the control panel integrtation method selector issue
        physics::PhysicsSystem& getPhysicsSystem();
        const physics::PhysicsSystem& getPhysicsSystem() const;

		// Robot System
        void loadRobot(const std::string& name);
        void setRobotLinkRotation(const std::string& linkName, double angle);
        void setRobotRootPose(const glm::vec3& pos, const glm::quat& rot);
        void setRobotRootHome(const glm::vec3& pos, const glm::quat& rot);
        void resetRobot();
        void clearRobot();
        bool hasRobot() const;

        robots::RobotSystem* getRobotSystem();
        const robots::RobotSystem* getRobotSystem() const;

        control::TrajectoryManager& traj();
        const control::TrajectoryManager& traj() const;

		// Input Handling
        void processMovementKey(int key, float delta);
        void handleContinuousMovement(GLFWwindow* window, float dt);
        void handleMouseLook(GLFWwindow* window, double xpos, double ypos);
        void onMouseMove(double x, double y, scene::eInputButton button);
        void onMouseWheel(double delta);
        void resetMouseDelta();
        
		// Simulation Control
		double getFixedDeltaTime() const { return _dt; }
		void setFixedDeltaTime(double dt) { _dt = dt; }

		bool isSimRunning() const { return _simRunning; }
        void startSimulation();
        void stopSimulation();

		double getSimTime() const { return _simTime; }
		void setSimTime(double t) { _simTime = t; }
		void incrementSimTime(double dt) { _simTime += dt; }

		bool isScriptRunning() const { return _scriptRunning; }
		void setScriptRunning(bool running) { _scriptRunning = running; }

        void setActiveProgram(interpreter::IStoredProgram* p) { _activeProgram = p; }
        interpreter::IStoredProgram* activeProgram() const { return _activeProgram; }

		// Telemetry
        diagnostics::TelemetryRecorder& telemetry() { return _telemetry; }
		const diagnostics::TelemetryRecorder& telemetry() const { return _telemetry; }

    private:       
		// Rendering Pipeline Methods
        void MeshRender(scene::Camera* cam);
        void WorldGridRender(scene::Camera* cam);
        void InitShadowResource(int baseRes);
        void InitIBL();
        void SkyboxRender(scene::Camera* cam);
        void ShadowPass(scene::Camera* cam);
        glm::mat4 LightSpaceMatrix(scene::Camera* cam, float nearPlane, float farPlane);
        glm::vec2 getPresetResolutionPx() const;
		glm::vec2 getInternalResolutionSizePx() const;

        void drawMainDockspace();
        void drawViewportWindow();
        //void drawSceneWindow();
        //void drawInspectorWindow();

        void beginSimManager(const char* id);
		void endSimManager();

        // Misc Settings
        bool _glReady = false;
        bool _scriptRunning = false;

        glm::vec2 _internalSize = { 1920.0f, 1080.0f };
		glm::vec2 _displaySize = { 1920.0f, 1080.0f };
        glm::vec3 _backgroundColour{ 1.0f, 1.0f, 1.0f };

		float _displayW = 0.0f, _displayH = 0.0f;
		float _gridInternalScale = 1.0f;
        float _backgroundAlpha = 1.0f;

        double _dt = 1.0f / 180.0f;
        double _fixedDt = 1.0f / 180.0f;
		double _accum = 0.0;   // Accumulator for fixed timestep
		double _simTime = 0.0; // Current simulation time
		bool _simRunning = false;

        static constexpr float planeHeight = -2.5f;
        float planeY = 2.5f;
        glm::vec3 planeNormal{ 0.0f, 1.0f, 0.0f };

		// Rendering Pipeline Resources
        struct Impl;
		std::unique_ptr<Impl> _impl;

		// Objects & Scene Management
        std::vector<ModelGroup> modelGroups;
		scene::ObjectID _nextObjectID = scene::FIRST_VALID_OBJECT_ID; // Next available ObjectID
        
		// Name and ID mapping (for easy lookup)
        std::unordered_map<std::string, scene::ObjectID> _nameToId;
        std::unordered_map<scene::ObjectID, scene::Object*> _idToPtr;

		// Active Script Program
        interpreter::IStoredProgram* _activeProgram = nullptr;

		// Telemetry
		diagnostics::TelemetryRecorder _telemetry; // Dynamic telemetry recorder
        bool _telemetryBegun = false;

		// Environment & Lighting
        render::RenderSettings _settingsCurrent{};
		render::ResolutionPreset _resCurrent = render::ResolutionPreset::R_1080p;
        glm::vec3 _clearColour = glm::vec3(0.02f, 0.02f, 0.03f);
        std::string _activeHDRPath;
        static constexpr int NUM_CASCADES = 2;
        float _cascadeSplits[NUM_CASCADES] = { 0.1f, 0.3f };
        const unsigned int SHADOW_W = 8192;
        const unsigned int SHADOW_H = 8192;
        int _currentShaderIndex = 2; // 2 = PBR by default (atm)

        bool _settingsValid = false;
        bool _shadowsInit = false;
        bool _hdrUserOverride = false;

        // Editor & UI
        gui::FpsCounter _fpsCounter;

        bool _isHovered = false;
        bool skyboxEnabled = true;
        bool _firstMouse = true;
        bool _firstUpdate = true;

        // Camera & Mouse
        glm::vec2 _lastMousePos{ 0.f, 0.f };
    };
} // namespace gui