#pragma once
// File:   SimManager.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include "Platform/StudyRunner.h"

#include "Rendering/ModelGroup.h"
#include "Scene/ObjectID.h"
#include "ui/RenderPreset.h"
#include "FpsCounter.h"

#include "Analysis/Telemetry.h"
#include "Analysis/MetricLogger.h"

#include "Platform/Logger.h"

// Forward Declarations for Rendering
namespace render {
    class ENGINE_API OpenGLFrameBuffer;
	class ENGINE_API IBL;
    class SkyboxRenderer;
}
namespace shaders { class ENGINE_API Shader; }

// Forward Declarations for Scene
namespace scene {
    enum class eInputButton;
    class ENGINE_API Light;
    class ENGINE_API Camera;
    class ENGINE_API Input;
    class ENGINE_API Mesh;
    class ENGINE_API Object;
    class ENGINE_API SceneRenderer;
}

// Forward Declarations for Simulation Core
namespace core { class ENGINE_API SimulationCore; }

// Forward Declarations for Physics, Robots, Control, and Integration
namespace interpreter { class ENGINE_API IStoredProgram; }
namespace physics { class ENGINE_API PhysicsSystem; }
namespace robots { class ENGINE_API RobotSystem; }
namespace control { class ENGINE_API TrajectoryManager; }
namespace integration { enum class eIntegrationMethod; }

namespace gui {
    // View IDs
    enum class ViewID { Manual = 0, Top, Right, Front, Follow, COUNT };

	// Forward Declarations for Axis Orientator
	class ENGINE_API AxisOrientator;

    // Control Modes & Camera
    enum class ControlMode {
        Camera,
        Object
    };

	// SimManager Class (Plan on renaming later)
    class ENGINE_API SimManager {
    public:
		// Constructor & Destructor
        SimManager();
        ~SimManager();

		// OpenGL Initialisation
        void initGL();

		// Light
        scene::Light* getLight();
        void setLightColour(const glm::vec3& c);

		// Skybox & IBL
        void setSkyboxEnabled(bool b) { skyboxEnabled = b; }
        const bool isSkyboxEnabled() const { return skyboxEnabled; }

		// HDR Environment Maps
        void loadNewHDR(const std::string& path);
        void loadNewHDR_UI(const std::string& path);
        void loadNewHDR_Preset(const std::string& path);

        // Background & Scene
        void setInternalSize(const glm::vec2& size) { _internalSize = size; }
        glm::vec2 internalSize() const { return _internalSize; }

		// Setter and getter for display size (used for post-processing and final output)
        void setDisplaySize(const glm::vec2& size) { _displaySize = size; }
        glm::vec2 displaySize() const { return _displaySize; }

		// Setter and getter for the "logical" scene size (used for camera projection and physics scaling)
        void setSize(const glm::vec2& size) { setInternalSize(size); }
        glm::vec2 size() const { return internalSize(); }

		// Setter and getter for background colour
        void setBackgroundColour(const glm::vec3& c) { _backgroundColour = c; }
        const glm::vec3 backgroundColour() const { return _backgroundColour; }

		// Setter and getter for background alpha
        void setBackgroundAlpha(float a) { _backgroundAlpha = a; }
        float backgroundAlpha() const { return _backgroundAlpha; }

		// Getter for the default HDR path
		std::string getDefaultHDR() const;
		// Getter plane height (y=0 plane for physics and object placement)
        float getPlaneHeight() const { return planeHeight; }

        ControlMode ctrlMode = ControlMode::Camera;

		// Setter and getter for control mode
        void setControlMode(ControlMode mode) { ctrlMode = mode; }
        ControlMode controlMode() const { return ctrlMode; }

		// Get the currently active camera (based on active view)
        scene::Camera* getCamera();

		// Reset the active view camera to default position
        void resetView();

		// Attach and detach the active view camera to an object (camera will follow the object's position and rotation)
        void attachCameraToObject(scene::Object* obj);
        void detachCameraFromObject();

		// Follow a specific object in the Follow view (binds the view to that object's transform)
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
        
		// Shader Management
        enum class ShaderMode {
            Basic = 0,
            Lit = 1,
            PBR = 2
        };
        ShaderMode currentShaderMode = ShaderMode::PBR;  // default

		// Setter and getter for the current shader
        const shaders::Shader* getCurrentShader() const;

		// Render Settings & Profiles
        void applyRenderSettings(const render::RenderSettings& s, render::ResolutionPreset r);
        void applyRenderProfile(const render::RenderSettings& s, render::ResolutionPreset r);

		// Reset HDR to default
		void resetHDRToPreset();
		// Reload shaders (e.g. after editing source files)
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

        void updatePhysics(double dt);

		// Robot System loading and management
        void loadRobot(const std::string& name);
        void resetRobot();
        void clearRobot();
        const bool hasRobot() const;

		// Setters for robot joint states (angle in radians)
        void setRobotLinkRotation(const std::string& linkName, double angle);
        void setRobotRootPose(const glm::vec3& pos, const glm::quat& rot);
        void setRobotRootHome(const glm::vec3& pos, const glm::quat& rot);

        // Access to Physics System (non-const and const versions)
        physics::PhysicsSystem* physicsSystem();
        const physics::PhysicsSystem* physicsSystem() const;

		// Accesors for the robot system (non-const and const versions)
        robots::RobotSystem* robotSystem();
        const robots::RobotSystem* robotSystem() const;

		// Accessors for the trajectory manager (non-const and const versions)
        control::TrajectoryManager* traj();
        const control::TrajectoryManager* traj() const;

        // Simulation Control
        void startSimulation();
        void stopSimulation();

		// Simulation State
        const bool isSimRunning() const;

        // Setter and getter for current simulation time (seconds)
        void setSimTime(double t);
        const double simTime() const;

        // Setter and getter for fixed timestep (seconds)
        void setFixedDt(double dt);
        const double fixedDt() const;

        // Setter and getter for telemetry frequency (Hz)
        void setTelemetryHz(double hz);
        const double telemetryHz() const;

		// Setters for script and simulation running states
        void setScriptRunning(bool running);
        const bool isScriptRunning() const;

		// Setters and getters for last script text
		void setLastScriptText(const std::string& text);
		const std::string& lastScriptText() const;

		// Accessors for the last script text
        void setActiveProgram(interpreter::IStoredProgram* program);
		interpreter::IStoredProgram* activeProgram();
        const interpreter::IStoredProgram* activeProgram() const;
        
        // Run a script to completion synchronously with a specific integrator
        bool runScriptToCompletion(const std::string& scriptText, integration::eIntegrationMethod method);

        // Accesors for the telemetry recorder (non-const and const versions)
        diagnostics::TelemetryRecorder& telemetry();
        const diagnostics::TelemetryRecorder& telemetry() const;

		// Accessors for the Simulation Core interface (non-const and const versions)
        core::ISimulationCore* simCoreInterface();
        const core::ISimulationCore* simCoreInterface() const;

        // Accesor for Simulation Core (non-const and const versions)
		core::SimulationCore* simCore();
		const core::SimulationCore* simCore() const;

		// Access to the underlying StudyRunner for running batch studies from the GUI
		StudyRunner* studyRunner() { return _studyRunner.get(); }

		// Methods for handling completed studies from the background worker
        void pushCompletedStudies(std::vector<StudyResult> results);
		void pushCompletedStudy(StudyResult result);
        bool hasCompletedStudy() const;
        std::vector<StudyResult> consumeCompletedStudy();

        // Input Handling
        void processMovementKey(int key, float delta);
        void handleContinuousMovement(GLFWwindow* window, float dt);
        void handleMouseLook(GLFWwindow* window, double xpos, double ypos);
        void onMouseMove(double x, double y, scene::eInputButton button);
        void onMouseWheel(double delta);
        void resetMouseDelta();

    private:
        std::unique_ptr<core::SimulationCore> _core = nullptr;
		std::unique_ptr<StudyRunner> _studyRunner = nullptr; // Background worker for running batch studies
		bool _hasCompletedStudy = false;

		// Rendering Pipeline Methods
        void MeshRender(scene::Camera* cam);
        void WorldGridRender(scene::Camera* cam, int rtW);
        void InitShadowResource(int baseRes);
        void InitIBL();
        void SkyboxRender(scene::Camera* cam);
        void ShadowPass(scene::Camera* cam);
        glm::mat4 LightSpaceMatrix(scene::Camera* cam, float nearPlane, float farPlane);
        glm::vec2 getPresetResolutionPx() const;
		glm::vec2 getInternalResolutionSizePx() const;

        void drawMainDockspace();
        void drawViewportWindow();

        // Simulation Management
        void beginSimManager(const char* id);
		void endSimManager();

        // Misc Settings
        bool _glReady = false;

		// Sizes & Display
		glm::vec2 _internalSize{ 1920.0f, 1080.0f };  // Internal render target size
		glm::vec2 _displaySize{ 1920.0f, 1080.0f };   // Actual display size
		glm::vec3 _backgroundColour{ 1.0f, 1.0f, 1.0f }; // Background colour (default white, but can be changed by user)

		// Cached display size for scaling calculations (updated on resize)
		float _displayW = 0.0f, _displayH = 0.0f;
		float _gridInternalScale = 1.0f;
        float _backgroundAlpha = 1.0f;

        // Last script text for comparison re-use
        std::string _lastScriptText;

		// Ground Plane
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

		// Telemetry
		diagnostics::TelemetryRecorder _telemetry; // Dynamic telemetry recorder
		robots::JointLogBuffer _jointLogBuffer;    // Buffer for logging joint data each step
		robots::TrajRefBuffer _trajRefBuffer;      // Buffer for logging trajectory reference data each step
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

		// Flags for tracking initialization and settings state
        bool _settingsValid = false;
        bool _shadowsInit = false;
        bool _hdrUserOverride = false;

        // Editor & UI
        gui::FpsCounter _fpsCounter;

		// View management
        bool _isHovered = false;
        bool skyboxEnabled = true;
        bool _firstMouse = true;
        bool _firstUpdate = true;

        // Camera & Mouse
        glm::vec2 _lastMousePos{ 0.f, 0.f };
    };
} // namespace gui