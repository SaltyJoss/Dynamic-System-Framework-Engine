// DSFE_GUI SimulationManager.h
#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include <unordered_set>
#include "Platform/StudyRunner.h"

#include "Simulation/MeshStore.h"
#include "Simulation/SimulationScene.h"
#include "Robots/RobotBinding.h"
#include "Renderer/VulkanRenderer.h"
#include "Scene/ObjectID.h"
#include "ui/RenderPreset.h"

#include "Platform/KeyCode.h"

#include "Analysis/Telemetry.h"
#include "Analysis/MetricLogger.h"

#include "Platform/Logger.h"

// Forward Declarations for Scene
namespace scene {
    class Light;
    class Camera;
    class Mesh;
    class Object;
    class SceneRenderer;
}

// Forward Declarations for Simulation Core
namespace core { class ISimulationCore; }

// Forward Declarations for Physics, Robots, Control, and Integration
namespace interpreter { class IStoredProgram; }
namespace robots { class RobotSystem; struct RobotModel; }
namespace control { class TrajectoryManager; }
namespace integration { enum class eIntegrationMethod; }

namespace gui {
    // View IDs
    enum class ViewID { Manual = 0, Top, Right, Front, Follow, COUNT };

    class SimulationRenderer;

	// Forward Declarations for eKeyCode
    enum class eKeyCode;

    // Control Modes & Camera
    enum class ControlMode {
        Camera,
        Object
    };

    struct CoreDeleter{
        void operator()(core::ISimulationCore* p) const { 
            if (p) {
                DestroySimulationCore(p);
            }
        }
    };

    class SimulationManager {
    public:
		// Constructor & Destructor
        SimulationManager();
        ~SimulationManager();

        void initialiseRenderer(void* nativeWindowHandle);
        void resizeRenderer(int w, int h);
        void renderViewport(int w, int h);
        bool rendererReady() const { return _rendererInitialised; }

        uint32_t load_mesh(const std::string& path);

		// Light
        scene::Light* getLight();
        void setLightColour(const glm::vec3& c);

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

		// Render Settings & Profiles
        void applyRenderSettings(const render::RenderSettings& s, render::ResolutionPreset r);
        void applyRenderProfile(const render::RenderSettings& s, render::ResolutionPreset r);
		void resetHDRToPreset();

		// Rendering Entry Points
        void tick(double dt);
        void setDisplaySize(int w, int h);
        void resize(int32_t width, int32_t height);

		void syncRobotToScene();
		void syncBodyToScene();

		// Scene Objects Management
        void setSelectedObject(scene::Object* obj);
        void addObject(std::unique_ptr<scene::Object> obj);
        void deleteObject(int index);
		void removeObject(scene::Object* obj);

		// Scene Objects Lookup
        std::vector<std::unique_ptr<scene::Object>>& getObjects();
        scene::Object* getObject();
		scene::Object* getObjectByID(scene::ObjectID id);

		// Robot System loading and management
        void load_robot(const std::string& name);
        void resetRobot();
        void clearRobot();
        const bool hasRobot() const;

		const bool hasBody() const;

		// Setters for robot joint states (angle in radians)
        void setRobotLinkRotation(const std::string& linkName, double angle);
        void setRobotRootPose(const mathlib::Vec3& pos, mathlib::Quat& rot);
        void setRobotRootHome(const mathlib::Vec3& pos, mathlib::Quat& rot);

		// Accesors for the robot system (non-const and const versions)
        robots::RobotSystem& robotSystem();
        const robots::RobotSystem& robotSystem() const;

		single_body_system::SingleBodySystem& singleBodySystem();
		const single_body_system::SingleBodySystem& singleBodySystem() const;

		// Accessors for the trajectory manager (non-const and const versions)
        control::TrajectoryManager& traj();
        const control::TrajectoryManager& traj() const;

        // Simulation Control
        void startSimulation();
        void stopSimulation();

		// Simulation State
        bool isSimRunning() const;

        // Setter and getter for current simulation time (seconds)
        void setSimTime(double time);
        double simTime() const;

        // Setter and getter for fixed timestep (seconds)
        void setFixedDt(double dt);
        double fixedDt() const;

        // Setter and getter for telemetry frequency (Hz)
        void setTelemetryHz(double hz);
        double telemetryHz() const;

		// Setters for script and simulation running states
        void setScriptRunning(bool running);
        bool isScriptRunning() const;

		// Setters and getters for last script text
		void setLastScriptText(const std::string& text);
		std::string& lastScriptText() const;

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
        core::ISimulationCore* simCore();
        const core::ISimulationCore* simCore() const;

        // Setters for Integration state / method
		void setIntegrationMethod(integration::eIntegrationMethod method);
		const integration::eIntegrationMethod integrationMethod() const;
        void setADIntegrationMethod(integration::eAutoDiffIntegrationMethod method);
		const integration::eAutoDiffIntegrationMethod autoDiffIntegrationMethod() const;

		// Access to the underlying StudyRunner for running batch studies from the GUI
		StudyRunner* studyRunner() { return _studyRunner.get(); }

		// Methods for handling completed studies from the background worker
        void pushCompletedStudies(std::vector<StudyResult> results);
		void pushCompletedStudy(StudyResult result);
        bool hasCompletedStudy() const;
        std::vector<StudyResult> consumeCompletedStudy();

        // Input Handling
        void processMovementKey(int key, float delta);
        void handleContinuousMovement(const std::unordered_set<eKeyCode>& pressedKeys, float dt);
        void handleMouseLook(double xpos, double ypos, bool mouseCaptured);
        void onMouseWheel(double delta);
        void resetMouseDelta();

    private:
        std::unique_ptr<core::ISimulationCore, CoreDeleter> _core = nullptr;

        renderer::VulkanRenderer _renderer;
        bool _rendererInitialised = false;
        std::unique_ptr<SimulationRenderer> _sim_renderer;
        MeshStore _mesh_store;
        SimulationScene _scene;
        RobotBinding _robot_binding;
        std::vector<uint32_t> _loaded_mesh_ids;

        void buildRobotVisuals(const robots::RobotModel& model);
        void updateRobotTransforms();
        //void buildRobotCollision(const robots::RobotModel& model);
        
		std::unique_ptr<StudyRunner> _studyRunner = nullptr; // Background worker for running batch studies
		bool _hasCompletedStudy = false;

        // Misc Settings
        bool _bodyLoaded = false;

		// Sizes & Display
		glm::vec2 _internalSize{ 1920.0f, 1080.0f };  // Internal render target size
		glm::vec2 _displaySize{ 1920.0f, 1080.0f };   // Actual display size
		glm::vec3 _backgroundColour{ 1.0f, 1.0f, 1.0f }; // Background colour (default white, but can be changed by user)

        float _backgroundAlpha = 1.0f;

        // Last script text for comparison re-use
        std::string _lastScriptText;

		// Ground Plane
        static constexpr float planeHeight = -2.5f;
        float planeY = 2.5f;
        glm::vec3 planeNormal{ 0.0f, 1.0f, 0.0f };

		// Objects & Scene Management
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

		// Flags for tracking initialisation and settings state
        bool _settingsValid = false;

		// View management
        bool _isHovered = false;
        bool _firstMouse = true;
        bool _firstUpdate = true;

        bool skyboxEnabled = true;
        bool floorEnabled = true;

        // Camera & Mouse
        glm::vec2 _lastMousePos{ 0.f, 0.f };

        // Stub state — replace as the real subsystems come back
        bool   _simRunning    = false;
        bool   _scriptRunning = false;
        double _simTime       = 0.0;
        double _fixedDt       = 1.0 / 240.0;
        double _telemetryHz   = 100.0;

        scene::Object* _selectedObject = nullptr;
        std::vector<std::unique_ptr<scene::Object>> _objects;

        interpreter::IStoredProgram* _activeProgram = nullptr;

        integration::eIntegrationMethod         _integrationMethod{};
        integration::eAutoDiffIntegrationMethod _adIntegrationMethod{};
    };
} // namespace gui