// DSFE_Core ISimulationCore.h
#pragma once

#include "EngineCore.h"
#include <MathLib>
#include <cstdint>
#include <string>

// Forward Declarations
namespace integration { enum class eIntegrationMethod; enum class eAutoDiffIntegrationMethod; }
namespace systems { class RigidBodySystem; }
namespace single_body_system { class SingleBodySystem; }
namespace control { class TrajectoryManager; }
namespace diagnostics { class TelemetryRecorder; }
namespace dsl { class IStoredProgram; }

enum class eSimulationBackend;

namespace core {
    // Simulation Snapshot Structure
    struct DSFE_API SimulationSnapshot {
        double simTime;
        bool simRunning;
        bool scriptRunning;
    };

    // Configuration structure for the Simulation Core
    struct DSFE_API CoreConfig {
        // Placeholder... Added so I remmember to add this later
    };

    // Headless API for the Simulation Core
    // Aimed at allowing scripts and other systems to interact with the simulation without direct access to the full SimulationCore implementation
    struct DSFE_API ISimulationCore {
        virtual ~ISimulationCore() = default;

        // Simulation control
        virtual void startSimulation() = 0;
        virtual void stopSimulation() = 0;
        virtual bool isSimRunning() const = 0;
        virtual void tick(double frame_dt) = 0;
        // Time stepping
        virtual void setFixedDt(double dt) = 0;
        virtual double fixedDt() const = 0;
        virtual void setSimTime(double time) = 0;
        virtual double simTime() const = 0;
        virtual SimulationSnapshot snapshot() const = 0; // need to utilise this, that will be next stage once I have the simulation core working properly
        // Integrator
        virtual void setIntegrationMethod(integration::eIntegrationMethod method) = 0;
		virtual void setADIntegrationMethod(integration::eAutoDiffIntegrationMethod method) = 0;
        virtual std::string integrationMethodName() const = 0;
        virtual integration::eIntegrationMethod integrationMethod() const = 0;
		virtual integration::eAutoDiffIntegrationMethod autoDiffIntegrationMethod() const = 0;
		virtual void enableAutoDiff(bool enable) = 0;
        virtual bool autoDiffEnabled() const = 0;
        // Subsystems
        virtual systems::RigidBodySystem& rigidBodySystem() = 0;
        virtual single_body_system::SingleBodySystem& singleBodySystem() = 0;
        virtual control::TrajectoryManager& trajectoryManager() = 0;
		// Body management
		virtual bool hasSingleBody() const = 0;
		virtual void loadSingleBody(const std::string& name) = 0;
        // RigidBody management
        virtual bool hasRigidBody() const = 0;
        virtual void loadRigidBody(const std::string& name) = 0;
        virtual bool rigidBodyPresentationDirty() const = 0;
        virtual void clearRigidBodyPresentationDirty() = 0;
        // Script execution
        virtual void setRunTag(const std::string& tag) = 0;
        virtual void setScriptRunning(bool running) = 0;
        virtual bool isScriptRunning() const = 0;
        virtual void setLastScriptText(const std::string& text) = 0;
        virtual std::string& lastScriptText() = 0;
        virtual bool runScriptToCompletion(dsl::IStoredProgram* program, integration::eIntegrationMethod method) = 0;
        // Telemetry access
        virtual void setTelemetryHz(double hz) = 0;
        virtual double telemetryHz() const = 0;
        virtual diagnostics::TelemetryRecorder& telemetry() = 0;
        virtual size_t telemetrySampleCount() const = 0;
        // Access to the active program (if any)
        virtual void setActiveProgram(dsl::IStoredProgram* program) = 0;
        virtual dsl::IStoredProgram* activeProgram() const = 0;
        // Access the Free-Dynamics manipulation state (for external control of the rigidBody)
        virtual void setManipulating(bool on) = 0;
		virtual bool isManipulating() const = 0;
		virtual bool setLinkExternalForce(const std::string& link, const mathlib::Vec3& worldPoint, const mathlib::Vec3& worldForce) = 0;
		virtual const std::vector<mathlib::Mat4>& linkWorldTransforms() const = 0;
        virtual std::vector<std::string> linkNames() const = 0;
    };
}