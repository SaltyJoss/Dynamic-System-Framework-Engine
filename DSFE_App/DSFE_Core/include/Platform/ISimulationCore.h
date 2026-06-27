// DSFE_Core ISimulationCore.h
#pragma once

#include "EngineCore.h"

#include <cstdint>
#include <string>

// Forward Declarations
namespace integration { enum class eIntegrationMethod; enum class eAutoDiffIntegrationMethod; }
namespace robots { class RobotSystem; }
namespace single_body_system { class SingleBodySystem; }
namespace control { class TrajectoryManager; }
namespace diagnostics { class TelemetryRecorder; }
namespace interpreter { class IStoredProgram; }

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
        // Time stepping
        virtual void setFixedDt(double dt) = 0;
        virtual double fixedDt() const = 0;
        virtual double simTime() const = 0;
        virtual SimulationSnapshot snapshot() const = 0;
        // Integrator
        virtual void setIntegrationMethod(integration::eIntegrationMethod method) = 0;
		virtual void setADIntegrationMethod(integration::eAutoDiffIntegrationMethod method) = 0;
        virtual std::string integrationMethodName() const = 0;
        virtual integration::eIntegrationMethod integrationMethod() const = 0;
		virtual integration::eAutoDiffIntegrationMethod autoDiffIntegrationMethod() const = 0;
		virtual void enableAutoDiff(bool enable) = 0;
        // Setter for run tag name of current script
        virtual void setRunTag(const std::string& tag) = 0;
        // Subsystems
        virtual robots::RobotSystem& robotSystem() = 0;
        virtual single_body_system::SingleBodySystem& singleBodySystem() = 0;
        virtual control::TrajectoryManager& trajectoryManager() = 0;
		// Body management
		virtual bool hasSingleBody() const = 0;
		virtual void loadSingleBody(const std::string& name) = 0;
        // Robot management
        virtual bool hasRobot() const = 0;
        virtual void loadRobot(const std::string& name) = 0;
        // Script execution
        virtual bool runScriptToCompletion(interpreter::IStoredProgram* program, integration::eIntegrationMethod method) = 0;
        // Telemetry access
        virtual diagnostics::TelemetryRecorder& telemetry() = 0;
        virtual size_t telemetrySampleCount() const = 0;
    };
}