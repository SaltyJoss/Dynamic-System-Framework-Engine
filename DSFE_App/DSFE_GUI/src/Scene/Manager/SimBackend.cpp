// DSFE_GUI SimBackend.cpp
#include "Platform/ISimulationCore.h"
#include "Scene/SimulationManager.h"
#ifdef __gl_h_
#undef __gl_h_
#endif
#include "Manager/SimImplementation.h"

#include "Interpreter/IStoredProgram.h"
#include "Interpreter/StoredProgram.h"
#include "Interpreter/Parser.h"

namespace gui {
	// Start the simulation
	void SimManager::startSimulation() {
		if (!hasRobot()) {
			LOG_WARN("Cannot start simulation: no robot loaded");
			return;
		}
		if (hasRobot() && !_bodyLoaded) {
			auto& rs = _core->robotSystem();
			loadRobot(rs.robotName());
			return;
		}
		_core->startSimulation();
	}

	// Stop the simulation
	void SimManager::stopSimulation() { _core->stopSimulation(); }

	// Check if the simulation is currently running
	bool SimManager::isSimRunning() const { return _core->isSimRunning(); }

	// Setter for current simulation time (in seconds)
	void SimManager::setSimTime(double time) { _core->setSimTime(time); }
	double SimManager::simTime() const { return _core->simTime(); }

	// Setter and gettter for fixed timstep (in seconds)
	void SimManager::setFixedDt(double dt) { _core->setFixedDt(dt); }
	double SimManager::fixedDt() const { return _core->fixedDt(); }

	// Setter and getter for telemetry frequency (in Hz)
	void SimManager::setTelemetryHz(double hz) { _core->setTelemetryHz(hz); }
	double SimManager::telemetryHz() const { return _core->telemetryHz(); }

	// Set whether a script is currently running (used to disable UI elements, etc.)
	void SimManager::setScriptRunning(bool running) { _core->setScriptRunning(running); }
	bool SimManager::isScriptRunning() const { return _core->isScriptRunning(); }

	// Setters and getters for last script text
	void SimManager::setLastScriptText(const std::string& text) { _core->setLastScriptText(text); }
	std::string& SimManager::lastScriptText() const { return _core->lastScriptText(); }

	// Accessors for the Simulation Core's telemetry data
	diagnostics::TelemetryRecorder& SimManager::telemetry() { return _core->telemetry(); }
	const diagnostics::TelemetryRecorder& SimManager::telemetry() const { return _core->telemetry(); }

	// Accesors for the active program (if any)
	void SimManager::setActiveProgram(interpreter::IStoredProgram* program) { _core->setActiveProgram(program); }
	interpreter::IStoredProgram* SimManager::activeProgram() { return _core->activeProgram(); }
	const interpreter::IStoredProgram* SimManager::activeProgram() const { return _core->activeProgram(); }

	// Access the simulation core interface (non-const and const versions)
	core::ISimulationCore* SimManager::simCore() { return _core.get(); }
	const core::ISimulationCore* SimManager::simCore() const { return _core.get(); }

	// Set the integrator method for the current simulation run
	void SimManager::setIntegrationMethod(integration::eIntegrationMethod method) {
		_core->setIntegrationMethod(method);
	}
	const integration::eIntegrationMethod SimManager::integrationMethod() const {
		return _core->integrationMethod();
	}
	void SimManager::setADIntegrationMethod(integration::eAutoDiffIntegrationMethod method) {
		_core->setADIntegrationMethod(method);
	}
	const integration::eAutoDiffIntegrationMethod SimManager::autoDiffIntegrationMethod() const {
		return _core->autoDiffIntegrationMethod();
	}

	// This seems to be the better solution?
	static std::string replaceIntegratorInScript(const std::string& script, const std::string& methodName) {
		std::regex re(R"((?i)set\s*\(\s*integrator\s*,\s*([a-z0-9_]+)\s*\))"); // case-insensitive regex to match my DSL command -> set(integrator, method)
		std::string replacement = "set(integrator, " + methodName + ")";
		return std::regex_replace(script, re, replacement);
	}

	// Run a script to completion synchronously with a specific integrator
	bool SimManager::runScriptToCompletion(const std::string& scriptText, integration::eIntegrationMethod method) {
		if (!hasRobot()) { return false; }

		// Map method enum to string name
		static const char* names[] = { "euler", "midpoint", "heun", "ralston", "rk4", "rk45", "implicit_euler", "implicit_midpoint", "glrk2", "glrk3" };
		const std::string methodName = names[static_cast<int>(method)];

		// Replace the integrator method in the script text
		std::string modifiedScript = replaceIntegratorInScript(scriptText, methodName);

		// Create program and parser (bound to headless core)
		auto program = std::make_unique<interpreter::StoredProgram>(_core.get());
		//if (scene::Object* o = getObject()) program->setDefaultObject(o);
		auto parser = std::make_unique<interpreter::Parser>(program.get());

		// Parse the modified script and start the program
		parser->parse(modifiedScript);
		program->start();
		return _core->runScriptToCompletion(program.get(), method); // this will block until the script finishes
	}
}