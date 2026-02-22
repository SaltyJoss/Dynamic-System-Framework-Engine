#include "pch.h"
// File:   StoredProgram.cpp
// GitHub: SaltyJoss
#include "Interpreter/StoredProgram.h"
#include "Platform/ISimulationCore.h"
#include "Physics/PhysicsSystem.h"
#include "Robots/RobotSystem.h"
#include "Scene/ObjectID.h"
#include "Scene/Object.h"

#include "EngineLib/LogMacros.h"

namespace interpreter {
	StoredProgram::StoredProgram(core::ISimulationCore* core)
		: _currentLineNumber(0), PC(0), _core(core), _cntx(core) {
		// If necessary, set a default object by querying core->getObject()
		if (_core) {
			scene::Object* obj = _core->getObject();
			_cntx.motion().setDefaultObjectID(obj ? obj->id : scene::ObjectID::INVALID_OBJECT_ID);
		}
	}
	StoredProgram::~StoredProgram() { clear(); }

	// Convert mathlib::Vec3 to glm::vec3
	inline glm::vec3 toGlm(const mathlib::Vec3& v) { return glm::vec3(v.x(), v.y(), v.z()); }

	// Add a command to the program
	void StoredProgram::add(std::unique_ptr<commands::ICommand> cmd) {
		if (cmd == nullptr) {
			D_FAIL("Attempted to add null command to StoredProgram.");
			throw std::invalid_argument("Attempted to add null command to StoredProgram.");
		}

		cmd->setContext(_cntx.motion());
		cmd->setContext(_cntx.ui());
		cmd->setProgram(this);
		_commands.push_back(std::move(cmd));
	}

	// Add a command to the program
	void StoredProgram::add(commands::ICommand* cmd) {
		if (cmd == nullptr) {
			D_FAIL("Attempted to add null command to StoredProgram.");
			throw std::invalid_argument("Attempted to add null command to StoredProgram.");
		}
		cmd->setContext(_cntx.motion());
		cmd->setContext(_cntx.ui());
		cmd->setProgram(this);
		_commands.emplace_back(cmd);
	}

	// Reset program counters
	void StoredProgram::reset() {
		_currentLineNumber = 0;
		PC = 0;
	}

	// Clear all stored instructions
	void StoredProgram::clear() {
		for (auto& cmd : _commands) { if (cmd) { cmd->setProgram(nullptr); } } // Clear program reference from commands
		_commands.clear();
		_currentLineNumber = 0;
		PC = 0;
		_state = ProgramState::Stopped;
		_stopRequested = false;
	}

	// Start program execution
	void StoredProgram::start() {
		_state = ProgramState::Running;
		_stopRequested = false;
	}

	// Start simulation
	void StoredProgram::startSim() {
		if (_state != ProgramState::Running) { start(); }
		if (_core && !_core->isSimRunning()) { _core->startSimulation(); }
	}

	// Stop program execution
	void StoredProgram::stop() {
		_state = ProgramState::Stopped;
		_stopRequested = true;

		stopSim(); // Also stop simulation, if running
	}
	
	// Stop simulation
	void StoredProgram::stopSim() {
		if (!_core) { return; }
		if (_core->isSimRunning()) { _core->stopSimulation(); }

		scene::Object* obj = _cntx.motion().resolveDefaultObject(); // <-- uses stored default ID
		if (obj) {
			utils::AxisMask all{ true,true,true };
			_cntx.motion().stopRotation(obj, all);
			_cntx.motion().stopTranslation(obj, all);
		}

		if (_core->hasRobot()) { _cntx.motion().Robot()->stopAll(); }
	}

	// Pause program execution
	void StoredProgram::pause() {
		_state = ProgramState::Paused;

		scene::Object* obj = _cntx.motion().resolveDefaultObject(); // <-- uses stored default ID
		if (obj) {
			utils::AxisMask all{ true,true,true };
			_cntx.motion().stopRotation(obj, all);
			_cntx.motion().stopTranslation(obj, all);
		}

		if (_core->hasRobot()) { _cntx.motion().Robot()->stopAll(); }
	}

	// Wait for simulation to run for dt seconds
	void StoredProgram::waitSim(double dt) {
		if (_core && !_core->isSimRunning()) { _core->startSimulation(); }
		double elapsed = 0.0;
		const double stepDt = _core ? _core->fixedDt() : static_cast<double>(1.0 / 180.0);
		while (elapsed < dt) {
			if (_core) { _core->updatePhysics(stepDt); }
			elapsed += stepDt;
		}
		if (_core && _core->isSimRunning()) { _core->stopSimulation(); }
	}

	// Get current program status
	ProgramStatus StoredProgram::status() const { return ProgramStatus{}; }
	// Bool for tracking if the program has reached the end
	bool StoredProgram::atEnd() const { return PC >= static_cast<int>(_commands.size()); }
	// Bool for tracking if there are commands left to execute
	bool StoredProgram::commandsLeft() const { return PC >= 0 && PC < static_cast<int>(_commands.size()); }

	// Step through the program by dt seconds
	void StoredProgram::step(double dt) {
		// State checks
		if (_state == ProgramState::Paused) { return; }
		if (_state == ProgramState::Stopped || _state == ProgramState::Completed || _state == ProgramState::Faulted) { return; }
		if (_state != ProgramState::Running) { start(); }
		if (_stopRequested) { stop(); return; }
		// Command checks
		if (_commands.empty()) { _state = ProgramState::Faulted; return; }
		if (!commandsLeft()) { _state = ProgramState::Completed; return; _core->stopSimulation(); }

		// Ensure default object is valid in context
		scene::Object* o = _defaultObj ? _defaultObj : (_core ? _core->getObject() : nullptr);
		_cntx.motion().setDefaultObjectID(o ? o->id : scene::ObjectID::INVALID_OBJECT_ID);

		// Get current command
		auto& cmd = _commands[PC];
		cmd->setContext(_cntx.motion());
		cmd->setContext(_cntx.ui());

		if (!cmd->hasStarted()) { cmd->execute(); }

		// Check current result
		CmdResult r0 = cmd->currentResult();
		if (r0.state == CmdState::Failed) { _state = ProgramState::Faulted; return; }
		if (r0.state == CmdState::Executed) {
			++PC;
			if (!commandsLeft()) _state = ProgramState::Completed;
			return;
		}

		// Update command
		CmdResult r = cmd->update(_cntx.motion(), dt);

		// Check result
		if (r.state == CmdState::Failed) { _state = ProgramState::Faulted; return; }
		if (r.state == CmdState::Executed) {
			PC++; // not ++PC because we may want to re-execute the same command
			if (!commandsLeft()) { _state = ProgramState::Completed; return; }
		}
	}

	// Update the command state
	CmdResult StoredProgram::updateState() { return CmdResult{}; }

	// Set Integrator Method
	void StoredProgram::setIntegratorMethod(IntegratorMethod method) {
		_integratorMethod = method;
		if (_core) {
			if (_core->hasRobot()) {
				robots::RobotSystem* robot = _core->robotSystem();
				if (!robot) { D_FAIL("No robot system found in simulation manager."); return; }
				robot->setIntegrationMethod(static_cast<integration::eIntegrationMethod>(method));
			}

			physics::PhysicsSystem* phys = _core->physicsSystem();
			if (!phys) { D_FAIL("No physics system found in simulation manager."); return; }
			phys->setIntegrationMethod(static_cast<integration::eIntegrationMethod>(method));
		}
	}
	// Get Integrator Method
	IntegratorMethod StoredProgram::getIntegratorMethod() const { return _integratorMethod; }

	// Set Omega
	void StoredProgram::setOmega(mathlib::Vec3 omega, utils::AngularUnits units) {
		_cntx.motion().setAngularUnits(units);
		_cntx.motion().setOmega(omega);
	}

	// Set Fixed Dt
	void StoredProgram::setFixedDt(double dt) { _dt = dt; if (_core) { _core->setFixedDt(dt); } }
	// Get Fixed Dt
	double StoredProgram::getFixedDt() const {
		if (_core) { return _core->fixedDt(); }
		return _dt;
	}

	// Set Gravity
	void StoredProgram::setGravity(double gravity) { 
		_gravity = gravity;
		if (_core) {
			if (_core->hasRobot()) {
				robots::RobotSystem* robot = _core->robotSystem();
				if (!robot) { D_FAIL("No robot system found in simulation manager."); return; }
				robot->setGravity(gravity);
			}
			 
			physics::PhysicsSystem* phys = _core->physicsSystem();
			if (!phys) { D_FAIL("No physics system found in simulation manager."); return; }
			phys->setGravity(mathlib::Vec3(0.0f, 0.0f, static_cast<float>(gravity)));
		}
	}
	// Get Gravity
	double StoredProgram::getGravity() const {
		if (_core) {
			if (_core->hasRobot()) {
				robots::RobotSystem* robot = _core->robotSystem();
				if (!robot) { D_FAIL("No robot system found in simulation manager."); return _gravity; }
				return robot->getGravity();
			}

			physics::PhysicsSystem* phys = _core->physicsSystem();
			if (!phys) { D_FAIL("No physics system found in simulation manager."); return _gravity; }
			return phys->getGravity().y();
		}
		return _gravity;
	}

	// Set Colour
	void StoredProgram::setColour(mathlib::Vec3 rgb) {
		_rgb = rgb;
		if (_core) {
			D_WARN("This method has not been integrated with the rendering system yet, so it has no effect.");
		    // _core->setShaderAlbedo(toGlm(rgb));
			// D_INFO("Set shader albedo -> %.2f,%.2f,%.2f", rgb[0],rgb[1],rgb[2]);
		}
	}

	// Get Colour
	mathlib::Vec3 StoredProgram::getColour() const { return _rgb; }
} // namespace interpreter