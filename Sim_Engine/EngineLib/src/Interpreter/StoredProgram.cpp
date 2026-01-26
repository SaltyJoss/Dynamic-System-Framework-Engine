#include "pch.h"
#include "Interpreter/StoredProgram.h"
#include "Scene/SimulationManager.h"
#include "Physics/PhysicsSystem.h"
#include "Robots/RobotSystem.h"
#include "Scene/ObjectID.h"
#include "Scene/Object.h"

#include "EngineLib/LogMacros.h"

namespace interpreter {
	StoredProgram::StoredProgram(gui::simManager* sim) : _currentLineNumber(0), PC(0), _sim(sim), 
		_cntx(sim, [&] { scene::Object* o = (sim ? sim->getObject() : nullptr); return o ? o->id : scene::ObjectID::INVALID_OBJECT_ID; }()) {
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

	// Reset program to initial state
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
		if (_sim && !_sim->isSimRunning()) { _sim->startSimulation(); }
	}

	// Stop program execution
	void StoredProgram::stop() {
		_state = ProgramState::Stopped;
		_stopRequested = true;

		stopSim(); // Also stop simulation, if running
	}
	
	// Stop simulation
	void StoredProgram::stopSim() {
		if (!_sim) { return; }
		if (_sim->isSimRunning()) { _sim->stopSimulation(); }

		scene::Object* obj = _cntx.motion().resolveDefaultObject(); // <-- uses stored default ID
		if (obj) {
			utils::AxisMask all{ true,true,true };
			_cntx.motion().stopRotation(obj, all);
			_cntx.motion().stopTranslation(obj, all);
		}

		if (_sim->hasRobot()) { _cntx.motion().Robot()->stopAll(); }
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

		if (_sim->hasRobot()) { _cntx.motion().Robot()->stopAll(); }
	}

	// Wait for simulation to run for dt seconds
	void StoredProgram::waitSim(double dt) {
		if (_sim && !_sim->isSimRunning()) { _sim->startSimulation(); }
		double elapsed = 0.0;
		const double stepDt = _sim ? _sim->getFixedDeltaTime() : static_cast<double>(1.0 / 120.0);
		while (elapsed < dt) {
			if (_sim) { _sim->updatePhysics(stepDt); }
			elapsed += stepDt;
		}
		if (_sim && _sim->isSimRunning()) { _sim->stopSimulation(); }
	}

	// Get current program status
	ProgramStatus StoredProgram::status() const { return ProgramStatus{}; }
	// Bool for tracking if the program has reached the end
	bool StoredProgram::atEnd() const { return PC >= static_cast<int>(_commands.size()); }
	// Bool for tracking if there are commands left to execute
	bool StoredProgram::commandsLeft() const { return PC >= 0 && PC < static_cast<int>(_commands.size()); }

	// Step through the program by dt seconds
	void StoredProgram::step(double dt) {
		//LOG_INFO("prog step: state=%d PC=%d cmds=%zu", (int)_state, PC, _commands.size());


		if (_state == ProgramState::Paused) { return; }
		if (_state == ProgramState::Stopped || _state == ProgramState::Completed || _state == ProgramState::Faulted) { return; }
		if (_state != ProgramState::Running) { start(); }
		if (_stopRequested) { stop(); return; }
		if (_commands.empty()) { _state = ProgramState::Faulted; return; }
		if (!commandsLeft()) { _state = ProgramState::Completed; return; }

		scene::Object* o = _defaultObj ? _defaultObj : (_sim ? _sim->getObject() : nullptr);
		_cntx.motion().setDefaultObjectID(o ? o->id : scene::ObjectID::INVALID_OBJECT_ID);

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

		if (r.state == CmdState::Failed) { _state = ProgramState::Faulted; return; }
		if (r.state == CmdState::Executed) {
			PC++;
			if (!commandsLeft()) { _state = ProgramState::Completed; return; }
		}
	}

	CmdResult StoredProgram::updateState() { return CmdResult{}; }

	// Set Integrator Method
	void StoredProgram::setIntegratorMethod(IntegratorMethod method) {
		_integratorMethod = method;
		if (_sim) {
			if (_sim->hasRobot()) {
				robots::RobotSystem* robot = _sim->getRobotSystem();
				if (robot) { robot->setIntegrationMethod(static_cast<integration::eIntegrationMethod>(method)); }
			}

			physics::PhysicsSystem& phys = _sim->getPhysicsSystem();
			phys.setIntegrationMethod(static_cast<integration::eIntegrationMethod>(method));
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
	void StoredProgram::setFixedDt(double dt) { if (_sim) { _sim->setFixedDeltaTime(dt); } }
	// Get Fixed Dt
	double StoredProgram::getFixedDt() const {
		if (_sim) { return _sim->getFixedDeltaTime(); }
		return 0.0;
	}

	// Set Colour
	void StoredProgram::setColour(mathlib::Vec3 rgb) {
		_rgb = rgb;
		if (_sim) {
			_sim->setLightColour(toGlm(rgb));
			D_INFO("Set shader albedo -> %.2f,%.2f,%.2f", rgb[0],rgb[1],rgb[2]);
		}
	}

	// Get Colour
	mathlib::Vec3 StoredProgram::getColour() const { return _rgb; }
} // namespace interpreter