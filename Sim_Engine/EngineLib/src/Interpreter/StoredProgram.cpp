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
		_commands = std::vector<commands::ICommand*>();
	}

	StoredProgram::~StoredProgram() { clear(); }

	// Convert mathlib::Vec3 to glm::vec3
	inline glm::vec3 toGlm(const mathlib::Vec3& v) { return glm::vec3(v.x(), v.y(), v.z()); }

	void StoredProgram::add(commands::ICommand* cmd) {
		if (cmd == nullptr) {
			D_FAIL("Attempted to add null command to StoredProgram.");
			throw std::invalid_argument("Attempted to add null command to StoredProgram.");
		}

		cmd->setContext(_cntx.motion());
		cmd->setContext(_cntx.ui());

		cmd->setProgram(this);
		_commands.push_back(cmd);
	}

	void StoredProgram::reset() {
		_currentLineNumber = 0;
		PC = 0;
	}

	// Clear all stored instructions
	void StoredProgram::clear() {
		for (auto* c : _commands) { delete c; }
		_commands.clear();
		_currentLineNumber = 0;
		PC = 0;
		_state = ProgramState::Stopped;
		_stopRequested = false;
	}

	void StoredProgram::start() {
		_state = ProgramState::Running;
		_stopRequested = false;
	}

	void StoredProgram::stop() {
		_state = ProgramState::Stopped;
		_stopRequested = true;

		scene::Object* obj = _cntx.motion().resolveDefaultObject(); // <-- uses stored default ID
		if (obj) {
			utils::AxisMask all{ true,true,true };
			_cntx.motion().stopRotation(obj, all);
			_cntx.motion().stopTranslation(obj, all);
		}

		if (_sim && _sim->hasRobot()) { _sim->getRobotSystem()->updateRobotKinematics(); }
	}

	void StoredProgram::pause() {
		_state = ProgramState::Paused;

		scene::Object* obj = _cntx.motion().resolveDefaultObject(); // <-- uses stored default ID
		if (obj) {
			utils::AxisMask all{ true,true,true };
			_cntx.motion().stopRotation(obj, all);
			_cntx.motion().stopTranslation(obj, all);
		}

		if (_sim && _sim->hasRobot()) { _sim->getRobotSystem()->updateRobotKinematics(); }
	}

	ProgramStatus StoredProgram::status() const { return ProgramStatus{}; }
	bool StoredProgram::atEnd() const { return PC >= static_cast<int>(_commands.size()); }
	bool StoredProgram::commandsLeft() const { return PC >= 0 && PC < static_cast<int>(_commands.size()); }

	void StoredProgram::step(double dt) {
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

	CmdResult StoredProgram::updateState() {
		return CmdResult{};
	}

	// Set Integrator Method
	void StoredProgram::setIntegratorMethod(IntegratorMethod method) {
		_integratorMethod = method;
		if (_sim) {
			auto& physics = _sim->getPhysicsSystem();
			physics.setIntegrationMethod(static_cast<physics::PhysicsSystem::eIntegrationMethod>(method));
		}
	}
	// Get Integrator Method
	IntegratorMethod StoredProgram::getIntegratorMethod() const { return _integratorMethod; }

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