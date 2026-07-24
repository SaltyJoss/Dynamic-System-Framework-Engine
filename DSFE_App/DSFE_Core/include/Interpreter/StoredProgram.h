// DSFE_Core StoredProgram.h
#pragma once
#include "EngineCore.h"
#include "IStoredProgram.h"
#include "ICommand.h"
#include "Interpreter/MainContext.h"
#include <string>
#include <vector>

namespace interpreter {
	// Class representing a stored program in the interpreter.
	class DSFE_API StoredProgram : public IStoredProgram {
	public:
		// Constructor
		StoredProgram(core::ISimulationCore* core);
		~StoredProgram() override;

		// Delete copy constructor and assignment operator to prevent copies
		StoredProgram(const StoredProgram&) = delete;
		StoredProgram& operator=(const StoredProgram&) = delete;

		// Delete move constructor and assignment operator to prevent moves
		StoredProgram(StoredProgram&&) = delete;
		StoredProgram& operator=(StoredProgram&&) = delete;

		void add(std::unique_ptr<commands::ICommand> cmd) override;
		void add(commands::ICommand* cmd) override;

		void reset() override;

		void clear() override;

		void start() override;
		void startSim() override;

		void stop() override;
		void stopSim() override;

		void pause() override;
		void waitSim(double dt) override;

		// Step the program by dt
		void step(double dt) override;

		// Get current program status
		ProgramStatus status() const override;

		// State checkers
		bool isEmpty() const override { return _commands.empty(); }	
		bool isRunning() const override { return _state == ProgramState::Running; }
		bool isPaused() const override { return _state == ProgramState::Paused; }
		bool isStopped() const override { return _state == ProgramState::Stopped; }
		bool isCompleted() const override { return _state == ProgramState::Completed; }
		bool isFaulted() const override { return _state == ProgramState::Faulted; }

		// Set & Get Current line number
		void setCurrentLineNumber(int lineNumber) override { _currentLineNumber = lineNumber; }
		int getCurrentLineNumber() const override { return _currentLineNumber; }

		// Set & Get Integrator Method
		void setIntegratorMethod(IntegratorMethod method) override;
		IntegratorMethod getIntegratorMethod() const override;

		// Set & Get Omega
		void setOmega(mathlib::Vec3 omega, utils::AngularUnits units) override;

		// Set & Get Fixed Dt
		void setFixedDt(double dt) override;
		double getFixedDt() const override;

		// Set & Get Gravity
		void setGravity(double gravity) override;
		double getGravity() const override;

	private:
		core::ISimulationCore* _core = nullptr;
		commands::MainContext _cntx;

		// Bool for tracking if the program has reached the end
		bool atEnd() const;
		// Bool for tracking if there are commands left to execute
		bool commandsLeft() const;

		ProgramState _state = ProgramState::Stopped;
		bool _stopRequested = false;

		CmdResult updateState() override;
		int _currentLineNumber = 0;
		int PC = 0; // Program Counter

		std::vector<std::unique_ptr<commands::ICommand>> _commands;

		IntegratorMethod _integratorMethod = IntegratorMethod::RK4; // Default integrator method
		double _gravity = 0.0;
		double _dt = 0.0;
		mathlib::Vec3 _rgb = mathlib::Vec3{ 1.0f, 0.0f, 0.0f };
	};
} // namespace interpreter
