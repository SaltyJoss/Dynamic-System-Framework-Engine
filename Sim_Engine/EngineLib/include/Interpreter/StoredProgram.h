#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include "ICommand.h"
#include "Interpreter/MainContext.h"
#include <string>
#include <vector>

namespace interpreter {
	// Class representing a stored program in the interpreter.
	class ENGINE_API StoredProgram : public IStoredProgram {
	public:
		// Constructor
		StoredProgram(gui::simManager* sim);
		~StoredProgram() override;

		// Add a command to the program
		void add(commands::ICommand* cmd) override;

		// Reset program to initial state
		void reset() override;
		// Clear all stored instructions
		void clear() override;
		// Start program execution
		void start() override;
		// Stop program execution
		void stop() override;
		// Puase program execution
		void pause() override;
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

		// Get the current instruction
		const Command* getCurrentInstruction() const;

		// Set & Get Current line number
		void setCurrentLineNumber(int lineNumber) override { _currentLineNumber = lineNumber; }
		int getCurrentLineNumber() const override { return _currentLineNumber; }

		// Set default object
		void setDefaultObject(scene::Object* obj) override { _defaultObj = obj; }
		scene::Object* defaultObject() const override { return _defaultObj; }

		// Set & Get Integrator Method
		void setIntegratorMethod(IntegratorMethod method) override;
		IntegratorMethod getIntegratorMethod() const override;

		// Set & Get Colour
		void setColour(mathlib::Vec3 rgb) override;
		mathlib::Vec3 getColour() const override;

	private:
		gui::simManager* _sim = nullptr;
		commands::MainContext _cntx;
		scene::Object* _defaultObj = nullptr;

		// Bool for tracking if the program has reached the end
		bool atEnd() const;
		// Bool for tracking if there are commands left to execute
		bool commandsLeft() const;

		ProgramState _state = ProgramState::Stopped;
		bool _stopRequested = false;

		CmdResult updateState() override;
		int _currentLineNumber = 0;
		int PC = 0; // Program Counter

		std::vector<commands::ICommand*> _commands;
		IntegratorMethod _integratorMethod = IntegratorMethod::Euler; // Default integrator method
		mathlib::Vec3 _rgb = mathlib::Vec3{ 1.0f, 0.0f, 0.0f };
	};
} // namespace interpreter
