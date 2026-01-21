#pragma once

#include "EngineCore.h"
#include "ProgramData.h"
#include <string>
#include <vector>

namespace commands { class ENGINE_API ICommand; }
namespace scene { class ENGINE_API Object;  }

using namespace program_data;

namespace interpreter {
	// IStoredProgram interface
	class ENGINE_API IStoredProgram { 
	public:
		// Virtual destructor
		virtual ~IStoredProgram() = default;

		// Add a command to the program
		virtual void add(commands::ICommand* cmd) = 0;

		// Reset program to initial state
		virtual void reset() = 0;
		// Clear all stored instructions
		virtual void clear() = 0;
		// Start program execution
		virtual void start() = 0;
		// Stop program execution
		virtual void stop() = 0;
		// Puase program execution
		virtual void pause() = 0;
		// Step the program by dt
		virtual void step(double dt) = 0;
		// Get current program status
		virtual ProgramStatus status() const = 0;

		// State checkers
		virtual bool isEmpty() const = 0;
		virtual bool isRunning() const = 0;
		virtual bool isPaused() const = 0;
		virtual bool isStopped() const = 0;
		virtual bool isCompleted() const = 0;
		virtual bool isFaulted() const = 0;

		// Update the command state
		virtual CmdResult updateState() = 0;

		// Set & Get Current line number
		virtual void setCurrentLineNumber(int lineNumber) = 0;
		virtual int getCurrentLineNumber() const = 0;


		// Set & Get Default Object
		virtual void setDefaultObject(scene::Object* obj) = 0;
		virtual scene::Object* defaultObject() const = 0;

		// Set & Get Integrator Method
		virtual void setIntegratorMethod(IntegratorMethod method) = 0;
		virtual IntegratorMethod getIntegratorMethod() const = 0;

		// Set & Get Omega
		virtual void setOmega(mathlib::Vec3 omega, utils::AngularUnits units) = 0;
		virtual mathlib::Vec3 getOmega(utils::AngularUnits units) const = 0;

		// Set & Get Fixed Dt
		virtual void setFixedDt(double dt) = 0;
		virtual double getFixedDt() const = 0;

		// Set & Get Colour
		virtual void setColour(mathlib::Vec3 rgb) = 0;
		virtual mathlib::Vec3 getColour() const = 0;
	};
} // namespace interpreter