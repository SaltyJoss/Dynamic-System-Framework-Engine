/*
 * File: DSL/IStoredProgram.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include "DSL/ProgramData.h"
#include "DSL/Utils.h"
#include <string>
#include <vector>

// Forward declarations
namespace commands { class ICommand; }
namespace scene { class Object;  }

namespace dsl {
	// IStoredProgram interface
	class DSFE_API IStoredProgram { 
	public:
		// Virtual destructor
		virtual ~IStoredProgram() = default;

		// Add a command to the program
		virtual void add(std::unique_ptr<commands::ICommand> cmd) = 0;
		virtual void add(commands::ICommand* cmd) = 0;

		// Reset program to initial state
		virtual void reset() = 0;

		// Clear all stored instructions
		virtual void clear() = 0;

		// Start program execution
		virtual void start() = 0;
		// Start simulation
		virtual void startSim() = 0;

		// Stop program execution
		virtual void stop() = 0;
		// Stop simulation
		virtual void stopSim() = 0;

		// Puase program execution
		virtual void pause() = 0;
		// Wait for simulation to run for dt seconds
		virtual void waitSim(double dt) = 0;

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

		// Set & Get Integrator Method
		virtual void setIntegratorMethod(IntegratorMethod method) = 0;
		virtual IntegratorMethod getIntegratorMethod() const = 0;

		// Set & Get Omega
		virtual void setOmega(mathlib::Vec3 omega, utils::AngularUnits units) = 0;

		// Set & Get Fixed Dt
		virtual void setFixedDt(double dt) = 0;
		virtual double getFixedDt() const = 0;

		// Set & Get Gravity
		virtual void setGravity(double g) = 0;
		virtual double getGravity() const = 0;
	};
} // namespace interpreter