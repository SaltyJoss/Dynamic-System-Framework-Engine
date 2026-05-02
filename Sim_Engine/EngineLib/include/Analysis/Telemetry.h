#pragma once
// File:    Telemetry.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include <vector>
#include <functional>

#include "Platform/Logger.h"
#include "EngineLib/LogMacros.h"

namespace robots { class DSFE_API RobotSystem; }
namespace control { class DSFE_API TrajectoryManager; }

namespace diagnostics {
	// Enum for telemetry levels
	enum class eTelemetryLevel {
		NONE  = 0,	// No telemetry data
		BASIC = 1,	// Basic telemetry data
		FULL  = 2	// Detailed telemetry data
	};

	// Struct for joint telemetry data
	struct DSFE_API JointTelemetry{
		// Actual data
		double q   = 0.0f; // Joint angle in radians
		double qd  = 0.0f; // Joint angular velocity in radians per second
		double eta = 0.0f; // 

		// Additional dynamics data
		double torqueNm = 0.0f; // Joint torque (N·m)
		double damping  = 0.0f; // Joint damping coefficient  (kg·m²/s)
		double friction = 0.0f; // Joint friction coefficient (Coulomb friction - N·m)
		double effort   = 0.0f; // Normalized effort (0 to 1)

		// Reference data
		double q_ref   = 0.0f; // Reference joint angle in radians
		double qd_ref  = 0.0f; // Reference joint angular velocity in radians per second
		double qdd_ref = 0.0f; // Reference joint angular acceleration in radians per second squared

		// Trajectory data
		double traj_q    = 0.0f;  // Trajectory joint position
		double traj_qd   = 0.0f;  // Trajectory joint velocity
		double traj_qdd  = 0.0f;  // Trajectory joint acceleration
		bool traj_active = false; // Trajectory active state for a joint

		// Limit clamping flags
		bool clampTheta = false; // Whether the joint angle is clamped to limits
		bool clampOmega = false; // Whether the joint velocity is clamped to limits
	};

	// Struct for a single telemetry sample
	struct DSFE_API TelemetrySample {
		// Timestamp and joint data
		double timeSec = 0.0;		   // Timestamp of the sample in seconds
		std::vector<JointTelemetry> j; // Vector of joint telemetry data

		// Summary statistics (precomputed to relieve analysis load)
		double err_rms	= 0.0f;	// RMS error across all joints
		double err_max	= 0.0f;	// Maximum error across all joints
		int clamp_theta = 0;	// Sum of angle clamping events across all joints
		int clamp_omega = 0;	// Sum of velocity clamping events across all joints
		int clamp_sum	= 0;	// Sum of clamping events across all joints
		int worst_joint = -1;	// Index of the joint with max |e|
	};

	// Ring buffer for storing telemetry samples
	class DSFE_API TelemetryRing {
	public:
		// Constructor with specified capacity
		explicit TelemetryRing(size_t cap = 10000) : _cap(cap) { _buf.resize(cap); }

		// Clear the ring buffer
		void clear() { _size = 0; _head = 0; }
		// Get current size
		size_t size() const { return _size; }
		// Get capacity
		size_t capacity() const { return _cap; }

		// Reset the ring buffer with new capacity
		void reset(size_t cap) {
			_cap = std::max<size_t>(cap, 1);
			_buf.clear();
			_buf.resize(_cap);
			clear();
		}

		// Begin write operation
		TelemetrySample& beginWrite() {
			TelemetrySample& slot = _buf[_head];
			return slot;
		}

		// Finalize write operation
		void endWrite() {
			_head = (_head + 1) % _cap;
			if (_size < _cap) { ++_size; }
		}

		// Access element in logical order (oldest to newest)
		const TelemetrySample& at(size_t i) const {
			const size_t start = (_head + _cap - _size) % _cap; // Index of the oldest element
			return _buf[(start + i) % _cap]; // Access element in logical order (oldest to newest) 
		}

	private:
		size_t _cap  = 0;					// Capacity of the ring buffer
		size_t _size = 0;					// Current size of the buffer
		size_t _head = 0;					// Head index for the next write
		std::vector<TelemetrySample> _buf;	// Buffer to store telemetry samples
	};

	// Interface for recording telemetry data
	class DSFE_API TelemetryRecorder {
	public:
		// Constructor
		TelemetryRecorder() = default;

		// Begin a telemetry recording session
		void beginRun(double simTime0, double hz, double maxSeconds = 120.0) {
			_fs = (hz > 0.0) ? hz : 60.0;
			_T = 1.0 / _fs;
			_next_t = simTime0;

			const size_t capSamples = (size_t)std::ceil(maxSeconds * _fs);
			ring.reset(capSamples);
		}

		// Clear all recorded telemetry data
		void clear() {
			ring.clear();
			_next_t = 0.0;
		}

		// Update method to be called each simulation step
		void update(double simTime, const robots::RobotSystem& robotSys, const control::TrajectoryManager* trajOpt = nullptr, eTelemetryLevel level = eTelemetryLevel::NONE) {
			if (level == eTelemetryLevel::NONE) { return; }

			// Record samples at the specified frequency
			while (simTime >= _next_t) {
				// Record telemetry data
				record(_next_t, robotSys, trajOpt, level);
				_next_t += _T;
			}
		}

		// Ring buffer to store telemetry samples
		TelemetryRing ring;

	private:
		// Record telemetry data at time t
		void record(double t, const robots::RobotSystem& robotSys, const control::TrajectoryManager* trajOpt, eTelemetryLevel level);

		// Sampling parameters
		double _fs = 60.0;
		double _T = 1.0 / 60.0;
		double _next_t = 0.0;
	};

} // namespace diagnostics

// Reminder of Hz -> seconds:
// --------------------------
// 1 Hz    =       1 second
// 10 Hz   =     0.1 seconds
// 60 Hz   =  0.0167 seconds
// 120 Hz  =  0.0083 seconds
// 180 Hz  =  0.0056 seconds
// 240 Hz  =  0.0042 seconds
// 360 Hz  =  0.0028 seconds
// 480 Hz  =  0.0021 seconds
// 720 Hz  =  0.0014 seconds
// 960 Hz  =  0.0010 seconds
// 1200 Hz = 0.00083 seconds
// --------------------------