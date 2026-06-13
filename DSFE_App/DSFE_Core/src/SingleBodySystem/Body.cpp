// DSFE_CORE Body.cpp
#include "pch.h"
#include "SingleBodySystem/Body.h"
#include "EngineLib/LogMacros.h"

namespace single_body_system {
	SingleBodySystem::SingleBodySystem()
		: _body(new Body()), _dynamics(std::make_unique<dynamics::SingleBodyDynamics>()),
		_integrator(std::make_unique<integration::IntegrationService>()), _AD_integrator(std::make_unique<integration::DifferentiableIntegrator>())
	{
	}

	mathlib::VecX SingleBodySystem::packState() const {
		if (!_body) { LOG_ERROR("Body not initialised!"); return; }
		mathlib::VecX x(13);
		x.segment<3>(0) = _body->state.p; // position
		x.segment<3>(3) = _body->state.pd; // linear velocity
		x.segment<4>(6) = _body->state.w; // angular velocity
		x.segment<3>(9) = _body->state.q.coeffs(); // quaternion coefficients <- included because this should work for various single bodies, but in the case of a particle we can just use a different state representation via particle_packState()
		return x;
	}

	void SingleBodySystem::unpackState(const mathlib::VecX& x) {
		if (!_body) { LOG_ERROR("Body not initialised!"); return; }
		if (x.size() != 13) { LOG_ERROR("State vector size mismatch! Expected 13, got %d", (int)x.size()); return; }

		if (_clampVel.size() != 3) { _clampVel.resize(3, 0); }
		if (_clampAngVel.size() != 3) { _clampAngVel.resize(3, 0); }

		auto& p_in = x.segment<3>(0);
		auto& pd_in = x.segment<3>(3);
		auto& w_in = x.segment<3>(6);

		for (int i = 0; i < 3; ++i) {
			double pd_i = pd_in[i]; // Store the original value before clamping
			double w_i = w_in[i];   // Store the original value before clamping

			double pdMax_hpd = std::abs(_body->limits.pdMax[i]);
			double pd_i_out = pd_i;
			double wMax_hw = std::abs(_body->limits.wMax[i]);
			double w_i_out = w_i;

			const double eps = 0.05; // 5 % tolerance for clamping
			// TODO: Check the correct way to clamp the velocities close the S.o.L
			//	* Not really sure if I have done this correctly
			if (pdMax_hpd > 0.0) {
				if (pdMax_hpd == constants::c_0) { std::clamp(pd_i, -pdMax_hpd, pdMax_hpd); }
				else if (std::abs(pd_i) > (1.0 + eps) * pdMax_hpd) { pd_i_out = std::clamp(pd_i, -pdMax_hpd, pdMax_hpd); }
			}
			if (wMax_hw > 0.0) {
				if (wMax_hw == constants::c_0) { std::clamp(w_i, -wMax_hw, wMax_hw); }
				else if (std::abs(w_i) > (1.0 + eps) * wMax_hw) { w_i_out = std::clamp(w_i, -wMax_hw, wMax_hw); }
			}

			_clampVel[i] = (pd_i_out != pd_i) ? 1 : 0;
			_clampAngVel[i] = (w_i_out != w_i) ? 1 : 0;

			_body->state.p[i] = p_in[i];
			_body->state.pd[i] = pd_i_out;
			_body->state.w[i] = w_i_out;
		}
		_body->state.q.coeffs() = x.segment<4>(9);


		if (_clampVel[0] || _clampVel[1] || _clampVel[2]) { LOG_WARN("Linear velocity clamping applied: pd = [%f, %f, %f]", pd_in[0], pd_in[1], pd_in[2]); }
		if (_clampAngVel[0] || _clampAngVel[1] || _clampAngVel[2]) { LOG_WARN("Angular velocity clamping applied: w = [%f, %f, %f]", w_in[0], w_in[1], w_in[2]); }
	}

	// Step the simulation forward by dt seconds at time t
	void SingleBodySystem::step(double dt, double t) {
		if (!_body) { LOG_ERROR("Body not initialised!"); return; }
		if (!_dynamics) { LOG_ERROR("Dynamics system not initialised!"); return; }
		
		_simTime = t;
		VecX x = packState();
		// Not going to use the dynamics methods initially, just want a straight cut test first.
		auto func = [this](const mathlib::VecX& x, mathlib::VecX& dxdt) {
			// Unpack the state vector into the body state
			unpackState(x);
			// Compute the derivatives of the state (dx/dt)
			mathlib::Vec3 externalForce{ 0.0, 0.0, 0.0 }; // Placeholder for external forces
			mathlib::Vec3 externalTorque{ 0.0, 0.0, 0.0 }; // Placeholder for external torques
			// Compute linear acceleration
			if (externalForce == mathlib::Vec3(0.0, 0.0, 0.0)) { _body->state.pdd = mathlib::Vec3(0.0, 0.0, 0.0); }
			else { _body->state.pdd = (externalForce / _body->inertia.mass).eval(); }
			// Compute angular acceleration
			mathlib::Mat3 inertiaInv = _body->inertia.inertiaTensor.inverse();
			_body->state.wd = inertiaInv * (externalTorque - _body->state.w.cross(_body->inertia.inertiaTensor * _body->state.w));
			// Fill in the derivative vector dxdt
			dxdt.segment<3>(0) = _body->state.pd; // dp/dt = pd
			dxdt.segment<3>(3) = _body->state.pdd; // dpd/dt = pdd
			dxdt.segment<4>(6) = _body->state.w; // dw/dt = w
			dxdt.segment<3>(10) = _body->state.wd; // dwd/dt = wd
		};

		auto step = _integrator->step(_curIntMethod, x, t, dt, func, nullptr); // Wont work with Implicit since no jacobian provided, but will work with RK4 and other explicit methods
		// Also will not with adaptive (probably) since no rtol/atol provided, but will work with fixed step methods
		VecX x_next = step.x_next;

		unpackState(x);
	}
}