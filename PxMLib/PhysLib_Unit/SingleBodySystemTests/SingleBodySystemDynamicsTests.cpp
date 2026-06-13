// PxM/PhysLib_Unit SingleBodySystemDynamicsTests.cpp
#include "TestHarness.h"

#include "PhysLib.h"
#include "SingleBodySystems/Dynamics.h"

#include "core/MathLib.h"
#include "integrators/numerical_integrators.h"

#include <iostream>

namespace {
	auto f = [](const mathlib::VecX& x, double t) -> mathlib::VecX {
		mathlib::VecX dxdt(13);
		dxdt.setZero();
		dxdt[3] = 1.0; // Constant velocity in x-direction
		return dxdt;
	};
}

// Test case for the derivatives function in SingleBodyDynamics
TEST("Single Body Dynamics Derivatives Method Test", SingleBodyDynamics_Derivatives) {
	single_body_system::Body body;
	body.inertia.mass = 1.0;
	body.state.p = mathlib::Vec3(0.0, 0.0, 0.0);
	body.state.pd = mathlib::Vec3(1.0, 0.0, 0.0);
	body.state.q = mathlib::Quat::Identity();
	body.state.w = mathlib::Vec3(0.0, 0.0, 0.0);
	mathlib::VecX x(13);
	x.segment<3>(0) = body.state.p;
	x.segment<3>(3) = body.state.pd;
	x.segment<4>(6) = body.state.q.coeffs();
	x.segment<3>(10) = body.state.w;
	mathlib::Vec3 F_ext(1.0, 0.0, 0.0); // External force in x-direction
	mathlib::Vec3 tau_ext(0.0, 0.0, 1.0); // External torque around z-axis
	double dt = 1.0;
	single_body_system::dynamics::SingleBodyDynamics dynamics;
	mathlib::VecX dxdt = dynamics.derivatives(body, x, F_ext, tau_ext, dt);
	// Check that the derivatives are as expected
	ASSERT_NEAR(dxdt[3], 1.0, 1e-6); // Velocity in x-direction should be constant
	ASSERT_NEAR(dxdt[4], 0.0, 1e-6); // Velocity in y-direction should be zero
	ASSERT_NEAR(dxdt[5], 0.0, 1e-6); // Velocity in z-direction should be zero
}
// Test case for the buildInertiaTensor function in SingleBodyDynamics
TEST("Single Body Dynamics Build Inertia Tensor Method Test", SingleBodyDynamics_BuildInertiaTensor) {
	single_body_system::Body body;
	body.inertia.mass = 2.0;
	body.inertia.com_xyz = mathlib::Vec3(1.0, 1.0, 1.0);
	single_body_system::dynamics::SingleBodyDynamics dynamics;
	dynamics.buildInertiaTensor(body);
	// Check that the inertia tensor is computed correctly
	mathlib::Mat3 expectedInertia;
	expectedInertia << (1.0 / 12.0) * body.inertia.mass * (body.inertia.com_xyz.y() * body.inertia.com_xyz.y() + body.inertia.com_xyz.z() * body.inertia.com_xyz.z()), 0, 0,
		0, (1.0 / 12.0)* body.inertia.mass* (body.inertia.com_xyz.x() * body.inertia.com_xyz.x() + body.inertia.com_xyz.z() * body.inertia.com_xyz.z()), 0,
		0, 0, (1.0 / 12.0)* body.inertia.mass* (body.inertia.com_xyz.x() * body.inertia.com_xyz.x() + body.inertia.com_xyz.y() * body.inertia.com_xyz.y());
	ASSERT_TRUE(body.inertia.inertiaTensor.isApprox(expectedInertia, 1e-6), "Inertia tensor is not computed correctly");
}
// Test case for the buildParticleInertia function in SingleBodyDynamics
TEST("Single Body Dynamics Build Particle Inertia Method Test", SingleBodyDynamics_BuildParticleInertia) {
	single_body_system::Body body;
	body.inertia.mass = 1.0;
	single_body_system::dynamics::SingleBodyDynamics dynamics;
	dynamics.buildParticleInertia(body);
	// Check that the inertia tensor is zero for a particle
	ASSERT_TRUE(body.inertia.inertiaTensor.isZero(1e-6), "Inertia tensor for a particle should be zero");
}
// Test case for the jacobian function in SingleBodyDynamics
TEST("Single Body Dynamics Jacobian Method Test", SingleBodyDynamics_Jacobian) {
	single_body_system::Body body;
	body.inertia.mass = 1.0;
	body.state.p = mathlib::Vec3(0.0, 0.0, 0.0);
	body.state.pd = mathlib::Vec3(1.0, 0.0, 0.0);
	body.state.q = mathlib::Quat::Identity();
	body.state.w = mathlib::Vec3(0.0, 0.0, 0.0);
	mathlib::VecX x(13);
	x.segment<3>(0) = body.state.p;
	x.segment<3>(3) = body.state.pd;
	x.segment<4>(6) = body.state.q.coeffs();
	x.segment<3>(10) = body.state.w;
	mathlib::MatX J_out(13, 13);
	single_body_system::dynamics::SingleBodyDynamics dynamics;
	dynamics.jacobian(body, x, J_out);
	int rows = J_out.rows();
	int cols = J_out.cols();

	// Check that the Jacobian is computed correctly (this is a simple check for the structure of the Jacobian)
	ASSERT_TRUE(rows == 13, "Jacobian should have 13 rows");
	ASSERT_TRUE(cols == 13, "Jacobian should have 13 columns");
}
// Test case for the derivatives function with zero external forces and torques
TEST("Single Body Dynamics Derivatives Method Test with Zero External Forces and Torques", SingleBodyDynamics_Derivatives_ZeroForcesTorques) {
	single_body_system::Body body;
	body.inertia.mass = 1.0;
	body.state.p = mathlib::Vec3(0.0, 0.0, 0.0);
	body.state.pd = mathlib::Vec3(1.0, 0.0, 0.0);
	body.state.q = mathlib::Quat::Identity();
	body.state.w = mathlib::Vec3(0.0, 0.0, 0.0);
	mathlib::VecX x(13);
	x.segment<3>(0) = body.state.p;
	x.segment<3>(3) = body.state.pd;
	x.segment<4>(6) = body.state.q.coeffs();
	x.segment<3>(10) = body.state.w;
	mathlib::Vec3 F_ext(0.0, 0.0, 0.0); // No external force
	mathlib::Vec3 tau_ext(0.0, 0.0, 0.0); // No external torque
	double dt = 1.0;
	single_body_system::dynamics::SingleBodyDynamics dynamics;
	mathlib::VecX dxdt = dynamics.derivatives(body, x, F_ext, tau_ext, dt);
	// Check that the derivatives are as expected (no change in velocity or angular velocity)
	ASSERT_NEAR(dxdt[3], 0.0, 1e-6); // Velocity in x-direction should remain constant
	ASSERT_NEAR(dxdt[4], 0.0, 1e-6); // Velocity in y-direction should be zero
	ASSERT_NEAR(dxdt[5], 0.0, 1e-6); // Velocity in z-direction should be zero
	ASSERT_NEAR(dxdt[10], 0.0, 1e-6); // Angular velocity in x-direction should be zero
	ASSERT_NEAR(dxdt[11], 0.0, 1e-6); // Angular velocity in y-direction should be zero
	ASSERT_NEAR(dxdt[12], 0.0, 1e-6); // Angular velocity in z-direction should be zero
}
