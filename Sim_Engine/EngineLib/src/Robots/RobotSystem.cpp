#include "pch.h"

#include "Robots/RobotSystem.h"
#include "Robots/RobotLoader.h"
#include "Scene/Object.h"
#include "Scene/Mesh.h"

#include <stack>
#include <unordered_set>

#include <glm/gtc/matrix_transform.hpp>
#include <Core/Utils.h>
#include <kinematics/Forward_Kinematics.h>
#include "Robots/TrajectoryManager.h"
#include "Platform/Paths.h"

#include "EngineLib/LogMacros.h"

#include "Platform/DataManager.h"

using namespace mathlib;
using namespace constants;

namespace robots {
	// Constructor
	RobotSystem::RobotSystem(std::vector<std::unique_ptr<scene::Object>>& objects, spawnFn meshLoader)
		: _integrator(std::make_unique<integration::IntegrationService>()), _curIntMethod(integration::eIntegrationMethod::Euler), 
		_objects(objects), _loadMeshReturn(std::move(meshLoader)) {
		if (!_integrator) { LOG_WARN("RobotSystem got null IntegrationService*"); }
	}

	// --- 'toGlm' OVERLOADS ---

	static glm::vec3 toGlm(const Vec3& v) { return glm::vec3(v.x(), v.y(), v.z()); }
	static glm::vec4 toGlm(const Vec4& v) { return glm::vec4(v.x(), v.y(), v.z(), v.w()); }
	static glm::quat toGlm(const Quat& q) {
		return glm::quat(
			static_cast<float>(q.w()),
			static_cast<float>(q.x()),
			static_cast<float>(q.y()),
			static_cast<float>(q.z())
		);
	}
	
	// Converts a 3x3 Eigen matrix to a glm::mat3, taking into account the row-major to column-major conversion
	static glm::mat3 toGlm(const Mat3& m) {
		glm::mat3 g(1.0f);
		for (int c = 0; c < 3; ++c)
			for (int r = 0; r < 3; ++r)
				g[c][r] = static_cast<float>(m(r, c));
		return g;
	}

	// Converts a 4x4 Eigen matrix to a glm::mat4, taking into account the row-major to column-major conversion
	static glm::mat4 toGlm(const Mat4& m) {
		glm::mat4 g(1.0f);
		for (int c = 0; c < 4; ++c)
			for (int r = 0; r < 4; ++r)
				g[c][r] = static_cast<float>(m(r, c));
		return g;
	}

	// --- HELPER METHODS ---

	// Method to clamp a joint angle to its limits
	float RobotSystem::clampJointAngle(const RobotJoint& joint, float angleRad) {
		if (joint.limits.continuous) { return wrapRad(angleRad); }
		else { return glm::clamp(angleRad, joint.limits.minAngle, joint.limits.maxAngle); }
	}

	// Method to wrap an angle in radians to the range [-pi, pi]
	float RobotSystem::wrapToPi(float angleRad) {
		angleRad = std::fmod(angleRad + static_cast<float>(PI_d), static_cast<float>(TWO_PI_d));
		if (angleRad < 0.0f) { angleRad += static_cast<float>(TWO_PI_d); }
		return angleRad - static_cast<float>(PI_d);
	}

	// Method to wrap an angle in radians to the range [0, 2pi]
	float RobotSystem::wrapRad(float angleRad) {
		angleRad = fmod(angleRad, static_cast<float>(TWO_PI_d));
		if (angleRad < 0.0f) { angleRad += static_cast<float>(TWO_PI_d); }
		return angleRad;
	}

	// Method to apply a soft velocity barrier to joint torque using a quadratic "wall" function (basically a softer version of a hard velocity limit)
	static void applyOmegaBarrier(double& tau, double omega, double wMax, double I_eff) {
		if (wMax <= 0.0) return;

		// Check if we're in the "soft zone" near the velocity limit
		const double absw = std::abs(omega);
		const double wSoft = 0.90 * wMax;

		if (absw <= wSoft) return;

		// Check if we're above the hard limit (with some tolerance)
		const double t = (absw - wSoft) / (wMax - wSoft);
		const double T = 0.2; // time constant
		const double wall = (I_eff / T) * (t * t) * (absw - wSoft); // [Nm]

		// Apply opposing torque to reduce |omega|
		tau -= wall * (omega >= 0.0 ? 1.0 : -1.0);
	}

	// Method to compute the inertia tensor of a robot link
	static Mat3 computeLinkInertiaTensor(const RobotLink& link) {
		const robots::Inertia& I = link.inertial.inertia;

		// Construct the inertia tensor matrix
		Mat3 M = Mat3::Zero();
		M << I.ixx, I.ixy, I.ixz,
			 I.ixy, I.iyy, I.iyz,
			 I.ixz, I.iyz, I.izz;

		return M;
	}

	// Method to compute the transformation matrix for a joint motion given its axis and angle
	static Pose jointMotionTransform(const Vec3& axis_joint, double theta) {
		Pose T = Pose::Identity(); // homogeneous transformation matrix (4x4)

		Eigen::AngleAxisd aa(theta, axis_joint.normalized()); // create angle-axis rotation from joint angle and axis
		T.block<3, 3>(0, 0) = aa.toRotationMatrix();		  // set upper-left 3x3 block to rotation matrix
		
		return T;
	}

	// Method to compute the full spatial velocity Jacobian column for a joint
	static Vec6 computeJacobianColumn(RobotMetrics& m, const RobotJoint& joint, const RobotLink& link, const Pose& T_world) {
		Vec3 joint_pos_world = T_world.block<3, 1>(0, 3); // position of joint in world frame
		Mat3 R_world = T_world.block<3, 3>(0, 0);		  // rotation from joint frame to world frame

		Vec3 world_com = R_world * link.inertial.com_xyz + joint_pos_world; // COM position in world frame
		Vec3 world_axis = (R_world * joint.axis).normalized();				// joint axis in world frame

		Vec3 r = world_com - joint_pos_world; // vector from joint to COM in world frame
		Vec3 J_v = world_axis.cross(r);		  // [rad/s], linear velocity Jacobian  (3x1)
		Vec3 J_w = world_axis;				  // [rad/s], angular velocity Jacobian (3x1)

		// Store Jacobians in metrics
		m.Jv = J_v;
		m.Jw = J_w;

		return Vec6(J_v.x(), J_v.y(), J_v.z(), J_w.x(), J_w.y(), J_w.z()); // [rad/s], spatial velocity Jacobian column (6x1)
	}

	// Method to compute the spatial inertia matrix for a link
	static Mat6 computeSpatialInertiaMatrix(double mass, const Mat3& I) {
		Mat6 M = Mat6::Zero(); // spacial inertia matrix (6x6)

		// Upper-left 3x3 block is mass matrix, lower-right 3x3 block is inertia tensor, off-diagonal blocks are zero for point mass assumption
		M.topLeftCorner<3, 3>() = mass * Mat3::Identity(); // mass matrix
		M.bottomRightCorner<3, 3>() = I;				   // inertia tensor

		return M; // [kg, kg*m^2]
	}

	// Method to compute the inertia matrix element for a joint
	static double computeJointInertiaContribution(RobotMetrics& m, const RobotJoint& joint, const RobotLink& link, const Pose& T_world) {
		const double mass = link.inertial.mass; // mass of the link, locally defined

		// Transform inertia tensor to world frame
		Mat3 R = T_world.block<3, 3>(0, 0); // rotation from joint frame to world frame
		Mat3 I_local = computeLinkInertiaTensor(link); // inertia tensor in joint frame, locally defined
		Mat3 I_world = R * I_local * R.transpose(); // inertia tensor in world frame

		// Compute Jacobian column (6x1 vector: [Jv; Jw]) and spatial inertia matrix (6x6) for this link
		Vec6 J_i = computeJacobianColumn(m, joint, link, T_world); // [rad/s]
		Mat6 M_i = computeSpatialInertiaMatrix(mass, I_world);   // [kg, kg*m^2]

		// Effective inertia contribution for this joint: M_ii = J_i^T * M_i * J_i, could also use .coeff(0) since it's a scalar
		// REMINDER: see if .coeff(0) is faster than current method for 1x1 matrices
		double M_ii = (J_i.transpose() * M_i * J_i); // [kg*m^2], [1x6] * [6x6] * [6x1] = [1x1] scalar 

		return std::max(M_ii, 1e-6); // [kg*m^2], avoid zero inertia
	}

	// --- ROBOT STATE INTEGRATION METHODS ---

	// Method to create Object instances for each robot link
	void RobotSystem::instantiateRobotLinks() {
		for (auto& link : _robot.links) {
			auto objs = _loadMeshReturn((paths::assets() / "objects" / "Robotic_Arm_Models" / link.visual.meshFile).string());
			if (objs.empty()) { continue; }

			scene::Object* obj = objs[0];

			obj->category = scene::ObjectCategory::RobotLink;
			obj->transform.scale = glm::vec3(_robot.scale);
			link.attachedObject = obj;

			LOG_INFO_ONCE("Instantiated %zu robot links", _robot.links.size());
			D_INFO_ONCE("Instantiated %zu robot links", _robot.links.size());
		}
	}

	// Method to build a name-to-index map for robot links
	void RobotSystem::buildLinkIndex() {
		_linkIndex.clear();
		for (size_t i = 0; i < _robot.links.size(); i++) {
			_linkIndex[_robot.links[i].name] = (int)i;
			LOG_INFO("Link %zu: %s -> index %d", i, _robot.links[i].name.c_str(), (int)i);
		}
	}

	// Method to pack robot joint states into a state vector
	mathlib::VecX RobotSystem::packState() const {
		const size_t n = static_cast<int>(_robot.joints.size());
		mathlib::VecX x(3 * n);

		// Pack angles and velocities
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Current states
			x[i] = (double)j.thetaRad;
			x[i + n] = (double)j.omegaRad_s;
			x[i + 2 * n] = (double)j.eta; // integral state
		}
		return x;
	}
	
	// Method to unpack state vector into robot joints
	void RobotSystem::unpackState(const mathlib::VecX& x) {
		const size_t n = static_cast<int>(_robot.joints.size());

		// Resize clamping vectors if necessary
		if (_clampTheta.size() != n) { _clampTheta.assign(n, 0); }
		if (_clampOmega.size() != n) { _clampOmega.assign(n, 0); }

		// For each joint
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Current states
			float theta_in = (float)x[i];
			float omega_in = (float)x[i + n];
			float eta_in   = (float)x[i + 2 * n];

			// Clamp joint angle
			float theta_out = clampJointAngle(j, theta_in);

			// max |omega|
			float wMax_hw = std::abs(j.limits.maxOmegaRad_s);
			float omega_out = omega_in;

			// Velocity limit clamping
			if (wMax_hw > 0.0f) {
				const float eps = 0.05f;
				if (std::abs(omega_in) > (1.0f + eps) * wMax_hw) {
					omega_out = glm::clamp(omega_in, -wMax_hw, wMax_hw);
				}
			}

			// Velocity limit enforcement
			if (theta_out != theta_in) {
				const float upperLimit = j.limits.maxAngle;
				const float lowerLimit = j.limits.minAngle;

				if (theta_out >= upperLimit && omega_in > 0.0f) { omega_out = 0.0f; }
				if (theta_out <= lowerLimit && omega_in < 0.0f) { omega_out = 0.0f; }
			}

			// Record clamping
			_clampTheta[i] = (theta_in != theta_out) ? 1 : 0;
			_clampOmega[i] = (omega_in != omega_out) ? 1 : 0;

			// Update joint states
			j.thetaRad = theta_out;
			j.omegaRad_s = omega_out;
			j.eta = eta_in;
		}
	}

	// Method to pack reference state vector (target angles and velocities) for control
	mathlib::VecX RobotSystem::packRefState() const {
		const size_t n = (int)_robot.joints.size();
		mathlib::VecX x(2 * n);
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Pack reference angles and velocities
			x[i] = (double)j.thetaRefRad;
			x[i + n] = (double)j.omegaRefRad_s;
		}
		return x;
	}

	// Method to unpack reference state vector into robot joints
	void RobotSystem::unpackRefState(const mathlib::VecX& x) {
		const size_t n = (int)_robot.joints.size();
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];
			j.thetaRefRad = (float)x[i];
			j.omegaRefRad_s = (float)x[i + n];
			j.thetaRefRad = clampJointAngle(j, j.thetaRefRad);
		}
	}

	// Method to compute forward kinematics for all links given joint angles
	std::vector<Pose> RobotSystem::computeForwardKinematics_fromState(const VecX& x) const {
		const auto& joints = _robot.joints;
		const auto& links = _robot.links;
		const size_t n = (size_t)joints.size();

		std::vector<Pose> T_world;
		T_world.reserve((size_t)links.size());

		Pose T = Pose::Identity(); // world -> base
		T_world.push_back(T);	   // base link 

		for (size_t i = 0; i < n; ++i) {
			const auto& joint = joints[i];

			const double theta = x[i]; // joint angle from state vector

			Pose T_origin = Pose::Identity(); // transform from parent link to joint frame (fixed)
			T_origin.block<3, 3>(0, 0) = joint.origin_q.toRotationMatrix(); // rotation from parent link frame to joint frame, derived from rpy in JSON
			T_origin.block<3, 1>(0, 3) = joint.origin_xyz;					// translation from parent link to joint frame
			
			// Compute joint motion transform based on joint axis and angle
			Pose T_motion = Pose::Identity();
			if (joint.type == eJointType::REVOLUTE) {
				T_motion = jointMotionTransform(joint.axis, theta); // rotation about joint axis
			}
			else if (joint.type == eJointType::PRISMATIC) {
				T_motion.block<3, 1>(0, 3) = joint.axis.normalized() * theta; // translation along joint axis
			}

			// compose transforms: parent -> joint -> motion -> child
			T = T * T_origin * T_motion;

			T_world.push_back(T); // link i+1 pose in world frame
		}

		return T_world;
	}

	void RobotSystem::computeI_eff(const std::vector<double>& theta, std::vector<double>& I_eff_out) const {
		const size_t n = _robot.joints.size();
		I_eff_out.assign(n, 0.0);

		// Use existing state packing
		mathlib::VecX x = packState();
		for (size_t i = 0; i < n; ++i) {
			x[i] = theta[i];  // overwrite positions
		}

		// Forward kinematics
		std::vector<Pose> T_world = computeForwardKinematics_fromState(x);

		// Accumulate inertia per joint
		for (size_t i = 0; i < n; ++i) {

			double I_eff_i = 0.0;
			const RobotJoint& joint = _robot.joints[i];

			for (size_t k = 0; k < _robot.links.size(); ++k) {

				const RobotLink& link = _robot.links[k];

				RobotMetrics m;
				computeJacobianColumn(m, joint, link, T_world[k]);

				// Translational contribution
				I_eff_i += link.inertial.mass * m.Jv.squaredNorm();
				// Rotational contribution (scalar inertia)
				I_eff_i += link.inertial.inertia.izz * m.Jw.squaredNorm();
			}

			I_eff_out[i] = std::max(I_eff_i, 1e-6);
		}
	}

	std::vector<double> RobotSystem::computeCoriolisDiagonal(const std::vector<double>& theta, const std::vector<double>& omega, const std::vector<double>& I_eff) {
		const size_t n = (size_t)_robot.joints.size();
		std::vector<double> tau_C(n, 0.0);

		constexpr double eps = 1e-6;

		for (size_t i = 0; i < n; ++i) {
			double sum = 0.0;

			for (size_t j = 0; j < n; ++j) {
				// Compute Christoffel symbols (simplified for diagonal inertia)
				std::vector<double> theta_pert = theta;
				theta_pert[j] += eps;

				// Compute perturbed inertia matrix diagonal element
				std::vector<double> I_eff_pert = I_eff;
				computeI_eff(theta_pert, I_eff_pert);

				double dI_omega_j = (I_eff_pert[i] - I_eff[i]) / eps;
				sum += dI_omega_j * omega[j];
			}
			// C_i = 0.5 * sum_jk (dI_ij/dtheta_k + dI_ik/dtheta_j - dI_jk/dtheta_i) * omega_j * omega_k, simplified to diagonal case
			tau_C[i] = 0.5 * sum * omega[i]; // only consider terms where j=k=i for diagonal inertia			
		}
		return tau_C;
	}

	// Method to compute joint metrics for control
	RobotMetrics RobotSystem::computeJointMetrics(
		const RobotJoint& joint, const RobotLink& link, double I_eff, 
		double theta, double omega, 
		double thetaRef, double omegaRef, double alphaRef, 
		double eta, double tau_coriolis
	) const {
		RobotMetrics m{};

		// Constants
		m.mass = link.inertial.mass;
		m.g = _gravity; // 0 for now

		// Current states
		m.theta = theta;
		m.omega = omega;
		m.eta = eta;
		// Reference states
		m.thetaRef = thetaRef;
		m.omegaRef = omegaRef;
		m.alphaRef = alphaRef;

		// Errors
		m.err   = m.thetaRef - theta;
		m.err_d = m.omegaRef - omega;

		// Effective inertia
		m.I_eff = I_eff;

		// Joint-Space Model
		// M(q) = inertia matrix, C(q, qdot) = Coriolis/centrifugal, g(q) = gravity

		// Control parameters
		const double wn = (double)joint.wn_target;
		const double z  = (double)joint.zeta_target;
		const double b  = (double)joint.beta_target;

		// Compute PID gains
		double k_p = m.I_eff * wn * wn;
		double k_i = b * k_p * wn;
		double k_d = 2.0 * z * m.I_eff * wn;

		// Store gains
		m.kp = k_p;
		m.ki = k_i;
		m.kd = k_d;

		// Integral term
		double tau_i = k_i * eta;

		// Integral anti-windup
		if (joint.limits.maxEffort > 0.0f) {
			const double rho = 0.3; // fraction of max effort allocated to I-term
			const double tau_i_max = rho * (double)joint.limits.maxEffort;
			tau_i = std::clamp(tau_i, -tau_i_max, tau_i_max);
		}

		tau_i = 0.0; // disable I-term for now (testing)

		// Inverse dynamics control law (PD + feedforward)
		double tau_control = (k_p * m.err + tau_i + k_d * m.err_d) + m.I_eff * m.alphaRef; // control torque

		// Passive dynamics
		const double c = (double)joint.dynamics.damping;
		const double mu = (double)joint.dynamics.friction;
		const double v_eps = 1e-2; // small velocity threshold

		m.c = c;
		m.mu = mu;

		// Friction model (viscous + Coulomb/Stribeck)
		double tau_damping{ 0.0 }, tau_friction{ 0.0 };
		tau_damping  = c * omega; // viscous damping
		tau_friction = mu * std::tanh(omega / v_eps); // Coulomb friction
		
		// Cache torques in metrics
		m.tau_control = tau_control;
		m.tau_damping  = tau_damping;
		m.tau_friction = tau_friction;
		m.tau_coriolis = tau_coriolis;

		// Net torque
		m.tau = tau_control - tau_damping - tau_friction - tau_coriolis;

		double tau_preSat = m.tau;

		// Effort clamp
		if (joint.limits.maxEffort > 0.0f) {
			const double E_max = joint.limits.maxEffort;
			m.tau = std::clamp(m.tau, -E_max, E_max);
		}

		m.tau_sat = tau_preSat - m.tau;
		m.sat_flag = (m.tau_sat != 0.0);

		// Velocity soft limit
		const double wMax_hw = std::abs(joint.limits.maxOmegaRad_s);
		const double wMax_traj = std::abs(joint.limits.omegaRefMaxRad_s); // or derived from trajectory manager

		double tau_preBarrier = m.tau;

		// Apply soft velocity barrier
		applyOmegaBarrier(m.tau, omega, wMax_hw, m.I_eff);

		m.tau_barrier = tau_preBarrier - m.tau;

		m.wMax_hw = wMax_hw;
		m.wMax_traj = wMax_traj;
		m.traj_overspeed = std::max(0.0, std::abs(omega) - wMax_traj);
		m.traj_overspeed_flag = (m.traj_overspeed > 0.05); // 0.05 rad/s threshold

		// Final angular acceleration
		m.alpha = m.tau / m.I_eff;

		return m;
	}

	// Derivative function for ODE integration
	mathlib::VecX RobotSystem::deriv(const control::TrajectoryManager& traj, double t, const mathlib::VecX& x, const std::vector<double>& I_eff, const std::vector<double>& tau_coriolis) const {
		const size_t n = static_cast<int>(_robot.joints.size());
		mathlib::VecX dx(3 * n);

		// Compute forward kinematics for current state to get link poses and Jacobians needed for dynamics
		std::vector<Pose> T_world = computeForwardKinematics_fromState(x); // compute forward kinematics for current state

		// Loop through each joint and compute derivatives
		for (size_t i = 0; i < n; ++i) {
			// Current states
			const double theta = x[i];
			const double omega = x[i + n];
			const double eta   = x[i + 2 * n];

			// Current joint
			const RobotJoint& joint = _robot.joints[i];
			const RobotLink& link   = _robot.links[i+1];

			// Use integrated reference (baseline truth)
			const double thetaRef = (double)joint.thetaRefRad;
			const double omegaRef = (double)joint.omegaRefRad_s;
			const double alphaRef = (double)joint.alphaRefRad_s2;

			// Compute joint metrics
			RobotMetrics m = computeJointMetrics(
				joint, link, I_eff[i], 
				theta, omega, 
				thetaRef, omegaRef, alphaRef, 
				eta, tau_coriolis[i]
			);

			// Fill in derivatives
			dx[i]		  = omega;	 // dtheta/dt = omega
			dx[i + n]     = m.alpha; // domega/dt = alpha
			dx[i + 2 * n] = m.err; 	 // deta/dt   = e(t)
		}
		return dx;
	}

	// Method to enforce joint limits after integration
	void RobotSystem::enforceJointLimits(RobotJoint& j) {
		if (j.limits.continuous) { return; }

		const float lo = j.limits.minAngle;
		const float hi = j.limits.maxAngle;

		if (j.thetaRad < lo) { j.thetaRad = lo; if (j.omegaRad_s < 0.0f) { j.omegaRad_s = 0.0f; }}
		if (j.thetaRad > hi) { j.thetaRad = hi; if (j.omegaRad_s > 0.0f) { j.omegaRad_s = 0.0f; }}
	}

	// Method to advance the robot state by dt using the selected integrator
	void RobotSystem::step(const control::TrajectoryManager& traj, double dt, double simTime) {
		if (!_hasRobot) return;
		const size_t n = _robot.joints.size();
		_simTime = simTime;

		// FK needed for inertia
		mathlib::VecX x = packState();
		std::vector<Pose> T_world = computeForwardKinematics_fromState(x);

		// Caches for inertia and Coriolis
		std::vector<double> I_eff(n, 0.0);
		std::vector<double> q(n), qd(n);

		// Compute effective inertia for each joint at current state
		for (size_t i = 0; i < n; ++i) {
			q[i] = _robot.joints[i].thetaRad;
			qd[i] = _robot.joints[i].omegaRad_s;

			for (size_t k = 0; k < _robot.links.size(); ++k) {
				RobotMetrics tmp;
				I_eff[i] += computeJointInertiaContribution(
					tmp,
					_robot.joints[i],
					_robot.links[k],
					T_world[k]
				);
			}
			I_eff[i] = std::max(I_eff[i], 1e-6);
		}

		// Compute Coriolis torques for current state
		std::vector<double> tau_coriolis = computeCoriolisDiagonal(q, qd, I_eff);

		// Define the derivative function
		auto f = [&](double t, const mathlib::VecX& xIn) { return deriv(traj, t, xIn, I_eff, tau_coriolis); };
		mathlib::VecX x_Next = _integrator->stepODE(_curIntMethod, x, simTime, dt, f);

		// Unpack new state
		unpackState(x_Next);

		// Enforce joint limits
		for (auto& j : _robot.joints) {
			const float wMax = j.limits.maxOmegaRad_s;
			if (wMax > 0.0f) { j.omegaRad_s = glm::clamp(j.omegaRad_s, -wMax, wMax); }
			enforceJointLimits(j);
		}

		for (size_t i = 0; i < n; ++i) {
			const auto& joint = _robot.joints[i];
			const auto& link  = _robot.links[i + 1];

			// Current states
			const double theta = (double)joint.thetaRad;
			const double omega = (double)joint.omegaRad_s;

			// Use integrated reference (baseline truth)
			const double thetaRef = (double)joint.thetaRefRad;
			const double omegaRef = (double)joint.omegaRefRad_s;
			const double alphaRef = (double)joint.alphaRefRad_s2;

			// Compute joint metrics
			RobotMetrics m = computeJointMetrics(
				joint, link, I_eff[i], 
				theta, omega, 
				thetaRef, omegaRef, alphaRef, 
				0, tau_coriolis[i]
			);

			const std::string IntName = _integrator->IntegratorName(_curIntMethod);
			std::string header = "simulation_" + _robot.name + "_" + IntName;

			HDF5_SIM_DATA(header, (data::FieldList{
					// Simulation info
					{"sim_time", simTime},
					{"dt",       dt},
					//  Joint identification
					{"joint_name", std::string(joint.name)},
					{"link_name",  std::string(joint.child)},
					// Limit values
					{"minAngle",	   (double)joint.limits.minAngle},
					{"maxAngle",	   (double)joint.limits.maxAngle},
					{"wMax_hw",		   m.wMax_hw},
					{"wMax_traj",	   m.wMax_traj},
					{"traj_overspeed", m.traj_overspeed},
					{"maxEffort",	   (double)joint.limits.maxEffort},
					// Torque values
					{"torque",          m.tau},
					{"torque_motor",    m.tau_control},
					{"torque_damping",  m.tau_damping},
					{"torque_friction", m.tau_friction},
					{"torque_coriolis", m.tau_coriolis},
					{"torque_barrier",  m.tau_barrier},
					{"torque_sat",	    m.tau_sat},
					// Dynamics values
					{"damping",  m.c},
					{"friction", m.mu},
					{"I_eff",    m.I_eff},
					// Control gains
					{"k_p",	m.kp},
					{"k_i",	m.ki},
					{"k_d",	m.kd},
					// States
					{"theta",	  m.theta},
					{"theta_ref", m.thetaRef},
					{"omega",	  m.omega},
					{"omega_ref", m.omegaRef},
					{"alpha",	  m.alpha},
					{"alpha_ref", m.alphaRef},
					{"err",		  m.err},
					{"err_d",	  m.err_d},
					// Clamping flags
					{"clamp_theta", (double)_clampTheta[i]},
					{"clamp_omega", (double)_clampOmega[i]},
					{ "sat_flag",  (double)m.sat_flag },
					{"traj_overspeed_flag", (double)m.traj_overspeed_flag}
				})
			);
		}

		// Update kinematics
		updateRobotKinematics();
	}

	void RobotSystem::stepReference(control::TrajectoryManager& traj, double dt, double t) {
		if (!_hasRobot) return;
		
		const int n = (int)_robot.joints.size();
		if (n <= 0) { return; }

		// Sample trajectories ("ground truth" inputs)
		for (int i = 0; i < n; ++i) {
			RobotJoint& j = _robot.joints[i];
			control::TrajState s{};
			// Try to evaluate trajectory
			if (traj.tryEval(std::string(j.child), t, s)) {
				j.thetaRefRad = clampJointAngle(j, (float)s.q); // set ref angle
				j.omegaRefRad_s = (float)s.qd;
				j.alphaRefRad_s2 = (float)s.qdd;
			}
			// Store inputs
			else {
				j.alphaRefRad_s2 = 0.0f;
				j.omegaRefRad_s = 0.0f;
			}

			const std::string IntName = _integrator->IntegratorName(_curIntMethod);
			std::string header = "reference_" + _robot.name + "_" + IntName;

			HDF5_REF_DATA(header,
				(data::FieldList{
					{"sim_time", t},
					{"dt", dt},
					{"joint_name", std::string(j.name)},
					{"joint_child", std::string(j.child)},
					{"joint_parent", std::string(j.parent)},
					// "traj_*" == the truth in Option A
					{"traj_theta_ref", (double)j.thetaRefRad},
					{"traj_omega_ref", (double)j.omegaRefRad_s},
					{"traj_alpha_ref", (double)j.alphaRefRad_s2},
					// "theta_ref" etc also same truth
					{"theta_ref", (double)j.thetaRefRad},
					{"omega_ref", (double)j.omegaRefRad_s},
					{"alpha_ref", (double)j.alphaRefRad_s2},
				})
			);
		}
	}

	// --- ROBOT LOADING AND RESET METHODS ---

	// Method to load a robot model by name
	void RobotSystem::loadRobot(const std::string& name) {
		clearRobot();

		const std::filesystem::path jsonPath = paths::assets() / "objects" / "Robotic_Arm_Models" / name / (name + ".json");

		if (!std::filesystem::exists(jsonPath)) {
			LOG_ERROR("Robot JSON file not found -> %s", jsonPath.string().c_str());
			D_ERROR("Robot JSON file not found -> %s", jsonPath.string().c_str());
			return;
		}

		_robot = robots::RobotLoader::loadFromJSON(jsonPath.string());
		_hasRobot = true;
		_loadedName = name;

		_robotRootHome = glm::mat4(1.0f);
		_robotRootHome = glm::rotate(_robotRootHome, glm::radians(-90.0f), glm::vec3(1, 0, 0));
		_robotRootHome = glm::translate(_robotRootHome, glm::vec3(0.0f, 0.0f, 0.0f));
		_robotRootPose = _robotRootHome;

		_robotQHome = _robot.makeJointVector();
		_robotHomeValid = true;

		instantiateRobotLinks();
		buildLinkIndex();

		resetRobot();

		LOG_INFO("Loaded robot model -> %s", name.c_str());
		D_SUCCESS("Loaded robot model -> %s", name.c_str());
	}

	// Method to reset the robot to its home position
	void RobotSystem::resetRobot() {
		if (!_hasRobot || !_robotHomeValid) { return; }
		_robotRootPose = _robotRootHome;
		_robot.setJointVector(_robotQHome);

		for (auto& joint : _robot.joints) {
			joint.omegaRad_s = 0.0f;
			joint.thetaRefRad = joint.thetaRad;
			joint.omegaRefRad_s = 0.0f;
			joint.alphaRefRad_s2 = 0.0f;
		}
		updateRobotKinematics();
		D_INFO("Robot reset to home position.");
		D_SUCCESS("Robot reset to home position.");
	}

	// Method to clear the current robot from the scene
	void RobotSystem::clearRobot() {
		if (!_hasRobot) return;

		// Remove robot objects from _objects
		for (auto& link : _robot.links) {
			if (auto* dead = link.attachedObject) {
				// find and erase matching object
				_objects.erase(
					std::remove_if(_objects.begin(), _objects.end(),
						[&](const std::unique_ptr<scene::Object>& obj) { return obj.get() == dead; }),
					_objects.end()
				);
				link.attachedObject = nullptr; // clear pointer
			}
		}

		_robot.links.clear();
		_robot.joints.clear();
		_linkIndex.clear();
		_hasRobot = false;

		LOG_INFO("Old robot model removed");
		D_WARN("Old robot model removed");
	}

	void RobotSystem::stopAll() {
		if (!_hasRobot) return;
		for (auto& joint : _robot.joints) {
			joint.omegaRad_s = 0.0f;
			joint.thetaRefRad = joint.thetaRad;
		}
	}

	// --- ROBOT KINEMATICS AND JOINT STATE METHODS ---

	std::string RobotSystem::findRootLink() const {
		std::unordered_set<std::string> children;
		for (const auto& joint : _robot.joints) { children.insert(joint.child); }
		for (const auto& link : _robot.links) {
			if (children.find(link.name) == children.end()) {
				return link.name;
			}
		}
		return _robot.links.empty() ? "" : _robot.links.front().name; // fallback
	}

	void RobotSystem::updateRobotKinematics() {
		if (!_hasRobot) return;
		std::vector<glm::mat4> world(_robot.links.size(), glm::mat4(1.0f));

		// Find root link
		const std::string rootName = findRootLink();
		auto itRoot = _linkIndex.find(rootName);
		if (itRoot == _linkIndex.end()) {
			LOG_WARN_ONCE("RobotSystem::updateRobotKinematics: root link '%s' not found in link index", rootName.c_str());
			return;
		}

		// Set root link pose
		int rootIdx = itRoot->second;
		world[rootIdx] = _robotRootPose;

		// parent -> children joints
		std::unordered_map<std::string, std::vector<const RobotJoint*>> children;
		children.reserve(_robot.joints.size());
		for (const auto& j : _robot.joints) children[j.parent].push_back(&j);

		std::stack<std::string> st;
		st.push(rootName);

		while (!st.empty()) {
			std::string parentName = st.top(); st.pop();
			auto itP = _linkIndex.find(parentName);

			// Skip if parent link not found
			if (itP == _linkIndex.end()) { continue; }
			int pIdx = itP->second;

			// Find children joints
			auto it = children.find(parentName);
			if (it == children.end()) continue;

			// For each child joint
			for (const RobotJoint* jp : it->second) {
				const RobotJoint& j = *jp;
				auto itC = _linkIndex.find(j.child);
				if (itC == _linkIndex.end()) { continue; }
				int cIdx = itC->second;

				Vec4 axis = Vec4(j.axis.x(), j.axis.y(), j.axis.z(), 0.0);

				glm::mat4 T = glm::translate(glm::mat4(1.0f), toGlm(j.origin_xyz));
				glm::mat4 R0 = glm::mat4_cast(toGlm(j.origin_q));
				glm::vec3 axis_joint = glm::normalize(toGlm(j.axis));
				glm::mat4 Rq = glm::rotate(glm::mat4(1.0f), j.thetaRad, axis_joint);

				// Apply joint rotation in JOINT frame
				world[cIdx] = world[pIdx] * T * R0 * Rq;

				st.push(j.child);
			}
		}

		for (int i = 0; i < (int)_robot.links.size(); ++i) {
			if (auto* obj = _robot.links[i].attachedObject) {
				if (auto* mesh = obj->getMesh()) { mesh->localTransform = world[i]; }
			}
		}
	}

	// Method to get the angle of a specific robot joint
	bool RobotSystem::tryGetJointAngleRad(const std::string& childLink, float& outAngle) const {
		if (!_hasRobot) { return false; }
		// Find joint child matching childLink
		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				outAngle = joint.thetaRad;
				return true;
			}
		}
		return false;
	}

	// Method to set the angle of a specific robot joint
	bool RobotSystem::trySetJointAngleRad(const std::string& childLink, float angleRad) {
		if (!_hasRobot) { return false; }
		// Find joint child matching childLink
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.thetaRad = clampJointAngle(joint, angleRad); // clamp to joint limits
				return true;
			}
		}
		return false;
	}

	// Method to get the angular velocity of a specific robot joint
	bool RobotSystem::tryGetJointOmegaRad(const std::string& childLink, float& outOmega) const {
		if (!_hasRobot) { return false; }
		// Find joint child matching childLink
		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				outOmega = joint.omegaRad_s;
				return true;
			}
		}
		return false;
	}

	// Method to set the angular velocity of a specific robot joint
	bool RobotSystem::trySetJointOmegaRad(const std::string& childLink, float omegaRad) {
		if (!_hasRobot) { return false; }
		// Find joint child matching childLink
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.omegaRad_s = omegaRad;
				return true;
			}
		}
		return false;
	}

	// Method to get the target angle (reference) of a specific robot joint in radians
	bool RobotSystem::tryGetJointTargetRad(const std::string& childLink, float& outTargetRad) const {
		if (!_hasRobot) { return false; }
		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				outTargetRad = joint.thetaRefRad;
				return true;
			}
		}
		return false;
	}

	// Method to set the target angle (reference) of a specific robot joint in radians
	bool RobotSystem::trySetJointTargetRad(const std::string& childLink, float targetRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				if (joint.limits.continuous) { joint.thetaRefRad = wrapRad(targetRad); }
				else { joint.thetaRefRad = clampJointAngle(joint, targetRad); } // clamp to joint 
				return true;
			}
		}
		return false;
	}

	//	Method to get the maximum angular velocity of a specific robot joint in radians
	bool RobotSystem::tryGetJointOmegaMaxRad(const std::string& childLink, float& maxOmegaRad) const {
		if (!_hasRobot) { return false; }
		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				maxOmegaRad = joint.limits.maxOmegaRad_s;
				return true;
			}
		}
		return false;
	}

	//	Method to set the maximum angular velocity of a specific robot joint in radians
	bool RobotSystem::trySetJointOmegaMaxRad(const std::string& childLink, float maxOmegaRad) {
		if (!_hasRobot) { return false; }
		if (maxOmegaRad <= 0.0f) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.limits.maxOmegaRad_s = maxOmegaRad;
				return true;
			}
		}
		return false;
	}

	// Method to increment the target angle (reference) of a specific robot joint in radians
	bool RobotSystem::tryAddJointTargetRad(const std::string& childLink, float deltaRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				float t = joint.thetaRefRad + deltaRad;
				if (joint.limits.continuous) { t = wrapRad(t); }
				else { t = glm::clamp(t, joint.limits.minAngle, joint.limits.maxAngle); }
				joint.thetaRefRad = t; // clamp to joint limits
				return true;
			}
		}
		return false;
	}

	// Method to set the reference angular velocity of a specific robot joint in radians
	bool RobotSystem::trySetJointOmegaRefRad(const std::string& childLink, float omegaRefRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.omegaRefRad_s = omegaRefRad;
				return true;
			}
		}
		return false;
	}

	// Method to set the reference angular acceleration of a specific robot joint in radians
	bool RobotSystem::trySetJointAlphaRefRad(const std::string& childLink, float alphaRefRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.alphaRefRad_s2 = alphaRefRad;
				return true;
			}
		}
		return false;
	}

	// Method to set the max Omega reference of a specific robot joint in radians
	bool RobotSystem::trySetJointOmegaRefMaxRad(const std::string& childLink, float maxOmegaRad) {
		if (!_hasRobot) { return false; }
		if (maxOmegaRad <= 0.0f) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.limits.omegaRefMaxRad_s = maxOmegaRad;
				return true;
			}
		}
		return false;
	}

	// Method to zero the reference derivatives (velocity and acceleration) of a specific robot joint
	bool RobotSystem::tryZeroJointRefDerivatives() {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			joint.omegaRefRad_s = 0.0f;
			joint.alphaRefRad_s2 = 0.0f;
		}
		return true;
	}

	// Method to check if a specific robot joint is at its target angle within a tolerance (radians)
	bool RobotSystem::isJointAtTargetRad(const std::string& childLink, float tolRad) const {
		if (!_hasRobot) { return false; }
		if (tolRad < 0.0f) { tolRad = -tolRad; }

		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				float err = joint.thetaRefRad - joint.thetaRad;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				err = std::abs(err);
				return err <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific robot joint is at its target angle within a tolerance (degrees)
	bool RobotSystem::isJointAtTargetDeg(const std::string& childLink, float tolDeg) const { 
		return isJointAtTargetRad(childLink, glm::radians(tolDeg)); 
	}

	// Method to check if a specific robot joint is near a target angle within a tolerance (radians)
	bool RobotSystem::isJointNearAngleRad(const std::string& childLink, float targetRad, float tolRad) const {
		if (!_hasRobot) { return false; }
		tolRad = std::abs(tolRad);

		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				float err = targetRad - joint.thetaRad;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				return std::abs(err) <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific robot joint is near a target angle within a tolerance (degrees)
	bool RobotSystem::isJointNearAngleDeg(const std::string& childLink, float targetDeg, float tolDeg) const { 
		return isJointNearAngleRad(childLink, glm::radians(targetDeg), glm::radians(tolDeg));
	}

	// --- ROBOT LINK AND ROOT POSE METHODS ---

	// Method to set the rotation angle of a specific robot link angle in degrees
	bool RobotSystem::setRobotLinkRotation(const std::string& childLinkName, float angleDeg) {
		for (auto& j : _robot.joints) {
			if (j.child == childLinkName) {
				j.thetaRad = glm::radians(angleDeg);
				updateRobotKinematics();
				return true;
			}
		}
		return false;
	}

	// Method to set the robot root pose in world coordinates
	void RobotSystem::setRobotRootPose(const glm::vec3& pos, const glm::quat& rot) {
		glm::mat4 T = glm::translate(glm::mat4(1.0f), pos);
		glm::mat4 R = glm::mat4_cast(rot);
		glm::mat4 Align = glm::rotate(glm ::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1, 0, 0));
		_robotRootPose = (T * R) * Align;
	}

	// Method to set the robot root home pose in world coordinates
	void RobotSystem::setRobotRootHome(const glm::vec3& pos, const glm::quat& rot) {
		glm::mat4 T = glm::translate(glm::mat4(1.0f), pos);
		glm::mat4 R = glm::mat4_cast(rot);
		glm::mat4 Align = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1, 0, 0));
		_robotRootPose = (T * R) * Align;
		_robotRootPose = _robotRootHome;
	}

	// Method to set the default pose of the robot using joint angles in degrees
	bool RobotSystem::setDefaultPoseDeg(const std::vector<float>& qDeg) {
		if (!_hasRobot) { return false; }
		if (qDeg.size() != _robot.joints.size()) { return false; }
		for (size_t i = 0; i < _robot.joints.size(); ++i) {
			_robot.joints[i].thetaRad = glm::radians(qDeg[i]);
		}
		_robotQHome = _robot.makeJointVector();
		_robotHomeValid = true;
		return true;
	}
} // namespace robots