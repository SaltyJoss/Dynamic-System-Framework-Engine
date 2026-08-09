/*
 * File: Systems/RigidBodySystem.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include "Systems/RigidBodyModel.h"

#include "Systems/SpatialModel.h"
#include "Systems/RigidBodySnapshot.h"
#include "Physics/DynamicsTypes.h"

#include <kinematics/Forward_Kinematics.h>
#include "Physics/RigidBodyKinematics.h"
#include "Physics/RigidBodyDynamics.h"
#include "Physics/SpatialDynamics.h"

#include "Analysis/MetricLogger.h"
#include "Numerics/IntegrationService.h"

// Forward declarations
namespace control { class TrajectoryManager; }

namespace systems {
	// Joint state structure
    struct DSFE_API JointState {
		double theta;
		double omega;
	};
	// Metrics structure
	enum class eRole {
		Simulation,
		Baseline
	};
	// Step Result struct
	template<typename Scalar>
	struct RigidBodyStepResult_T {
		integration::StepOut_T<Scalar> stepOut;
		RigidBodySnapshot_T<Scalar> snap;
		physics::DynamicsResult<Scalar> dynamics;
		mathlib::VecX_T<Scalar> tau_rnea;
	};
	// Type alias for double precision step result
	inline constexpr size_t AD_VARS = 14; // number of independent variables for autodiff (used for pre-allocating AD integrator buffers)
	// Inline function to convert an exponential map (rotation vector) to a quaternion
	inline mathlib::Quat expToQuat(const mathlib::Vec3& rv) {
		const double theta = rv.norm();
		if (theta < 1e-9) { return mathlib::Quat(1, 0, 0, 0); }
		return mathlib::Quat(Eigen::AngleAxisd(theta, rv / theta));
	}

	/// @brief RigidBodySystem class encapsulates the state and dynamics of a rigid body system, including its links, joints, and physical properties. 
	/// It provides methods for accessing and manipulating the system's state, computing kinematics and dynamics, and handling external forces.
	class DSFE_API RigidBodySystem {
	public:
		RigidBodySystem();
		~RigidBodySystem();

        /*
		 * Joint Angle Clamping
		 */
        static double clampJointAngle(const RigidBodyJoint& joint, double angleRad);
		template<typename T>
		static T clampJointAngle_T(const RigidBodyJoint& joint, T angleRad);

        /*
		 * RigidBodySystem Accessors
		 */
		// Model accessors
		const systems::RigidBodyModel& model() const;
		const std::vector<Mat4>& worldTransforms() const { return _worldTransforms; }
		// System accesors
		const RigidBodySystem& getRigidBody() const { return *this; }
		const std::string& rigidBodyName() const { return _body.name; }
        bool hasRigidBody() const { return _hasBody; }
		// Body link accessors
        const std::vector<RigidBodyLink>& links() const { return _body.links; }
		std::vector<RigidBodyLink>& links() { return _body.links; }
		// Body joint accessors
        const std::vector<RigidBodyJoint>& joints() const { return _body.joints; }
		std::vector<RigidBodyJoint>& joints() { return _body.joints; }
		// Link accessors
		bool hasLinkName(const std::string& linkName) const { return _link_idx.find(linkName) != _link_idx.end(); }
		std::vector<std::string> linkNames() const;
		std::size_t linkCount() const { return _body.links.size(); }
		std::string findRootLink() const;
		// Joint accessors
		bool hasJointName(const std::string& jointName) const { return _joint_idx.find(jointName) != _joint_idx.end(); }
		std::vector<std::string> jointNames() const;
		std::size_t jointCount() const { return _body.joints.size(); }
        // Joint state accessors
		bool hasFreeJoint() const;
		bool latestFreeBodyEntry(FreeBodyLogBuffer::FreeBodyLogEntry& out, int bodyIdx) const;
		int jointStateOffset(size_t joint_idx) const;
		int totalDOF() const;

		/*
		 * Physical Property Accessors
		 */
		// Mass and Inertia accessors
		const double mass() const { return _body.links[0].inertial.mass; }
		const std::vector<mathlib::Mat3>& linkInertias() const;
		const mathlib::Mat3& linkInertia(int idx) const;
		// Body restitution accessors
		void setRestitution_d(double r, int idx = 0) { _restitution[idx] = r; }
		void setRestitution(std::vector<double> r) { _restitution = r; }
		const double restitution_d(int idx = 0) const { return _restitution[idx]; }
		const std::vector<double> restitution() const { return _restitution; }
		// Body friction accessors
		void setFriction_d(double f, int idx = 0) { _friction[idx] = f; }
		void setFriction(std::vector<double> f) { _friction = f; }
		const double friction_d(int idx = 0) const { return _friction[idx]; }
		const std::vector<double> friction() const { return _friction; }
		// Gravity accessors (Naming needs to be updated)
		void setGravity(double g);
		void setGravityVec(const mathlib::Vec3& g);
		const mathlib::Vec3& getGravityVec() const { return _gravity; }
		double getGravity() const { return _gravity.norm(); }
		// Natural Frequency accesors (Needs to be updated)
		void setNaturalFrequency(double wn) { _wn = wn; }
		double getNaturalFrequency() const { return _wn; }
		void resetNaturalFrequencyToTarget() { for (auto& joint : _body.joints) { joint.wn_target = _wn; } }
		// Damping ratio accessors (Needs to be updated)
		void setDampingRatio(double zeta) { _zeta = zeta; }
		double getDampingRatio() const { return _zeta; }
		void resetDampingRatioToTarget() { for (auto& joint : _body.joints) { joint.zeta_target = _zeta; } }
		// FreeBody retrieval accesors;
		const mathlib::Mat3& inertia_fb() const;
		const mathlib::Vec3& position_fb() const;
		const mathlib::Quat& orientation_fb() const;
		const mathlib::Vec3& linearVelocity_fb() const;
		const mathlib::Vec3& angularVelocity_fb() const;
		// FreeBody state accessors
		bool state_fb(mathlib::Vec3& pos, mathlib::Quat& orient, mathlib::Vec3& linVel, mathlib::Vec3& angVel) const;
		bool setVelocity_fb(const mathlib::Vec3& linVel, const mathlib::Vec3& angVel);
		bool setPosition_fb(const mathlib::Vec3& pos); // for penetration corrections
		bool massInertia_fb(double& mass, mathlib::Mat3& I_body) const; // body frame inertia
		bool localAABB_fb(mathlib::Vec3& aabbMin, mathlib::Vec3& aabbMax) const;

		// Role accessors
		eRole role() const { return _role; }
		void setRole(eRole role) { _role = role; }


		/*
		 * External Force Accessors
		 */
		bool linkWorldOrigin(const std::string& linkName, mathlib::Vec3& out) const;
		bool setLinkExtForce(const std::string& linkName, const mathlib::Vec3& worldPoint, const mathlib::Vec3& worldForce);
		bool setLinkExtForce(const std::string& linkName, const mathlib::Vec3& worldForce);
		void clearExtForces();

		/*
		 * Joint State Accessors
		 */
        void computeRigidBodyKinematics(std::vector<mathlib::Mat4>& world);
		// Free Joint Velocity Accessors
		bool tryGetFreeVelocity(const std::string& linkName, mathlib::VecX& outVel) const;
		bool trySetFreeVelocity(const std::string& linkName, const mathlib::VecX& vel);
		// Joint Angle Accessors
		bool tryGetJointAngleRad(const std::string& childLink, double& outAngle) const;
		bool trySetJointAngleRad(const std::string& childLink, double angleRad);
		// Joint Target Accessors
		bool tryGetJointTargetRad(const std::string& childLink, double& outTargetRad) const;
		bool trySetJointTargetRad(const std::string& childLink, double targetRad);
		// Joint Velocity Accessors (Omega needs renaming)
        bool tryGetJointOmegaRad(const std::string& childLink, double& outOmega) const;
        bool trySetJointOmegaRad(const std::string& childLink, double omegaRad);
		bool injectJointOmegaRad(const std::string& childLink, double omega);
		// Joint Acceleration Accessors (Omega needs renaming)
		bool tryGetJointOmegaMaxRad(const std::string& childLink, double& maxOmegaRad) const;
		bool trySetJointOmegaMaxRad(const std::string& childLink, double maxOmegaRad);
		// Joint Velocity and Acceleration Reference Accessors
		bool trySetJointOmegaRefRad(const std::string& childLink, double omegaRefRad);
		bool trySetJointAlphaRefRad(const std::string& childLink, double alphaRefRad);
		bool trySetJointOmegaRefMaxRad(const std::string& childLink, double omegaRefMaxRad);
		// Joint Reference Derivative Accessors
		bool tryZeroJointRefDerivatives();
		// Joint Target Addition Accessors
		bool tryAddJointTargetRad(const std::string& childLink, double deltaRad);
		// Current Joint State Accessors
		bool isJointAtTargetRad(const std::string& childLink, double tolRad) const;
		bool isJointAtTargetDeg(const std::string& childLink, double tolDeg) const;
		bool isJointNearAngleRad(const std::string& childLink, double targetRad, double tolRad) const;
        bool isJointNearAngleDeg(const std::string& childLink, double targetDeg, double tolDeg) const;

		/*
		 * FreeBody Accesors
		 */
		bool freeBodyState(mathlib::Vec3& pos, mathlib::Quat& orient, mathlib::Vec3& linVel, mathlib::Vec3& angVel) const;
		bool setFreeBodyVelocity(const mathlib::Vec3& linVel, const mathlib::Vec3& angVel);
		bool setFreeBodyPosition(const mathlib::Vec3& pos);
		bool freeBodyMassInertia(double& mass, mathlib::Mat3& I_body) const;
		bool freeBodyLocalAABB(mathlib::Vec3& aabbMin, mathlib::Vec3& aabbMax) const;

		/*
		 * RigidBodySystem Simulation Methods
		 */
		template<typename T>
		RigidBodySnapshot_T<T> takeSnapshot(T simTime) const;
		void step(double dt, double simTime);
		template<size_t NVar>
		void step_AD(double dt, double simTime);
		void updateTrajectoryInputs(control::TrajectoryManager& traj, double t);

		/*
		 * RigidBodySystem Management Methods
		 */
        void loadRigidBody(const std::string& name);
        void resetRigidBody();
		void clearRigidBody();
		void clearRigidBodyLinks();
		void clearRigidBodyJoints();
		void clearRigidBodyExtForces();
		void clearRigidBodySnapshots();
		void clearRigidBodyMetrics();
		void clearRigidBodyLogBuffers();
		void clearRigidBodyTrajectoryInputs();
		void resetRigidBodyState();
		void resetRigidBodyKinematics();
		void resetRigidBodyDynamics();
		void resetRigidBodyMetrics();
		void resetRigidBodyLogBuffers();
		void resetRigidBodyTrajectoryInputs();
        void stopAll();

		/*
		 * RigidBodySystem Pose Methods
		 */
        bool setRigidBodyLinkRotation(const std::string& childLinkName, double angleDeg);
		mathlib::Mat4 setRigidBodyRoot(const mathlib::Vec3& pos, const mathlib::Quat& rot);
        void setRigidBodyRootPose(const mathlib::Vec3& pos, const mathlib::Quat& rot);
		void setRigidBodyRootHome(const mathlib::Vec3& pos, const mathlib::Quat& rot);
		bool setDefaultPoseDeg();
		void setCurrentJointIndex(int index) { _currentJointIndex = index; }

		/*
		 * RigidBodySystem Integration Methods
		 */
		// Integration method accessors
        integration::eIntegrationMethod getIntegrationMethod() const { return _curIntMethod; }
		std::string getIntegratorName() const { return _integrator->IntegratorName(_curIntMethod); }
		void setStandardIntegrator(integration::eIntegrationMethod m);
		// AutoDiff integration method accessors
		integration::eAutoDiffIntegrationMethod AD_IntegrationMethod() const { return _curIntMethod_AD; }
		std::string AD_integratorName() const { return _AD_integrator->IntegratorName(_curIntMethod_AD); }
		void setADIntegrator(integration::eAutoDiffIntegrationMethod m);
		// Integration service accessors
		integration::IntegrationService* getIntegrator();
		const integration::IntegrationService* getIntegrator() const;
		// AutoDiff integration service accessors
		integration::DifferentiableIntegrator* getADIntegrator();
		const integration::DifferentiableIntegrator* getADIntegrator() const;
		// AutoDiff integration enable/disable
		bool autoDiffEnabled() const { return _useAutoDiff; }
		void enableAutoDiff(bool enable) { _useAutoDiff = enable; }
		// Runtime integrator state accessors
		std::shared_ptr<integration::IntegratorState> runtimeIntegratorState();
		std::shared_ptr<const integration::IntegratorState> runtimeIntegratorState() const;

		/*
		 * RigidBodySystem Double Buffer Log Accessors
		 */
		// Log Buffer Accessors
		void setLogBuffer(systems::JointLogBuffer* buf) { _logBuffer = buf; }
		void setFreeBodyLogBuffer(systems::FreeBodyLogBuffer* buf) { _freeBodyLogBuffer = buf; }
		// Swap for the current log buffer, returning a ptr to new active buffer
		std::unique_ptr<systems::JointLogBuffer> claimExportLogBuffer();
		std::unique_ptr<systems::FreeBodyLogBuffer> claimExportLogBuffer_fb();
		// Method to enable or disable the use of internal log buffers
		void useInternalLogBuffer(bool enable);
		void useInternalLogBuffer_fb(bool enable); // Method to enable or disable the use of internal free body log buffers
		// Reserve space in the internal log buffers for a certain number of samples (expected)
		void reserveInternalLogBuffers(size_t expected);	
		void reserveInternalLogBuffers_fb(size_t expected); // Reserve space in the internal free body log buffers for a certain number of samples (expected)

	private:
		/*
		 * Internal RigidBodySystem Methods
		 */
		// Internal method to build the link index mapping from link names to their indices
        void buildLinkIndex();
		void buildSpatialModel();
		// Step implementation for the rigidbody system, templated on the scalar type and integrator type
		template<typename Scalar, typename IntegratorT>
		RigidBodyStepResult_T<Scalar> step_impl(
			const mathlib::VecX_T<Scalar>& x,
			Scalar dt, Scalar t, IntegratorT& integrator,
			physics::DynamicsScratch<Scalar>& dynamicScratch, physics::DynamicsResult<Scalar>& dynamicResult
		);
		// Post-step update method to handle logging and state updates after a simulation step
		template<typename T>
		void postStepUpdate(const mathlib::VecX& x, const physics::DynamicsScratch<T>& scratch, const RigidBodyStepResult_T<T>& result);
		// Log the joint metrics for each joint in the rigidbody system
		template<typename T>
		void logJointMetrics(
			const size_t n,
			const mathlib::VecX& x, const RigidBodyStepResult_T<T>& result,
			const mathlib::VecX& q, const mathlib::VecX& qd, 
			const mathlib::VecX& q_ref, const mathlib::VecX& qd_ref,
			const mathlib::VecX& tau_rnea,
			double sys_KE, double sys_PE, double sys_E
		);
		// Log the free body metrics for a given free body
		template<typename T>
		void logFreeBodyMetrics(
			const std::vector<Pose>& T_world,
			const RigidBodyStepResult_T<T>& result,
			double sys_PE
		);
		// Compute the external forces acting on the rigidbody system based on the current state and rigidbody configuration
		template<typename Scalar>
		void assembleExtForces(physics::DynamicsScratch<Scalar>& scratch) const;
		// Compute the forward drive (velocity) of the rigidbody's root link based on the current state and rigidbody configuration
		double computeForwardDrive() const; // Compute the forward drive (velocity) of the rigidbody's root link based on the current state and rigidbody configuration
		void integrateBaseTranslation(double dt); // Integrate the floating base translation based on the current state and rigidbody configuration
		void updateBaseRootPose(); // Integrate the floating base rotation (yaw-only for now) based on the current state and rigidbody configuration
		// State packing and unpacking
        mathlib::VecX packState() const;
		void unpackState(const mathlib::VecX& x);
		template<typename T>
		void unpackState(const mathlib::VecX_T<T>&& x);
		// State packing and unpacking using a DualNumber vector.
		mathlib::VecX_T<DualNumber_T<double, 14>> packState_AD() const;
		void unpackState_AD(const mathlib::VecX_T<DualNumber_T<double, 14>>& x);
		// Reference state packing and unpacking
		mathlib::VecX packRefState() const;
		void unpackRefState(const mathlib::VecX& xr);
		// Enforce joint limits after integration
		void enforceJointLimits(RigidBodyJoint& j);
		// Compute the minimum Y-coordinate of a link's world position given its index and the current world transforms
		double linkWorldMinY(size_t linkIdx, const Mat4& T) const;

		/*
		 * Internal RigidBodySystem Members
		 */
		// RigidBodySystem Dynamics and Kinematics
		std::unique_ptr<physics::RigidBodyKinematics> _kinematics;
		std::unique_ptr<physics::RigidBodyDynamics> _dynamics;
		// RigidBodySystem Integration
        std::unique_ptr<integration::IntegrationService> _integrator;
        integration::eIntegrationMethod _curIntMethod{};
		std::unique_ptr<integration::DifferentiableIntegrator> _AD_integrator;
		integration::eAutoDiffIntegrationMethod _curIntMethod_AD{};
		// RigidBodySystem State
		RigidBodyModel _body;
		RigidBodyConstModel _constModel;
		SpatialModel<double> _spatialModel;
		// Scratch and result buffers
		physics::DynamicsScratch<double> _dynScratch;
		physics::DynamicsResult<double> _dynResult;
		physics::DynamicsScratch<DualNumber_T<double, 14>> _dynScratch_AD;
		physics::DynamicsResult<DualNumber_T<double, 14>> _dynResult_AD;

		/*
		 * Internal RigidBodySystem Variables
		 */
		// Current role for the rigidbody system
		eRole _role = eRole::Simulation;
		// Simulation time
		double _simTime = 0.0;
		// Physical properties
		std::vector<double> _restitution;
		std::vector<double> _friction;
		mathlib::Vec3 _gravity{ 0.0, 0.0, 0.0 }; // Gravity acceleration (m/s^2)
		double _wn = 0.0;   // configurable natural frequency for PD control (rad/s)
		double _zeta = 0.0; // configurable damping ratio for PD control (unitless)
		bool _useAutoDiff = false;
		// World to rigidbody base transform (meters)
		std::vector<Mat4> _worldTransforms;
		// Index maps for quick lookup of links and joints by name
		std::unordered_map<std::string, int> _link_idx;
		std::unordered_map<std::string, int> _joint_idx;
		std::vector<size_t> _dofJointIndices; // List of joint indices that correspond to the rigidbody's degrees of freedom (excluding fixed joints)
        std::string _loadedName;
		int _currentJointIndex = -1;
		// precomputed clamp lookup tables
		mutable std::vector<uint8_t> _clampTheta;
		mutable std::vector<uint8_t> _clampOmega;
		// Reference state
		mathlib::VecX _xRef;
		bool _refInit = false;
		bool _isReference = false;
		// Flags and precomputed data
        bool _hasBody = false;
		bool _home_valid = false;			   // is home position valid
		mathlib::Mat4 _root_pose = Mat4::Identity();
		mathlib::Mat4 _root_home = Mat4::Identity();
		mathlib::VecX _q_home = mathlib::VecX(); // home/reset joint angles (radians)
		// External forces acting on the rigidbody system
		std::vector<std::tuple<int, int, mathlib::Vec3, mathlib::Vec3>> _pendingExtForces;
		std::vector<double> _prevLinkY;   // last-step link heights for floor damping
		// FLoating base state
		bool _baseIsFree = false;
		// Linear
		mathlib::Vec3 _basePos{ 0,0,0 };
		mathlib::Vec3 _baseVel{ 0,0,0 };
		mathlib::Vec3 _baseAcc{ 0,0,0 };
		// Angular (yaw-only for now, extend later)
		double _baseYaw = 0.0;
		double _baseYawRate = 0.0;
		double _baseYawAcc = 0.0;
		// Tunables
		double _baseMass = 62.0;			// kg (H1 ~60–65)
		double _baseLinearDamping = 6.0;	// Ns/m
		double _baseYawDamping = 2.0;		// Nms/rad
		double _lastBaseForwardForce = 0.0;

		/*
		 * Internal Double Buffer Logging Members
		 */
		// Joint logging buffers
		std::array<systems::JointLogBuffer, 2> _logBuffers{};
		std::atomic<int> _activeLogBufIdx{ 0 }; // index of the currently active log buffer for writing (0 or 1)
		std::mutex _logSwapMutex; // mutex to protect swapping log buffers between simulation and logging thread
		bool _useInternalLogging = true; // flag to determine whether to use internal log buffers or external one provided by setLogBuffer
		systems::JointLogBuffer* _logBuffer = nullptr;
		// Free body logging buffers
		std::array<systems::FreeBodyLogBuffer, 2> _logBuffers_fb{};
		std::atomic<int> _activeLogBufIdx_fb{ 0 }; // index of the currently active log buffer for writing (0 or 1)
		std::mutex _logSwapMutex_fb; // mutex to protect swapping free body log buffers between simulation and logging thread
		bool _useInternalLogging_fb = true; // flag to determine whether to use internal free body log buffers or external one provided by setFreeBodyLogBuffer
		systems::FreeBodyLogBuffer* _freeBodyLogBuffer = nullptr;

	};
} // namespace rigidbody
#include "Systems/RigidBodySystemStep.inl"