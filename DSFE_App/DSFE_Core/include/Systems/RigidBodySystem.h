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
	// Forward declarations
	enum class eTorqueMode;

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

	inline constexpr size_t AD_VARS = 14; // number of independent variables for autodiff (used for pre-allocating AD integrator buffers)

	inline mathlib::Quat expToQuat(const mathlib::Vec3& rv) {
		const double theta = rv.norm();
		if (theta < 1e-9) { return mathlib::Quat(1, 0, 0, 0); }
		return mathlib::Quat(Eigen::AngleAxisd(theta, rv / theta));
	}

	class DSFE_API RigidBodySystem {
	public:
		RigidBodySystem();
		~RigidBodySystem();

        // --- Utility Methods ---

        static double clampJointAngle(const RigidBodyJoint& joint, double angleRad);
		template<typename T>
		static T clampJointAngle_T(const RigidBodyJoint& joint, T angleRad);

        // ---- Accessors ---

		const systems::RigidBodyModel& model() const;
		const std::vector<Mat4>& worldTransforms() const { return _worldTransforms; }

        const std::vector<RigidBodyLink>& links() const { return _body.links; }
		std::vector<RigidBodyLink>& links() { return _body.links; }
        const std::vector<RigidBodyJoint>& joints() const { return _body.joints; }
		std::vector<RigidBodyJoint>& joints() { return _body.joints; }

		std::vector<std::string> linkNames() const;

        std::size_t linkCount() const { return _body.links.size(); }
		std::size_t jointCount() const { return _body.joints.size(); }
		std::string findRootLink() const;
        bool hasLinkName(const std::string& linkName) const { return _link_idx.find(linkName) != _link_idx.end(); }
		bool hasFreeJoint() const;

		int jointStateOffset(size_t joint_idx) const;
		int totalDOF() const;

        const std::string& rigidBodyName() const { return _body.name; }
        bool hasRigidBody() const { return _hasBody; }

		void setGravity(double g);
		void setGravityVec(const mathlib::Vec3& g);
		const mathlib::Vec3& getGravityVec() const { return _gravity; }
		double getGravity() const { return _gravity.norm(); }

		void setNaturalFrequency(double wn) { _wn = wn; }
		double getNaturalFrequency() const { return _wn; }
		void resetNaturalFrequencyToTarget() { for (auto& joint : _body.joints) { joint.wn_target = _wn; } }

		void setDampingRatio(double zeta) { _zeta = zeta; }
		double getDampingRatio() const { return _zeta; }
		void resetDampingRatioToTarget() {
			for (auto& joint : _body.joints) { joint.zeta_target = _zeta; }
		}

		// Get pointer to this Systemsystem
		const RigidBodySystem& getRigidBody() const { return *this; }

		// --- External Force Application Methods ---

		bool linkWorldOrigin(const std::string& linkName, mathlib::Vec3& out) const;
		bool setLinkExtForce(const std::string& linkName, const mathlib::Vec3& worldPoint, const mathlib::Vec3& worldForce);
		bool setLinkExtForce(const std::string& linkName, const mathlib::Vec3& worldForce);
		void clearExtForces();

		// ---- Joint State Methods ---

        void computeRigidBodyKinematics(std::vector<mathlib::Mat4>& world);

		bool tryGetJointAngleRad(const std::string& childLink, double& outAngle) const;
		bool trySetJointAngleRad(const std::string& childLink, double angleRad);

		bool tryGetFreeVelocity(const std::string& linkName, mathlib::VecX& outVel) const;
		bool trySetFreeVelocity(const std::string& linkName, const mathlib::VecX& vel);

        bool tryGetJointOmegaRad(const std::string& childLink, double& outOmega) const;
        bool trySetJointOmegaRad(const std::string& childLink, double omegaRad);
		bool injectJointOmegaRad(const std::string& childLink, double omega);

		bool tryGetJointTargetRad(const std::string& childLink, double& outTargetRad) const;
		bool trySetJointTargetRad(const std::string& childLink, double targetRad);

		bool tryGetJointOmegaMaxRad(const std::string& childLink, double& maxOmegaRad) const;
		bool trySetJointOmegaMaxRad(const std::string& childLink, double maxOmegaRad);

		bool tryAddJointTargetRad(const std::string& childLink, double deltaRad);

		bool isJointAtTargetRad(const std::string& childLink, double tolRad) const;
		bool isJointAtTargetDeg(const std::string& childLink, double tolDeg) const;

		bool isJointNearAngleRad(const std::string& childLink, double targetRad, double tolRad) const;
        bool isJointNearAngleDeg(const std::string& childLink, double targetDeg, double tolDeg) const;

		bool trySetJointOmegaRefRad(const std::string& childLink, double omegaRefRad);
		bool trySetJointAlphaRefRad(const std::string& childLink, double alphaRefRad);

		bool trySetJointOmegaRefMaxRad(const std::string& childLink, double omegaRefMaxRad);

		bool tryZeroJointRefDerivatives();

		// --- SIMULATION STEP METHOD ---

		template<typename T>
		RigidBodySnapshot_T<T> takeSnapshot(T simTime) const;
		template<size_t NVar>
		void step_AD(double dt, double simTime);

		void step(double dt, double simTime);
		void updateTrajectoryInputs(control::TrajectoryManager& traj, double t);

		// --- RIGIDBODY LOADING AND RESET METHODS ---

        void loadRigidBody(const std::string& name);
        void resetRigidBody();
        void stopAll();

        // --- RIGIDBODY LINK AND ROOT POSE METHODS ---

        bool setRigidBodyLinkRotation(const std::string& childLinkName, double angleDeg);
		mathlib::Mat4 setRigidBodyRoot(const mathlib::Vec3& pos, const mathlib::Quat& rot);
        void setRigidBodyRootPose(const mathlib::Vec3& pos, const mathlib::Quat& rot);
		void setRigidBodyRootHome(const mathlib::Vec3& pos, const mathlib::Quat& rot);

		bool setDefaultPoseDeg();
		void setCurrentJointIndex(int index) { _currentJointIndex = index; }

		// --- GET AND SET INTEGRATION METHOD ---

        integration::eIntegrationMethod getIntegrationMethod() const { return _curIntMethod; }
		std::string getIntegratorName() const { return _integrator->IntegratorName(_curIntMethod); }
		void setStandardIntegrator(integration::eIntegrationMethod m);

		integration::eAutoDiffIntegrationMethod AD_IntegrationMethod() const { return _curIntMethod_AD; }
		std::string AD_integratorName() const { return _AD_integrator->IntegratorName(_curIntMethod_AD); }
		void setADIntegrator(integration::eAutoDiffIntegrationMethod m);
		
		integration::IntegrationService* getIntegrator();
		const integration::IntegrationService* getIntegrator() const;

		integration::DifferentiableIntegrator* getADIntegrator();
		const integration::DifferentiableIntegrator* getADIntegrator() const;
		
		bool autoDiffEnabled() const { return _useAutoDiff; }
		void enableAutoDiff(bool enable) { _useAutoDiff = enable; }

		std::shared_ptr<integration::IntegratorState> runtimeIntegratorState();
		std::shared_ptr<const integration::IntegratorState> runtimeIntegratorState() const;

		void setRefBuffer(systems::TrajRefBuffer* buf)  { _refBuffer = buf; }
		void setLogBuffer(systems::JointLogBuffer* buf) { _logBuffer = buf; }
		void setRole(eRole role) { _role = role; }

		// Setter and getter the torque mode for the rigidbody system
		void setTorqueMode(eTorqueMode mode);
		eTorqueMode getTorqueMode() const { return _body.torqueMode; }

		// Swap for the current log buffer, returning a ptr to new active buffer
		std::unique_ptr<systems::JointLogBuffer> claimExportLogBuffer();

		// Method to enable or disable the use of internal log buffers
		void useInternalLogBuffer(bool enable);

		// Reserve space in the internal log buffers for a certain number of samples (expected)
		void reserveInternalLogBuffers(size_t expected);

	private:
        void buildLinkIndex();
		void buildSpatialModel();

		template<typename Scalar, typename IntegratorT>
		RigidBodyStepResult_T<Scalar> step_impl(
			const mathlib::VecX_T<Scalar>& x,
			Scalar dt, Scalar t, IntegratorT& integrator,
			physics::DynamicsScratch<Scalar>& dynamicScratch, physics::DynamicsResult<Scalar>& dynamicResult
		);

		template<typename T>
		void postStepUpdate(const mathlib::VecX& x, const physics::DynamicsScratch<T>& scratch, const RigidBodyStepResult_T<T>& result);

		template<typename Scalar>
		void assembleExtForces(physics::DynamicsScratch<Scalar>& scratch) const;

		std::unique_ptr<physics::RigidBodyKinematics> _kinematics;
		std::unique_ptr<physics::RigidBodyDynamics> _dynamics;

        std::unique_ptr<integration::IntegrationService> _integrator;
        integration::eIntegrationMethod _curIntMethod{};

		std::unique_ptr<integration::DifferentiableIntegrator> _AD_integrator;
		integration::eAutoDiffIntegrationMethod _curIntMethod_AD{};

		eRole _role = eRole::Simulation;

		double _wn = 0.0;   // configurable natural frequency for PD control (rad/s)
		double _zeta = 0.0; // configurable damping ratio for PD control (unitless)

		bool _useAutoDiff = false;

		// Compute the forward drive (velocity) of the rigidbody's root link based on the current state and rigidbody configuration
		double computeForwardDrive() const;
		// Integrate the floating base translation based on the current state and rigidbody configuration
		void integrateBaseTranslation(double dt);
		// Integrate the floating base rotation (yaw-only for now) based on the current state and rigidbody configuration
		void updateBaseRootPose();

		// State packing and unpacking
        mathlib::VecX packState() const;
		void unpackState(const mathlib::VecX& x);

		template<typename T>
		void unpackState(const mathlib::VecX_T<T>& x);

		// State packing and unpacking using a DualNumber vector.
		mathlib::VecX_T<DualNumber_T<double, 14>> packState_AD() const;
		void unpackState_AD(const mathlib::VecX_T<DualNumber_T<double, 14>>& x);

		// Reference state packing and unpacking
		mathlib::VecX packRefState() const;
		void unpackRefState(const mathlib::VecX& xr);

		// Enforce joint limits after integration
		void enforceJointLimits(RigidBodyJoint& j);

		// Simulation time
		double _simTime = 0.0;

		// RigidBody model, and rigidbody mode
        RigidBodyModel _body;
		eTorqueMode _torqueMode = _body.torqueMode;

		SpatialModel<double> _spatialModel;
		RigidBodyConstModel _constModel;
		
		std::vector<std::tuple<int, int, mathlib::Vec3, mathlib::Vec3>> _pendingExtForces;
		std::vector<double> _prevLinkY;   // last-step link heights for floor damping

		double linkWorldMinY(size_t linkIdx, const Mat4& T) const;

		physics::DynamicsScratch<double> _dynScratch;
		physics::DynamicsResult<double> _dynResult;

		physics::DynamicsScratch<DualNumber_T<double, 14>> _dynScratch_AD;
		physics::DynamicsResult<DualNumber_T<double, 14>> _dynResult_AD;

		// World to rigidbody base transform (meters)
		std::vector<Mat4> _worldTransforms;

		// Flags and precomputed data
        bool _hasBody = false;
		mathlib::Mat4 _root_pose = Mat4::Identity();
		mathlib::Mat4 _root_home = Mat4::Identity();
		mathlib::VecX _q_home = mathlib::VecX(); // home/reset joint angles (radians)
		bool _home_valid = false;			   // is home position valid

		// Index maps for quick lookup of links and joints by name
		std::unordered_map<std::string, int> _link_idx;
		std::unordered_map<std::string, int> _joint_idx;
		// List of joint indices that correspond to the rigidbody's degrees of freedom (excluding fixed joints)
		std::vector<size_t> _dofJointIndices; 

        std::string _loadedName;
		int _currentJointIndex = -1;

		// Reference state
		mathlib::VecX _xRef;
		bool _refInit = false;
		bool _isReference = false;

		// precomputed clamp lookup tables
		mutable std::vector<uint8_t> _clampTheta;
		mutable std::vector<uint8_t> _clampOmega;

		// Gravity acceleration (m/s^2)
		mathlib::Vec3 _gravity{ 0.0, 0.0, 0.0 };

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

		// Double-buffer design
		std::array<systems::JointLogBuffer, 2> _logBuffers{};
		std::atomic<int> _activeLogBufIdx{ 0 }; // index of the currently active log buffer for writing (0 or 1)
		std::mutex _logSwapMutex; // mutex to protect swapping log buffers between simulation and logging thread
		bool _useInternalLogging = true; // flag to determine whether to use internal log buffers or external one provided by setLogBuffer

		// Pointers to external log and reference buffers (not owned by Systemsystem)
		systems::JointLogBuffer* _logBuffer = nullptr;
		systems::TrajRefBuffer* _refBuffer = nullptr;

	};
} // namespace rigidbody
#include "Systems/RigidBodySystemStep.inl"