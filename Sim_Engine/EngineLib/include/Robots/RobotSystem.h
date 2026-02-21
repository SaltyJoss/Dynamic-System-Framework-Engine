#pragma once
// File:   RobotSystem.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "Robots/RobotModel.h"
#include "Analysis/MetricLogger.h"
#include "Numerics/IntegrationService.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Forward declarations
namespace control { class ENGINE_API TrajectoryManager; }

namespace robots {
	// Forward declarations
	class ENGINE_API RobotKinematics;
	class ENGINE_API RobotDynamics;
	enum class eTorqueMode;

	// Joint state structure
    struct ENGINE_API JointState {
		double theta;
		double omega;
	};

	// Metrics structure
	enum class eRole {
		Simulation,
		Baseline
	};

	class ENGINE_API RobotSystem {
	public:
		using spawnFn = std::function<std::vector<scene::Object*>(const std::string&)>; // function type for loading meshes

		RobotSystem(std::vector<std::unique_ptr<scene::Object>>& sceneObjects, spawnFn meshLoader);
		~RobotSystem();

        // --- Utility Methods ---

        static double clampJointAngle(const RobotJoint& joint, double angleRad);

        // ---- Accessors ---

        const std::vector<RobotLink>& links() const { return _robot.links; }
		std::vector<RobotLink>& links() { return _robot.links; }
        const std::vector<RobotJoint>& joints() const { return _robot.joints; }
		std::vector<RobotJoint>& joints() { return _robot.joints; }

        std::size_t linkCount() const { return _robot.links.size(); }
		std::size_t jointCount() const { return _robot.joints.size(); }

		std::string findRootLink() const;

        bool hasLinkName(const std::string& linkName) const { return _linkIndex.find(linkName) != _linkIndex.end(); }
        const std::string& robotName() const { return _robot.name; }
        bool hasRobot() const { return _hasRobot; }

		void setGravity(double g);
		double getGravity() const { return _gravity; }

		// Get pointer to this RobotSystem
		const RobotSystem& getRobot() const { return *this; }

		// ---- Joint State Methods ---

        void updateRobotKinematics();

		bool tryGetJointAngleRad(const std::string& childLink, double& outAngle) const;
		bool trySetJointAngleRad(const std::string& childLink, double angleRad);

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

		void step(double dt, double simTime);
		void updateTrajectoryInputs(control::TrajectoryManager& traj, double t);

		// --- ROBOT LOADING AND RESET METHODS ---

        void loadRobot(const std::string& name);
        void resetRobot();
        void clearRobot();
        void stopAll();

        // --- ROBOT LINK AND ROOT POSE METHODS ---

        bool setRobotLinkRotation(const std::string& childLinkName, double angleDeg);
        void setRobotRootPose(const glm::vec3& pos, const glm::quat& rot);
		void setRobotRootHome(const glm::vec3& pos, const glm::quat& rot);

		bool setDefaultPoseDeg();

		void setCurrentJointIndex(int index) { _currentJointIndex = index; }

		// --- GET AND SET INTEGRATION METHOD ---

        integration::eIntegrationMethod getIntegrationMethod() const { return _curIntMethod; }
		void setIntegrationMethod(integration::eIntegrationMethod method) { _curIntMethod = method; }
		std::string getIntegratorName() const { return _integrator->IntegratorName(_curIntMethod); }

		integration::IntegrationService* getIntegrator();
		const integration::IntegrationService* getIntegrator() const;

		void setRefBuffer(robots::TrajRefBuffer* buf)  { _refBuffer = buf; }
		void setLogBuffer(robots::JointLogBuffer* buf) { _logBuffer = buf; }
		void setRole(eRole role) { _role = role; }

		// Setter and getter the torque mode for the robot system
		void setTorqueMode(eTorqueMode mode);
		eTorqueMode getTorqueMode() const { return _torqueMode; }

	private:
        void instantiateRobotLinks();
        void buildLinkIndex();

		std::unique_ptr<RobotKinematics> _kinematics;
		std::unique_ptr<RobotDynamics> _dynamics;

        std::unique_ptr<integration::IntegrationService> _integrator;
        integration::eIntegrationMethod _curIntMethod{};

		std::vector<std::unique_ptr<scene::Object>>& _objects;

		eRole _role = eRole::Simulation;

		spawnFn _loadMeshReturn;

		// Forward kinematics computation
		std::vector<Pose> computeForwardKinematics_fromState(const VecX& q) const;

		// Compute the full mass matrix M(q) based on the current state and robot configuration
		mathlib::MatX computeMassMatrix(const std::vector<double>& q, const std::vector<Pose>& T_world) const;
		// Compute the gravity torque for a joint based on the current state and robot configuration
		std::vector<double> computeGravityTorque(const std::vector<double>& q, const std::vector<Pose>& T_world) const;

		// Computes the control torque for a joint based on the current state, reference, and robot configuration
		VecX computeAppliedTorques(
			const std::vector<double>& q,
			const std::vector<double>& qd,
			const std::vector<double>& eta,
			const std::vector<Pose>& T_world,
			std::vector<double> I_eff,
			std::vector<double> tau_gravity
		) const;

		// Compute control and dynamics metrics for a specific joint based on the current state and reference
		RobotMetrics computeJointMetrics(
			const RobotJoint& joint, const RobotLink& link, double I_eff,
			double q, double qd, double eta,
			double q_ref, double qd_ref, double qdd_ref,
			double tau_coriolis, double tau_g
		) const;

		// Compute the forward drive (velocity) of the robot's root link based on the current state and robot configuration
		double computeForwardDrive() const;
		// Integrate the floating base translation based on the current state and robot configuration
		void integrateBaseTranslation(double dt);
		// Integrate the floating base rotation (yaw-only for now) based on the current state and robot configuration
		void updateBaseRootPose();

		// State packing and unpacking
        mathlib::VecX packState() const;
		void unpackState(const mathlib::VecX& x);

		// Reference state packing and unpacking
		mathlib::VecX packRefState() const;
		void unpackRefState(const mathlib::VecX& xr);

		// Compute state derivatives
		mathlib::VecX deriv(double t, const mathlib::VecX& x) const;

		// Enforce joint limits after integration
		void enforceJointLimits(RobotJoint& j);

		// Simulation time
		double _simTime = 0.0;

		// Robot model, and robot mode
        RobotModel _robot;
		eTorqueMode _torqueMode;

		// Buffers for logging and reference state (not owned by RobotSystem)
		robots::JointLogBuffer* _logBuffer = nullptr;
		robots::TrajRefBuffer*  _refBuffer = nullptr;

		// Flags and precomputed data
        bool _hasRobot = false;
		glm::mat4 _robotRootPose = glm::mat4(1.0f); // current pose (meters)
		glm::mat4 _robotRootHome = glm::mat4(1.0f); // home/reset pose (meters)
		VecX _robotQHome;							// home/reset joint positions
		bool _robotHomeValid = false;				// is home position valid

		// Index maps for quick lookup of links and joints by name
		std::unordered_map<std::string, int> _linkIndex;
		std::unordered_map<std::string, int> _jointIndex;
		// List of joint indices that correspond to the robot's degrees of freedom (excluding fixed joints)
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
		double _gravity = 0.0;

		// FLoating base state
		bool _baseIsFree = false;

		// Linear
		Vec3 _basePos{ 0,0,0 };
		Vec3 _baseVel{ 0,0,0 };
		Vec3 _baseAcc{ 0,0,0 };

		// Angular (yaw-only for now, extend later)
		double _baseYaw = 0.0;
		double _baseYawRate = 0.0;
		double _baseYawAcc = 0.0;

		// Tunables
		double _baseMass = 62.0;          // kg (H1 ≈ 60–65)
		double _baseLinearDamping = 6.0;   // Ns/m
		double _baseYawDamping = 2.0;      // Nms/rad
		double _lastBaseForwardForce = 0.0;
	};
} // namespace robot

// --- Logging macros for robot syste debugging ---

// LOG_ROT
#ifdef LOG_ROT
#error LOG_ROT macro already defined. Please undefine it before including RobotSystem.h to avoid conflicts.
#endif
// Logs the rotation part of a 4x4 matrix with a custom tag
#define LOG_ROT(tag, M) \
	LOG_INFO("[ROT] %s | X=(%.2f %.2f %.2f) Y=(%.2f %.2f %.2f) Z=(%.2f %.2f %.2f)", \
	tag, \
	M[0][0], M[0][1], M[0][2], \
	M[1][0], M[1][1], M[1][2], \
	M[2][0], M[2][1], M[2][2])

#define LOG_MAT4(tag, M) \
	LOG_INFO("[MAT4] %s:\n" \
		"[ % .3f % .3f % .3f % .3f ]\n" \
		"[ % .3f % .3f % .3f % .3f ]\n" \
		"[ % .3f % .3f % .3f % .3f ]\n" \
		"[ % .3f % .3f % .3f % .3f ]", \
		tag, \
		M[0][0], M[1][0], M[2][0], M[3][0], \
		M[0][1], M[1][1], M[2][1], M[3][1], \
		M[0][2], M[1][2], M[2][2], M[3][2], \
		M[0][3], M[1][3], M[2][3], M[3][3])