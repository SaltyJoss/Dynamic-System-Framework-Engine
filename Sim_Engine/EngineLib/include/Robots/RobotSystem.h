#pragma once

#include "EngineCore.h"
#include "Robots/RobotModel.h"
#include "Numerics/IntegrationService.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace robots {
	// Joint state structure
    struct ENGINE_API JointState { double theta; double omega; };

	class ENGINE_API RobotSystem {
	public:
		using spawnFn = std::function<std::vector<scene::Object*>(const std::string&)>; // function type for loading meshes

		RobotSystem(std::vector<std::unique_ptr<scene::Object>>& sceneObjects, spawnFn meshLoader);

        // --- Utility Methods ---

        static float clampJointAngle(const RobotJoint& joint, float angleRad);
        static float wrapToPi(float angleRad);
        static float wrapRad(float angleRad);

        // ---- Accessors ---

        const std::vector<RobotLink>& links() const { return _robot.links; }
		std::vector<RobotLink>& links() { return _robot.links; }
        const std::vector<RobotJoint>& joints() const { return _robot.joints; }
		std::vector<RobotJoint>& joints() { return _robot.joints; }

        std::size_t linkCount() const { return _robot.links.size(); }

        bool hasLinkName(const std::string& linkName) const { return _linkIndex.find(linkName) != _linkIndex.end(); }
        const std::string& robotName() const { return _robot.name; }
        bool hasRobot() const { return _hasRobot; }

		double getGravity() const { return _gravity; }
        void setGravity(double g) { _gravity = g; }

		// ---- Joint State Methods ---

        void updateRobotKinematics();

		bool tryGetJointAngleRad(const std::string& childLink, float& outAngle) const;
		bool trySetJointAngleRad(const std::string& childLink, float angleRad);

        bool tryGetJointOmegaRad(const std::string& childLink, float& outOmega) const;
        bool trySetJointOmegaRad(const std::string& childLink, float omegaRad);

		bool tryGetJointTargetRad(const std::string& childLink, float& outTargetRad) const;
		bool trySetJointTargetRad(const std::string& childLink, float targetRad);

		bool trySetJointOmegaMaxRad(const std::string& childLink, float maxOmegaRad);
		bool tryAddJointTargetRad(const std::string& childLink, float deltaRad);

		bool isJointAtTargetRad(const std::string& childLink, float tolRad) const;
		bool isJointAtTargetDeg(const std::string& childLink, float tolDeg) const;

		bool isJointNearAngleRad(const std::string& childLink, float targetRad, float tolRad) const;
        bool isJointNearAngleDeg(const std::string& childLink, float targetDeg, float tolDeg) const;

		bool trySetJointOmegaRefRad(const std::string& childLink, float omegaRefRad);
		bool trySetJointAlphaRefRad(const std::string& childLink, float alphaRefRad);

		bool tryZeroJointRefDerivatives();

		// --- SIMULATION STEP METHOD ---

		void step(double dt, double simTime);

		// --- ROBOT LOADING AND RESET METHODS ---

        void loadRobot(const std::string& name);
        void resetRobot();
        void clearRobot();
        void stopAll();

        // --- ROBOT LINK AND ROOT POSE METHODS ---

        bool setRobotLinkRotation(const std::string& childLinkName, float angleDeg);
        void setRobotRootPose(const glm::vec3& pos, const glm::quat& rot);
        void setRobotRootHome(const glm::vec3& pos, const glm::quat& rot);

		void setCurrentJointIndex(int index) { _currentJointIndex = index; }

		// --- GET AND SET INTEGRATION METHOD ---

        integration::eIntegrationMethod getIntegrationMethod() const { return _curIntMethod; }
		void setIntegrationMethod(integration::eIntegrationMethod method) { _curIntMethod = method; }

	private:
        void instantiateRobotLinks();
        void buildLinkIndex();

        std::unique_ptr < integration::IntegrationService> _integrator;
        std::unique_ptr<integration::ReferenceSolver> _refSolver;
        integration::eIntegrationMethod _curIntMethod{};

		std::vector<std::unique_ptr<scene::Object>>& _objects;

        mathlib::VecX packState() const;
		void unpackState(const mathlib::VecX& x);
		mathlib::VecX deriv(double t, const mathlib::VecX& x) const;
		void enforceJointLimits(RobotJoint& j);

		double _simTime = 0.0;
		spawnFn _loadMeshReturn;

        RobotModel _robot;
        bool _hasRobot = false;

        std::string _loadedName;
		int _currentJointIndex = -1;

        std::unordered_map<std::string, int> _linkIndex;

        glm::mat4 _robotRootPose = glm::mat4(1.0f); // current pose (meters)
		glm::mat4 _robotRootHome = glm::mat4(1.0f); // home/reset pose (meters)

        std::vector<glm::mat4> _bindWorld0;  // size = links.size()
		std::vector<glm::mat4> _bindLocal0;  // size = links.size()

		double _gravity = 9.81; // m/s^2

		VecX _robotQHome; // home/reset joint angles (radians)
        bool _robotHomeValid = false;
	};
} // namespace robot