#pragma once

#include "EngineCore.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "Robots/RobotModel.h"

namespace robots {
	class ENGINE_API RobotSystem {
	public:
		using spawnFn = std::function<std::vector<scene::Object*>(const std::string&)>; // function type for loading meshes

		RobotSystem(std::vector<std::unique_ptr<scene::Object>>& sceneObjects, spawnFn meshLoader);

        // Robotic Arm System
        void loadRobot(const std::string& name);
        void updateRobotKinematics();

        bool hasRobot() const { return _hasRobot; }

		bool tryGetJointAngleRad(const std::string& childLink, float& outAngle) const;
		bool trySetJointAngleRad(const std::string& childLink, float angleRad);

        const std::vector<RobotLink>& links() const { return _robot.links; }
        const std::vector<RobotJoint>& joints() const { return _robot.joints; }
        std::size_t linkCount() const { return _robot.links.size(); }

        static float clampJointAngle(const RobotJoint& joint, float angleRad);
        static float wrapToPi(float angleRad);
        static float wrapRad(float angleRad);

        bool hasLinkName(const std::string& linkName) const { return _linkIndex.find(linkName) != _linkIndex.end(); }
		const std::string& robotName() const { return _robot.name; } // placeholder for now

        void setRobotLinkRotation(const std::string& linkName, float angle);
        void setRobotRootPose(const glm::vec3& pos, const glm::quat& rot);
        void setRobotRootHome(const glm::vec3& pos, const glm::quat& rot);
        void resetRobot();
        void clearRobot();

		RobotModel& getRobotModel() { return _robot; } // needed for now, may remove later

	private:
        void instantiateRobotLinks();
        void buildLinkIndex();

		std::vector<std::unique_ptr<scene::Object>>& _objects;
		spawnFn _loadMeshReturn;

        RobotModel _robot;
        bool _hasRobot = false;

        std::string _loadedName;

        std::unordered_map<std::string, int> _linkIndex;

        glm::mat4 _robotRootPose = glm::mat4(1.0f); // current pose (meters)
		glm::mat4 _robotRootHome = glm::mat4(1.0f); // home/reset pose (meters)

		VecX _robotQHome; // home/reset joint angles (radians)
        bool _robotHomeValid = false;
	};
} // namespace robot