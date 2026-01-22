
#include "pch.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <kinematics/Forward_Kinematics.h>
#include "Robots/RobotLoader.h"
#include "Scene/Object.h"
#include "Scene/MeshLoader.h"
#include "EngineLib/LogMacros.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using kinematics::DH_Params;
using kinematics::JointType;

namespace robots {
	// --- Static Helper Functions ---

	// Find a link by name in the robot model
	static RobotLink* findLink(std::vector<RobotLink>& links, const std::string& name) {
		for (auto& link : links) { if (link.name == name) { return &link; } }
		LOG_WARN("Link %s not found in robot model", name.c_str());
		return nullptr;
	}

	// Parse quaternion from joint data
	static glm::quat parseQuaternion(const json& jointData) {
		if (jointData.contains("quat") && jointData["quat"].is_array() && jointData["quat"].size() == 4) {
			const auto& q = jointData["quat"];
			return glm::normalize(glm::quat(
				q[0].get<float>(), // w
				q[1].get<float>(), // x
				q[2].get<float>(), // y
				q[3].get<float>()  // z
			));
		}

		return glm::quat(1.0, 0.0, 0.0, 0.0);
	}

	// tf2::Quaternion::setRPY(roll,pitch,yaw) corresponds to q = qz * qy * qx.
	static glm::quat rpyRadToQuat(const glm::vec3& rpyRad)
	{
		const float roll = rpyRad.x;
		const float pitch = rpyRad.y;
		const float yaw = rpyRad.z;

		const glm::quat qx = glm::angleAxis(roll, glm::vec3(1, 0, 0));
		const glm::quat qy = glm::angleAxis(pitch, glm::vec3(0, 1, 0));
		const glm::quat qz = glm::angleAxis(yaw, glm::vec3(0, 0, 1));

		return glm::normalize(qz * qy * qx);
	}

	// Parse DH joint type from string
	static JointType parseDHType(const std::string& s) {
		std::string t = s;
		for (char& c : t) { c = static_cast<char>(std::tolower((unsigned char)c)); }
		if (t == "revolute" || t == "r") { return JointType::Revolute; }
		return JointType::Prismatic;
	}

	// --- RobotLoader Implementation ---

	// Load a robot model from a JSON file
	robots::RobotModel RobotLoader::loadFromJSON(const std::string& filepath) {
		RobotModel robot;
		LOG_INFO("Loading robot model from JSON: %s", filepath.c_str());

		std::ifstream file(filepath);
		if (!file.is_open()) {
			LOG_ERROR("Failed to open JSON file: %s", filepath.c_str());
			return robot;
		}
		json data = json::parse(file);

		robot.name = data["name"].get<std::string>();
		robot.scale = data["scale"].get<float>();

		for (auto& linkData : data["links"]) {
			RobotLink link;
			link.name = linkData.value("name", "");

			link.visual.meshFile = linkData.value("mesh", ""); // legacy field for mesh

			// visual
			if (linkData.contains("visual")) {
				auto& v = linkData["visual"];
				link.visual.meshFile = v.value("mesh", "");
				if (v.contains("origin_xyz")) { link.visual.origin_xyz = glm::vec3(v["origin_xyz"][0], v["origin_xyz"][1], v["origin_xyz"][2]); }
				if (v.contains("origin_rpy")) { link.visual.origin_rpy = glm::vec3(v["origin_rpy"][0], v["origin_rpy"][1], v["origin_rpy"][2]); }
				if (v.contains("mesh")) { link.visual.meshFile = v.value("mesh", link.visual.meshFile); }
			}

			// collisions
			if (linkData.contains("collision")) {
				for (auto& c : linkData["collision"]) {
					CollisionShape s;
					s.type = c.value("type", "");
					if (c.contains("origin_xyz")) { s.origin_xyz = glm::vec3(c["origin_xyz"][0], c["origin_xyz"][1], c["origin_xyz"][2]); }
					if (c.contains("origin_rpy")) { s.origin_rpy = glm::vec3(c["origin_rpy"][0], c["origin_rpy"][1], c["origin_rpy"][2]); }

					if (s.type == "cylinder") {
						s.size.x = c.value("radius", 0.0f);  // radius
						s.size.y = c.value("length", 0.0f);  // length
					}
					else if (s.type == "box") {
						if (c.contains("size") && c["size"].is_array() && c["size"].size() == 3) {
							s.size = glm::vec3(c["size"][0].get<float>(), c["size"][1].get<float>(), c["size"][2].get<float>());
						}
					}
					else if (s.type == "mesh") { s.meshFile = c.value("mesh", ""); }
					link.collisions.push_back(s);
				}
			}

			// inertial
			if (linkData.contains("inertial")) {
				auto& I = linkData["inertial"];
				link.inertial.mass = I.value("mass", 0.0f);
				if (I.contains("com_xyz")) link.inertial.com_xyz = glm::vec3(I["com_xyz"][0], I["com_xyz"][1], I["com_xyz"][2]);
				if (I.contains("inertia")) {
					auto& J = I["inertia"];
					link.inertial.inertia.ixx = J.value("ixx", 0.0f);
					link.inertial.inertia.ixy = J.value("ixy", 0.0f);
					link.inertial.inertia.ixz = J.value("ixz", 0.0f);
					link.inertial.inertia.iyy = J.value("iyy", 0.0f);
					link.inertial.inertia.iyz = J.value("iyz", 0.0f);
					link.inertial.inertia.izz = J.value("izz", 0.0f);
				}
			}

			// render fix (optional)
			if (linkData.contains("render_fix") && linkData["render_fix"].contains("dh_to_mesh_quat")) {
				auto& q = linkData["render_fix"]["dh_to_mesh_quat"];
				link.dhToMeshFix = glm::normalize(glm::quat(q[0], q[1], q[2], q[3]));
			}

			robot.links.push_back(link);
		}

		// Load joints
		for (auto& jointData : data["joints"]) {
			RobotJoint joint;

			// Load basic joint info
			joint.name = jointData["name"].get<std::string>();
			joint.parent = jointData["parent"].get<std::string>();
			joint.child = jointData["child"].get<std::string>();

			// origin
			if (jointData.contains("origin")) {
				auto& o = jointData["origin"];
				if (o.contains("origin_xyz")) { joint.origin_xyz = glm::vec3(o["origin_xyz"][0], o["origin_xyz"][1], o["origin_xyz"][2]); }
				if (o.contains("origin_rpy")) {
					glm::vec3 rpy = glm::vec3(o["origin_rpy"][0], o["origin_rpy"][1], o["origin_rpy"][2]);
					joint.origin_q = rpyRadToQuat(rpy);
				}
			}

			// axis
			joint.axis = glm::normalize(glm::vec3(
				jointData["axis"][0].get<float>(),
				jointData["axis"][1].get<float>(),
				jointData["axis"][2].get<float>()
			));

			// limits
			json limits = jointData.contains("limits") ? jointData["limits"] : json::object();

			if (jointData.contains("limits")) {
				joint.limits.continuous = limits.value("continuous", false);
				joint.limits.maxOmegaRad_s = limits.value("velocity", joint.limits.maxOmegaRad_s);
				if (!joint.limits.continuous) {
					if (limits.contains("lower") && limits["lower"].is_number()) { joint.limits.minAngle = limits["lower"].get<float>(); }
					if (limits.contains("upper") && limits["upper"].is_number()) { joint.limits.maxAngle = limits["upper"].get<float>(); }
				}
				else {
					joint.limits.minAngle = glm::radians(-359.9f);	// practically continuous
					joint.limits.maxAngle = glm::radians(359.9f);	// practically continuous
				}
				// Need to add effort, but for now imma ignore to test this first!
			}

			// dynamics
			if (jointData.contains("dynamics")) {
				json dynamics = jointData["dynamics"];
				if (dynamics.contains("damping") && dynamics["damping"].is_number()) { joint.dynamics.damping = dynamics["damping"].get<float>(); }
				if (dynamics.contains("friction") && dynamics["friction"].is_number()) { joint.dynamics.friction = dynamics["friction"].get<float>(); }
			}

			robot.joints.push_back(joint);

			// Load DH parameters (kinematics)
			DH_Params dhp{};
			if (jointData.contains("dh")) {
				auto& dhData = jointData["dh"];
				// a
				if (dhData.contains("a") && dhData["a"].is_number()) { dhp.a = dhData["a"].get<double>(); }
				else { dhp.a = 0.0; }
				// alpha
				if (dhData.contains("alpha") && dhData["alpha"].is_number()) { dhp.alpha = dhData["alpha"].get<double>(); }
				else { dhp.alpha = 0.0; }
				// d
				if (dhData.contains("d") && dhData["d"].is_number()) { dhp.d = dhData["d"].get<double>(); }
				else { dhp.d = 0.0; }
				// theta
				if (dhData.contains("theta") && dhData["theta"].is_number()) { dhp.theta = dhData["theta"].get<double>(); }
				else { dhp.theta = 0.0; }

				std::string typeStr = dhData.value("type", "revolute");
				dhp.type = parseDHType(typeStr);

			} else {
				LOG_WARN("Joint %s missing 'dh' block; using zero DH", joint.name.c_str());
				dhp.a = 0.0; dhp.alpha = 0.0; dhp.d = 0.0; dhp.theta = 0.0; dhp.type = JointType::Revolute;
			}

			robot.dhParams.push_back(dhp);

	
			
			LOG_INFO("Joint: %s | Parent: %s, | Child: %s, | Continuous: %s, | Max Speed: %.2f, | Min Angle: %.2f, | Max Angle: %.2f",
				joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.limits.continuous ? "True" : "False", joint.limits.maxOmegaRad_s, joint.limits.minAngle, joint.limits.maxAngle);
			LOG_INFO("DH Params for Joint %s: a=%.4f, alpha=%.4f, d=%.4f, theta=%.4f, type=%s",
				joint.name.c_str(), dhp.a, dhp.alpha, dhp.d, dhp.theta,
				dhp.type == JointType::Revolute ? "revolute" : "prismatic");

			D_INFO("Joint: %s | Parent: %s, | Child: %s, | Continuous: %s, | Max Speed: %.2f, | Min Angle: %.2f, | Max Angle: %.2f",
				joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.limits.continuous ? "True" : "False", joint.limits.maxOmegaRad_s, joint.limits.minAngle, joint.limits.maxAngle);
			D_INFO("DH Params for Joint %s: a=%.4f, alpha=%.4f, d=%.4f, theta=%.4f, type=%s",
				joint.name.c_str(), dhp.a, dhp.alpha, dhp.d, dhp.theta,
				dhp.type == JointType::Revolute ? "revolute" : "prismatic");
		}

		LOG_INFO("Robot loaded: %d links, %d joints", (int)robot.links.size(), (int)robot.joints.size());
		D_SUCCESS("Robot loaded: %d links, %d joints", (int)robot.links.size(), (int)robot.joints.size());
		return robot;
	}
}