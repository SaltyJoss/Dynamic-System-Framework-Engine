
#include "pch.h"
#include "Robots/RobotLoader.h"

#include "Scene/Object.h"
#include "Scene/MeshLoader.h"
#include "EngineLib/LogMacros.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using kinematics::JointType_DH;
using kinematics::DH_Params;

namespace robots {
	// --- Static Helper Functions ---

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
	static JointType_DH parseDHType(const std::string& s) {
		std::string t = s;
		for (char& c : t) { c = static_cast<char>(std::tolower((unsigned char)c)); }
		if (t == "revolute" || t == "r") { return JointType_DH::Revolute; }
		return JointType_DH::Prismatic;
	}

	// Read a vec3 from a JSON array
	static glm::vec3 readVec3(const json& j, const char* key, glm::vec3 fallback = {}) {
		if (!j.contains(key) || !j[key].is_array() || j[key].size() != 3) { return fallback; }
		return glm::vec3(j[key][0].get<float>(), j[key][1].get<float>(), j[key][2].get<float>());
	}

	// --- RobotLoader Link and Joint Parsing ---

	// link parsing

	static void parseVisual(const json& linkData, RobotLink& link) {
		if (!linkData.contains("visual")) { return; }
		const auto& v = linkData["visual"];
		link.visual.meshFile = v.value("mesh", link.visual.meshFile);
		link.visual.origin_xyz = readVec3(v, "origin_xyz", link.visual.origin_xyz);
		link.visual.origin_rpy = readVec3(v, "origin_rpy", link.visual.origin_rpy);
	}

	static void parseCollisions(const json& linkData, RobotLink& link) {
		if (!linkData.contains("collision")) { return; }
		for (const auto& c : linkData["collision"]) {
			CollisionShape s;
			s.type = c.value("type", "");
			s.origin_xyz = readVec3(c, "origin_xyz", s.origin_xyz);
			s.origin_rpy = readVec3(c, "origin_rpy", s.origin_rpy);

			if (s.type == "cylinder") {
				s.size.x = c.value("radius", 0.0f);  // radius
				s.size.y = c.value("length", 0.0f);  // length
			}
			else if (s.type == "box") { s.size = readVec3(c, "size", s.size); }
			else if (s.type == "mesh") { s.meshFile = c.value("mesh", ""); }
			link.collisions.push_back(s);
		}
	}

	static void parseInertial(const json& linkData, RobotLink& link) {
		if (!linkData.contains("inertial")) { return; }
		const auto& I = linkData["inertial"];
		link.inertial.mass = I.value("mass", link.inertial.mass);
		link.inertial.com_xyz = readVec3(I, "com_xyz", link.inertial.com_xyz);

		if (I.contains("inertia")) {
			const auto& J = I["inertia"];
			link.inertial.inertia.ixx = J.value("ixx", link.inertial.inertia.ixx);
			link.inertial.inertia.ixy = J.value("ixy", link.inertial.inertia.ixy);
			link.inertial.inertia.ixz = J.value("ixz", link.inertial.inertia.ixz);
			link.inertial.inertia.iyy = J.value("iyy", link.inertial.inertia.iyy);
			link.inertial.inertia.iyz = J.value("iyz", link.inertial.inertia.iyz);
			link.inertial.inertia.izz = J.value("izz", link.inertial.inertia.izz);
		}
	}

	// joint parsing

	static void parseJointOrigin(const json& jointData, RobotJoint& joint) {
		if (jointData.contains("origin")) {
			const auto& o = jointData["origin"];
			joint.origin_xyz = readVec3(o, "origin_xyz", joint.origin_xyz);
			joint.origin_rpy = readVec3(o, "origin_rpy", joint.origin_rpy);
		} else {
			joint.origin_xyz = readVec3(jointData, "origin_xyz", glm::vec3(0));
			joint.origin_rpy = readVec3(jointData, "origin_rpy", glm::vec3(0));
		}
		joint.origin_q = rpyRadToQuat(joint.origin_rpy);
	}

	static void parseJointAxis(const json& jointData, RobotJoint& joint) {
		joint.axis = glm::vec3(0.0f, 0.0f, 1.0f); // default axis
		if (jointData.contains("axis") && jointData["axis"].is_array() && jointData["axis"].size() == 3) {
			const auto& a = jointData["axis"];
			joint.axis = glm::vec3(
				a[0].get<float>(),
				a[1].get<float>(),
				a[2].get<float>()
			);
			if (glm::length(joint.axis) < 1e-6f) {
				LOG_WARN("Joint %s has zero-length axis, defaulting to (0,0,1)", joint.name.c_str());
				joint.axis = glm::vec3(0.0f, 0.0f, 1.0f);
			}
			else { joint.axis = glm::normalize(joint.axis); }
		}
	}

	static void parseJointLimits(const json& jointData, RobotJoint& joint) {
		joint.limits.continuous = false;
		joint.limits.minAngle = 0.0f;
		joint.limits.maxAngle = 0.0f;
		joint.limits.maxOmegaRad_s = 0.0f;
		joint.limits.maxEffort = 0.0f;

		if (!jointData.contains("limits") || !jointData["limits"].is_object()) { LOG_WARN("Joint %s missing 'limits' block", joint.name.c_str()); return; }

		const auto& L = jointData["limits"];

		joint.limits.continuous = L.value("continuous", false);
		joint.limits.maxOmegaRad_s = L.value("velocity", joint.limits.maxOmegaRad_s);
		joint.limits.maxEffort = L.value("effort", joint.limits.maxEffort);

		if (!joint.limits.continuous) {
			if (L.contains("lower") && L["lower"].is_number()) { joint.limits.minAngle = L["lower"].get<float>(); }
			else { LOG_WARN("Joint %s limits missing 'lower'", joint.name.c_str()); }

			if (L.contains("upper") && L["upper"].is_number()) { joint.limits.maxAngle = L["upper"].get<float>(); }
			else { LOG_WARN("Joint %s limits missing 'upper'", joint.name.c_str()); }

			if (joint.limits.maxAngle < joint.limits.minAngle) {
				LOG_WARN("Joint %s has upper < lower (swapping).", joint.name.c_str());
				std::swap(joint.limits.minAngle, joint.limits.maxAngle);
			}
		}
		else { joint.limits.minAngle = -3.14159265f; joint.limits.maxAngle = 3.14159265f; }
	}

	static void parseJointDynamics(const json& jointData, RobotJoint& joint) {
		joint.dynamics.damping = 0.0f;
		joint.dynamics.friction = 0.0f;

		if (!jointData.contains("dynamics") || !jointData["dynamics"].is_object()) { return; }

		const auto& D = jointData["dynamics"];
		joint.dynamics.damping = D.value("damping", joint.dynamics.damping);
		joint.dynamics.friction = D.value("friction", joint.dynamics.friction);

		if (joint.dynamics.damping < 0.0f) { joint.dynamics.damping = 0.0f; }
		if (joint.dynamics.friction < 0.0f) { joint.dynamics.friction = 0.0f; }
	}

	static void parseJointControl(const json& jointData, RobotJoint& joint) {
		joint.k_p = 25.0f;
		joint.k_d = 8.0f;
		joint.thetaRefRad = 0.0f;

		if (!jointData.contains("control") || !jointData["control"].is_object()) return;

		const auto& C = jointData["control"];
		joint.k_p = C.value("k_p", joint.k_p);
		joint.k_d = C.value("k_d", joint.k_d);

		// You have this also in limits.velocity. Choose ONE source of truth.
		// I'd prefer: limits.velocity is truth, control.maxOmega is optional override.
		if (C.contains("maxOmega") && C["maxOmega"].is_number()) {
			float maxOmega = C["maxOmega"].get<float>();
			if (maxOmega > 0.0f) joint.limits.maxOmegaRad_s = maxOmega;
		}
	}

	static bool parseDHParameters(const json& jointData, DH_Params& out) {
		// Accept "dh" ONLY (your JSON uses "dh")
		if (!jointData.contains("dh") || !jointData["dh"].is_object()) return false;

		const auto& dh = jointData["dh"];
		out.a = dh.value("a", 0.0);
		out.alpha = dh.value("alpha", 0.0);
		out.d = dh.value("d", 0.0);
		out.theta = dh.value("theta0", 0.0);          // your key is theta0
		out.type = parseDHType(dh.value("type", "revolute"));
		return true;
	}

	static eKinematicsModel decideKinematicsModel(const json& data) {
		if (!data.contains("joints") || !data["joints"].is_array()) { return eKinematicsModel::URDF; } // no joints -> URDF
		for (const auto& jointData : data["joints"]) {
			if (!jointData.contains("dh") || !jointData["dh"].is_object()) { return eKinematicsModel::URDF; } // any missing -> URDF
		}
		return eKinematicsModel::DH;
	}

	// --- RobotLoader Loading, Public API ---

	RobotModel RobotLoader::loadFromJSON(const std::string& filepath) {
		RobotModel robot;
		LOG_INFO("Loading robot model from JSON: %s", filepath.c_str());
		D_INFO("Loading robot model from JSON: %s", filepath.c_str());

		std::ifstream file(filepath);
		if (!file.is_open()) { 
			LOG_ERROR("Failed to open JSON file: %s", filepath.c_str()); 
			D_FAIL("Failed to open JSON file: %s", filepath.c_str());
			return robot; 
		}
		json data = json::parse(file);

		robot.name = data["name"].get<std::string>();
		robot.scale = data["scale"].get<float>();


		// Load links
		for (auto& linkData : data["links"]) {
			RobotLink link;
			link.name = linkData.value("name", "");

			parseVisual(linkData, link);
			parseCollisions(linkData, link);
			parseInertial(linkData, link);

			robot.links.push_back(link);

			LOG_INFO("Link: %s | Mass: %.2f", link.name.c_str(), link.inertial.mass);
			D_INFO("Link: %s | Mass: %.2f", link.name.c_str(), link.inertial.mass);
		}

		// Decide kinematics model
		robot.kinematicsModel = decideKinematicsModel(data);

		// Load joints
		for (auto& jointData : data["joints"]) {
			RobotJoint joint;

			// Load basic joint info
			joint.name = jointData["name"].get<std::string>();
			joint.parent = jointData["parent"].get<std::string>();
			joint.child = jointData["child"].get<std::string>();

			parseJointOrigin(jointData, joint);
			parseJointAxis(jointData, joint);
			parseJointLimits(jointData, joint);
			parseJointDynamics(jointData, joint);
			parseJointControl(jointData, joint);

			robot.joints.push_back(joint);

			// If robot is DH-mode, also parse DH table
			if (robot.kinematicsModel == eKinematicsModel::DH) {
				DH_Params dh{};
				if (!parseDHParameters(jointData, dh)) {
					LOG_WARN("Joint %s missing 'dh' unexpectedly; forcing URDF mode.", joint.name.c_str());
					robot.kinematicsModel = eKinematicsModel::URDF;
					robot.dhParams.clear();
				}
				else {
					robot.dhParams.push_back(dh);
				}
			}

			LOG_INFO("Joint: %s | Parent: %s, | Child: %s, | Continuous: %s, | Max Speed: %.2f, | Min Angle: %.2f, | Max Angle: %.2f",
				joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.limits.continuous ? "True" : "False", joint.limits.maxOmegaRad_s, joint.limits.minAngle, joint.limits.maxAngle);
			D_INFO("Joint: %s | Parent: %s, | Child: %s, | Continuous: %s, | Max Speed: %.2f, | Min Angle: %.2f, | Max Angle: %.2f",
				joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.limits.continuous ? "True" : "False", joint.limits.maxOmegaRad_s, joint.limits.minAngle, joint.limits.maxAngle);

		}

		// If we forced URDF mid-way, ensure DH is empty
		if (robot.kinematicsModel == eKinematicsModel::URDF) {
			robot.dhParams.clear();
		}

		LOG_INFO("Robot loaded: %d links, %d joints", (int)robot.links.size(), (int)robot.joints.size());
		D_SUCCESS("Robot loaded: %d links, %d joints", (int)robot.links.size(), (int)robot.joints.size());

		return robot;
	}
}