#include "pch.h"
// File:   RobotLoader.cpp
// GitHub: SaltyJoss
#include "Robots/RobotLoader.h"

#include <MathLibAPI.h>
#include <core/constants.h>
#include "Scene/Object.h"
#include "Assets/MeshLoader.h"
#include "EngineLib/LogMacros.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using kinematics::JointType_DH;
using kinematics::DH_Params;

using namespace constants;

namespace robots {
	// --- Static Helper Functions ---

	// tf2::Quaternion::setRPY(roll,pitch,yaw) corresponds to q = qz * qy * qx.
	static Quat rpyRadToQuat(const Vec3& rpyRad)
	{
		const double roll  = rpyRad.x();
		const double pitch = rpyRad.y();
		const double yaw   = rpyRad.z();

		const Quat qx(Eigen::AngleAxisd(roll,  Vec3(1.0, 0.0, 0.0)));
		const Quat qy(Eigen::AngleAxisd(pitch, Vec3(0.0, 1.0, 0.0)));
		const Quat qz(Eigen::AngleAxisd(yaw,   Vec3(0.0, 0.0, 1.0)));

		return (qz * qy * qx).normalized();
	}

	// Parse DH joint type from string
	static JointType_DH parseDHType(const std::string& s) {
		std::string t = s;
		for (char& c : t) { c = static_cast<char>(std::tolower((unsigned char)c)); }
		if (t == "revolute" || t == "r") { return JointType_DH::Revolute; }
		return JointType_DH::Prismatic;
	}

	// Read a vec3 from a JSON array
	static Vec3 readVec3(const json& j, const char* key, Vec3 fallback = {}) {
		if (!j.contains(key) || !j[key].is_array() || j[key].size() != 3) { return fallback; }
		return Vec3(j[key][0].get<double>(), j[key][1].get<double>(), j[key][2].get<double>());
	}

	// --- RobotLoader Link and Joint Parsing ---

	// link parsing

	// Parse materials block if present
	// Each material is defined as: "materials": { "mat_name": [r, g, b, a] }
	void loadMaterials(const json& data, RobotModel& robot) {
		if (!data.contains("material")) { return; }
		const auto& mat = data["material"];

		// New nested format: "material" -> "Color" -> { name: [r,g,b,a] }
		if (mat.contains("Color") && mat["Color"].is_object()) {
			for (auto& [name, col] : mat["Color"].items()) {
				if (col.is_array() && col.size() == 4) {
					robot.materials[name] = Vec4(
						col[0].get<double>(),
						col[1].get<double>(),
						col[2].get<double>(),
						col[3].get<double>()
					);
				}
				else {
					LOG_WARN("Material Color '%s' has invalid format, expected array of 4 floats", name.c_str());
				}
			}
		}
		else {
			// Legacy flat format: "material" -> { name: [r,g,b,a] }
			for (auto& [name, col] : mat.items()) {
				if (col.is_array() && col.size() == 4) {
					robot.materials[name] = Vec4(
						col[0].get<double>(),
						col[1].get<double>(),
						col[2].get<double>(),
						col[3].get<double>()
					);
				}
			}
		}
	}

	// Helper to parse material properties from a JSON object into color/metallic/roughness
	static void parseMaterialObject(const json& m, const RobotModel& robot, const std::string& context,
		Vec4& outColor, float& outMetallic, float& outRoughness) {
		if (m.contains("Color") && m["Color"].is_string()) {
			const std::string colorName = m["Color"].get<std::string>();
			auto it = robot.materials.find(colorName);
			if (it != robot.materials.end()) {
				outColor = it->second;
			}
			else {
				LOG_WARN("%s references undefined color '%s', using default Grey", context.c_str(), colorName.c_str());
				outColor = Vec4(0.4, 0.4, 0.4, 1.0);
			}
		}
		if (m.contains("Metallic") && m["Metallic"].is_number()) {
			outMetallic = m["Metallic"].get<float>();
		}
		if (m.contains("Roughness") && m["Roughness"].is_number()) {
			outRoughness = m["Roughness"].get<float>();
		}
	}

	// Parse visual geometry, supporting both single mesh and multiple meshes, as well as material properties
	static void parseVisual(const json& linkData, const RobotModel& robot, RobotLink& link) {
		if (!linkData.contains("visual")) { return; }
		const auto& v = linkData["visual"];

		// Single Mesh (e.g. URDF style)
		if (v.contains("mesh") && v["mesh"].is_string()) {
			link.visual.meshFile = v["mesh"].get<std::string>();
		}

		// Multiple meshes - supports both string arrays and object arrays with per-mesh material
		if (v.contains("meshes") && v["meshes"].is_array()) {
			link.visual.meshFiles.clear();
			link.visual.meshEntries.clear();
			for (const auto& m : v["meshes"]) {
				if (m.is_string()) {
					// Legacy string format
					link.visual.meshFiles.push_back(m.get<std::string>());
				}
				else if (m.is_object() && m.contains("mesh") && m["mesh"].is_string()) {
					// Object format with per-mesh material
					VisualMeshEntry entry;
					entry.meshFile = m["mesh"].get<std::string>();

					if (m.contains("material") && m["material"].is_object()) {
						entry.hasMaterial = true;
						parseMaterialObject(m["material"], robot, "Mesh " + entry.meshFile,
							entry.material, entry.metallic, entry.roughness);
					}

					link.visual.meshEntries.push_back(entry);
				}
			}
		}

		// Visual geometry origin
		link.visual.origin_xyz = readVec3(v, "origin_xyz", link.visual.origin_xyz);
		link.visual.origin_rpy = readVec3(v, "origin_rpy", link.visual.origin_rpy);

		// Material
		if (v.contains("material") && v["material"].is_object()) {
			parseMaterialObject(v["material"], robot, "Link " + link.name,
				link.visual.material, link.visual.metallic, link.visual.roughness);
		}
		else if (v.contains("material") && v["material"].is_string()) {
			// Legacy string format: material name lookup
			const std::string matName = v["material"].get<std::string>();
			auto it = robot.materials.find(matName);
			if (it != robot.materials.end()) {
				link.visual.material = it->second;
			}
			else {
				LOG_WARN("Link %s references undefined material '%s', using default Grey", link.name.c_str(), matName.c_str());
				link.visual.material = Vec4(0.5, 0.5, 0.5, 1.0); // Default grey material if material name not found
			}
		}
		else {
			// Default material if not specified
			link.visual.material = Vec4(0.2, 0.4, 0.5, 1.0); // Default color for visual geometry if no material is specified
		}
	}

	// Parse collision geometry material properties
	static void parseCollisionMaterial(const json& collisionData, const RobotModel& robot, CollisionShape& shape) {
		if (!collisionData.contains("material")) { return; }
		const auto& m = collisionData["material"];

		if (m.is_object()) {
			parseMaterialObject(m, robot, "Collision",
				shape.material, shape.metallic, shape.roughness);
		}
	}

	// Parse collision geometry, supporting multiple collision shapes per link
	static void parseCollisions(const json& linkData, const RobotModel& robot, RobotLink& link) {
		if (!linkData.contains("collision")) { return; }
		for (const auto& c : linkData["collision"]) {
			CollisionShape s;
			s.type = c.value("type", "");
			s.origin_xyz = readVec3(c, "origin_xyz", s.origin_xyz);
			s.origin_rpy = readVec3(c, "origin_rpy", s.origin_rpy);

			if (s.type == "cylinder") {
				s.size.x() = c.value("radius", 0.0f);  // radius
				s.size.y() = c.value("length", 0.0f);  // length
			}
			else if (s.type == "box") { s.size = readVec3(c, "size", s.size); }
			else if (s.type == "mesh") { s.meshFile = c.value("mesh", ""); }

			parseCollisionMaterial(c, robot, s);
			link.collisions.push_back(s);
		}
	}

	// Parse inertial properties, including mass, center of mass, and inertia tensor
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

	// Parse joint origin, supporting both the "origin" block (with "origin_xyz" and "origin_rpy" inside) and the flat format with "origin_xyz" and "origin_rpy" directly in the joint block
	static void parseJointOrigin(const json& jointData, RobotJoint& joint) {
		if (jointData.contains("origin")) {
			const auto& o = jointData["origin"];
			joint.origin_xyz = readVec3(o, "origin_xyz", joint.origin_xyz);
			joint.origin_rpy = readVec3(o, "origin_rpy", joint.origin_rpy);
		} else {
			joint.origin_xyz = readVec3(jointData, "origin_xyz", Vec3::Zero());
			joint.origin_rpy = readVec3(jointData, "origin_rpy", Vec3::Zero());
		}
		joint.origin_q = rpyRadToQuat(joint.origin_rpy);
	}

	// Parse joint axis, supporting both the "axis" block (with "axis_xyz" inside) and the flat format with "axis" directly in the joint block
	static void parseJointAxis(const json& jointData, RobotJoint& joint) {
		joint.axis = Vec3(0.0f, 0.0f, 1.0f); // default axis
		if (jointData.contains("axis") && jointData["axis"].is_array() && jointData["axis"].size() == 3) {
			const auto& a = jointData["axis"];
			joint.axis = Vec3(
				a[0].get<double>(),
				a[1].get<double>(),
				a[2].get<double>()
			);
			if (joint.axis.norm() < 1e-6f) {
				LOG_WARN("Joint %s has zero-length axis, defaulting to (0,0,1)", joint.name.c_str());
				joint.axis = Vec3(0.0f, 0.0f, 1.0f);
			}
			else { joint.axis.normalize(); }
		}

		// Parse joint type (e.g., "revolute", "prismatic") if provided.
		if (jointData.contains("type") && jointData["type"].is_string()) {
			const std::string typeStr = jointData["type"].get<std::string>();
			if (typeStr == "revolute" || typeStr == "REVOLUTE") {
				joint.type = eJointType::REVOLUTE;
			}
			else if (typeStr == "prismatic" || typeStr == "PRISMATIC") {
				joint.type = eJointType::PRISMATIC;
			}
			else if (typeStr == "fixed" || typeStr == "FIXED") {
				joint.type = eJointType::FIXED;
			}
			else if (typeStr == "free" || typeStr == "FREE") {
				joint.type = eJointType::FREE;
			}
			else {
				LOG_WARN("Joint %s has unknown type '%s', defaulting to REVOLUTE", joint.name.c_str(), typeStr.c_str());
			}
		}
	}

	// Parse joint limits, including continuous revolute joints and prismatic joints
	static void parseJointLimits(const json& jointData, RobotJoint& joint) {
		joint.limits.continuous		= false;
		joint.limits.minAngle		= 0.0f;
		joint.limits.maxAngle		= 0.0f;
		joint.limits.maxqd	= 0.0f;
		joint.limits.maxEffort		= 0.0f;

		if (!jointData.contains("limits") || !jointData["limits"].is_object()) { LOG_WARN("Joint %s missing 'limits' block", joint.name.c_str()); return; }

		const auto& L = jointData["limits"];

		joint.limits.continuous = L.value("continuous", false);
		joint.limits.maxqd = L.value("velocity", joint.limits.maxqd);
		joint.limits.maxEffort = L.value("effort", joint.limits.maxEffort);

		if (!joint.limits.continuous) {
			if (L.contains("lower") && L["lower"].is_number()) { joint.limits.minAngle = L["lower"].get<double>(); }
			else { LOG_WARN("Joint %s limits missing 'lower'", joint.name.c_str()); }

			if (L.contains("upper") && L["upper"].is_number()) { joint.limits.maxAngle = L["upper"].get<double>(); }
			else { LOG_WARN("Joint %s limits missing 'upper'", joint.name.c_str()); }

			if (joint.limits.maxAngle < joint.limits.minAngle) {
				LOG_WARN("Joint %s has upper < lower (swapping).", joint.name.c_str());
				std::swap(joint.limits.minAngle, joint.limits.maxAngle);
			}
		}
		else { joint.limits.minAngle = -3.14159265f; joint.limits.maxAngle = 3.14159265f; }
	}

	// Parse joint dynamics parameters
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

	// Check if a joint is fixed based on its type string
	static bool isFixedJoint(const json& jointData) {
		if (!jointData.contains("type")) return false;
		const std::string t = jointData["type"].get<std::string>();
		return (t == "fixed" || t == "FIXED");
	}

	// Parse DH parameters if present
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

	// Decide kinematics model based on presence of DH parameters
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
		loadMaterials(data, robot);

		// Visual frame (optional, defaults to JOINT)
		if (data.contains("visual_frame")) {
			const std::string vf = data["visual_frame"].get<std::string>();
			if (vf == "joint")		{ robot.visualFrame = eVisualFrame::JOINT; }
			else if (vf == "link")  { robot.visualFrame = eVisualFrame::LINK; }
			else if (vf == "world") { robot.visualFrame = eVisualFrame::WORLD; }
			else { LOG_WARN("Unknown visual_frame '%s', defaulting to JOINT", vf.c_str()); }
		}
		else {
			robot.visualFrame = eVisualFrame::JOINT;
		}

		// Load robot scale (default 1.0)
		robot.scale = data["scale"].get<float>();

		// Load base frame if present
		if (data.contains("base_frame")) {
			robot.baseFrameIsEngineAligned = false;
			const auto& bf = data["base_frame"];

			// Read translation and rotation (RPY) from JSON, with defaults
			Vec3 t = readVec3(bf, "origin_xyz", Vec3::Zero());
			Vec3 r = readVec3(bf, "origin_rpy", Vec3::Zero());
			Quat q = rpyRadToQuat(r);

			robot.baseFrame = Mat4::Identity();
			robot.baseFrame.block<3, 3>(0, 0) = q.toRotationMatrix();
			robot.baseFrame.block<3, 1>(0, 3) = t;

			LOG_INFO("Base frame loaded from JSON: translation=(%.3f, %.3f, %.3f), rotation_rpy=(%.3f, %.3f, %.3f)", 
				t.x(), t.y(), t.z(), 
				r.x(), r.y(), r.z());
		}
		else {
			robot.baseFrameIsEngineAligned = true;
			robot.baseFrame = Mat4::Identity();
			LOG_INFO("No base frame specified in JSON, using identity (engine-aligned) by default.");
		}

		if (robot.name == "Z1") {
			robot.baseFrameIsEngineAligned = true;
		}

		// Load links
		for (auto& linkData : data["links"]) {
			RobotLink link;
			link.name = linkData.value("name", "");

			parseVisual(linkData, robot, link);
			parseCollisions(linkData, robot, link);
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

			// If it's a fixed joint, we can skip axis/limits/dynamics/control parsing and just set defaults.
			if (isFixedJoint(jointData)) {
				joint.type = eJointType::FIXED;
				joint.axis = Vec3::Zero();
				joint.limits.continuous = false;
				joint.limits.minAngle = 0.0f;
				joint.limits.maxAngle = 0.0f;
			}
			else {
				parseJointAxis(jointData, joint);
				parseJointLimits(jointData, joint);
				parseJointDynamics(jointData, joint);
			}

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

			if (abs(joint.limits.minAngle) == abs(joint.limits.maxAngle) && !joint.limits.continuous) {
				LOG_INFO("Joint: %s | Parent: %s, | Child: %s, | Max Speed: %.2f, | Angle Limit: +-%.2f",
					joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.limits.maxqd, joint.limits.maxAngle);
				D_INFO("Joint: %s | Parent: %s, | Child: %s, | Max Speed: %.2f, | Angle Limit: +-%.2f",
					joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.limits.maxqd, joint.limits.maxAngle);
			}
			else {
				LOG_INFO("Joint: %s | Parent: %s, | Child: %s, | Continuous: %s, | Max Speed: %.2f, | Min Angle: %.2f, | Max Angle: %.2f",
					joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.limits.continuous ? "True" : "False", joint.limits.maxqd, joint.limits.minAngle, joint.limits.maxAngle);
				D_INFO("Joint: %s | Parent: %s, | Child: %s, | Continuous: %s, | Max Speed: %.2f, | Min Angle: %.2f, | Max Angle: %.2f",
					joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.limits.continuous ? "True" : "False", joint.limits.maxqd, joint.limits.minAngle, joint.limits.maxAngle);
			}
		}

		if (robot.kinematicsModel == eKinematicsModel::URDF) { robot.dhParams.clear(); }

		LOG_INFO("Robot loaded: %d links, %d joints", (int)robot.links.size(), (int)robot.joints.size());
		D_SUCCESS("Robot loaded: %d links, %d joints", (int)robot.links.size(), (int)robot.joints.size());

		return robot;
	}
}