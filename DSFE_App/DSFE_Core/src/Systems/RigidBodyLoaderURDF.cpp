/*
 * File: Systems/RigidBodyLoaderURDF.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "Systems/RigidBodyLoader.h"
#include <MathLib>

#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"
#include <tinyxml2.h>

#include <string>
#include <set>
#include <algorithm>
#include <sstream>
#include <filesystem>

using namespace tinyxml2;
using namespace constants;

namespace systems {
    // rpy(radians) -> quaternion, q = qz*qy*qx, where qx = roll, qy = pitch, qz = yaw
    static Quat urdf_rpyToQuat(const mathlib::Vec3& rpy) {
		const Quat qx(Eigen::AngleAxisd(rpy.x(), Vec3(1.0, 0.0, 0.0)));
		const Quat qy(Eigen::AngleAxisd(rpy.y(), Vec3(0.0, 1.0, 0.0)));
		const Quat qz(Eigen::AngleAxisd(rpy.z(), Vec3(0.0, 0.0, 1.0)));
		return (qz * qy * qx).normalized();
	}
    // Parse a space-separated triple of doubles from a string, e.g. "1.0 2.0 3.0"
    static Vec3 parseTriple(const char* s, mathlib::Vec3 fallback = Vec3::Zero()) {
        if (!s) { return fallback; }
        std::istringstream iss(s);
        double x = fallback.x(), y = fallback.y(), z = fallback.z();
        iss >> x >> y >> z;
        return mathlib::Vec3(x, y, z);
    }
    // Translate a URDF mesh path to a platform-specific path, e.g. "package://my_robot/meshes/part.stl" -> "rigidbody_models/airbus_vispa/my_robot/meshes/part.stl"
    static std::string translateMeshPath(const std::string& raw, const std::string& meshdir) {
        std::string p = raw;
        // Remove "package://" prefix if present
        const std::string pkg = "package://";
        if (p.rfind(pkg, 0) == 0) { p = p.substr(pkg.size()); }
        // Replace forward slashes with platform-specific separators
        std::replace(p.begin(), p.end(), '\\', '/');
        const auto slash = p.find_last_of('/');
        const std::string fn = (slash == std::string::npos) ? p : p.substr(slash + 1);
        return meshdir + "/" + fn;
    }
    // Link Parsing
    static void urdf_parseLink(XMLElement* lEl, RigidBodyLink& link, const std::string& meshdir) {
        std::string name = lEl->Attribute("name") ? lEl->Attribute("name") : "";
        // convert to lowercase for consistency
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
        link.name = name;
        // Visual
        if (XMLElement* v = lEl->FirstChildElement("visual")) {
            // Origin
            if (XMLElement* o = v->FirstChildElement("origin")) {
                link.visual.origin_xyz = parseTriple(o->Attribute("xyz"), link.visual.origin_xyz);
                link.visual.origin_rpy = parseTriple(o->Attribute("rpy"), link.visual.origin_rpy);   
            }
            // Geometry & Mesh
            if (XMLElement* g = v->FirstChildElement("geometry")) {
                if (XMLElement* m = g->FirstChildElement("mesh")) {
                    if (const char* fn = m->Attribute("filename")) { 
                        VisualMeshEntry entry;
                        entry.meshFile = translateMeshPath(fn, meshdir);
                        link.visual.meshEntries.push_back(entry);
                    }
                }
            }
            if (XMLElement* m = v->FirstChildElement("dsfe_material")) {
                double r=0.7,g=0,b=0.2,a=1;
                if (const char* rgba = m->Attribute("rgba")) {
                    std::istringstream iss(rgba); iss >> r >> g >> b >> a;
                }
                float metallic = 0.1f, roughness = 0.65f;
                m->QueryFloatAttribute("metallic", &metallic);
                m->QueryFloatAttribute("roughness", &roughness);
                for (auto& entry : link.visual.meshEntries) {
                    LOG_INFO("Link %s has <dsfe_material> with rgba: %f %f %f %f, metallic: %f, roughness: %f", link.name.c_str(), r, g, b, a, metallic, roughness);
                    entry.material = Vec4(r, g, b, a);
                    entry.metallic = metallic;
                    entry.roughness = roughness;
                    entry.hasMaterial = true;
                }
            } else {
			    LOG_WARN("Link %s has NO <dsfe_material>", link.name.c_str());
			}
        }
        // Collision
        if (XMLElement* col = lEl->FirstChildElement("collision")) {
            if (XMLElement* o = col->FirstChildElement("origin")) {
                link.collision.origin_xyz = parseTriple(o->Attribute("xyz"), mathlib::Vec3::Zero());
                link.collision.origin_rpy = parseTriple(o->Attribute("rpy"), mathlib::Vec3::Zero());
            }
            if (XMLElement* cap = col->FirstChildElement("dsfe_capsule")) {
                link.collision.type = eCollisionShape::CAPSULE;
                link.collision.localA = parseTriple(cap->Attribute("a"), mathlib::Vec3::Zero());
                link.collision.localB = parseTriple(cap->Attribute("b"), mathlib::Vec3::Zero());
                double radius = 0.05; // default radius
                cap->QueryDoubleAttribute("radius", &radius);
                link.collision.radius = radius;
                LOG_INFO("%s (link) has <dsfe_capsule> a=[%.3f %.3f %.3f] b=[%.3f %.3f %.3f] r=%.3f", link.name.c_str(), 
                    link.collision.localA.x(), link.collision.localA.y(), link.collision.localA.z(), 
                    link.collision.localB.x(), link.collision.localB.y(), link.collision.localB.z(),
                    link.collision.radius
                );
            }
            else if (XMLElement* geom = col->FirstChildElement("geometry")) {
                if (XMLElement* box = geom->FirstChildElement("box")) {
                    link.collision.type = eCollisionShape::BOX;
                    mathlib::Vec3 size = parseTriple(box->Attribute("size"), mathlib::Vec3(1,1,1));
                    link.collision.halfExtents = size * 0.5; // URDF box size is full extents, we store half extents
                }
                else if (XMLElement* sphere = geom->FirstChildElement("sphere")) {
                    link.collision.type = eCollisionShape::SPHERE;
                    double r = 0.05; // default radius
                    sphere->QueryDoubleAttribute("radius", &r);
                    link.collision.radius = r;
                }
                else if (XMLElement* cyl = geom->FirstChildElement("cylinder")) {
                    link.collision.type = eCollisionShape::CYCLINDER;
                    double r = 0.05; // default radius
                    double len = 0.1; // default length
                    cyl->QueryDoubleAttribute("radius", &r);
                    cyl->QueryDoubleAttribute("length", &len);
                    link.collision.radius = r;
                    link.collision.localA = mathlib::Vec3(0, 0, -len * 0.5);
                    link.collision.localB = mathlib::Vec3(0, 0, len * 0.5);
                }
                else if (XMLElement* mesh = geom->FirstChildElement("mesh")) {
                    link.collision.type = eCollisionShape::MESH;
                    if (const char* fn = mesh->Attribute("filename")) { 
                        link.collision.meshFile = translateMeshPath(fn, meshdir);
                    }
                }
            }
        }
        // Inertial
        if (XMLElement* i = lEl->FirstChildElement("inertial")) {
            // Mass
            if (XMLElement* m = i->FirstChildElement("mass")) {
                if (m->Attribute("value")) { link.inertial.mass = std::stod(m->Attribute("value")); }
            }
            // Origin
            if (XMLElement* o = i->FirstChildElement("origin")) {
                link.inertial.com_xyz = parseTriple(o->Attribute("xyz"), link.inertial.com_xyz);
            }
            // Inertia
            if (XMLElement* I = i->FirstChildElement("inertia")) {
                I->QueryDoubleAttribute("ixx", &link.inertial.inertia.ixx);
                I->QueryDoubleAttribute("ixy", &link.inertial.inertia.ixy);
                I->QueryDoubleAttribute("ixz", &link.inertial.inertia.ixz);
                I->QueryDoubleAttribute("iyy", &link.inertial.inertia.iyy);
                I->QueryDoubleAttribute("iyz", &link.inertial.inertia.iyz);
                I->QueryDoubleAttribute("izz", &link.inertial.inertia.izz);
            }
        }
    }
    // Joint Parsing
    static eJointType urdf_jointType(const std::string& t) {
        if (t == "revolute")   { return eJointType::REVOLUTE; }
        if (t == "continuous") { return eJointType::REVOLUTE; }
        if (t == "prismatic")  { return eJointType::PRISMATIC; }
        if (t == "fixed")      { return eJointType::FIXED; }
        if (t == "floating")   { return eJointType::FREE; }
        return eJointType::REVOLUTE; // default to revolute if unknown
    }
    // Parse a URDF joint element into a RigidBodyJoint structure
    static void urdf_parseJoint(XMLElement* jEl, RigidBodyJoint& joint) {
        joint.name = jEl->Attribute("name") ? jEl->Attribute("name") : "";
        const std::string typeStr = jEl->Attribute("type") ? jEl->Attribute("type") : "revolute";
        const bool continuous = (typeStr == "continuous");
        joint.type = urdf_jointType(typeStr);
        // Parse parent and child links
        if (XMLElement* p = jEl->FirstChildElement("parent")) {
            joint.parent = p->Attribute("link") ? p->Attribute("link") : "";
        }
        if (XMLElement* c = jEl->FirstChildElement("child")) {
            joint.child = c->Attribute("link") ? c->Attribute("link") : "";
        }
        // Origin
        if (XMLElement* o = jEl->FirstChildElement("origin")) {
            joint.origin_xyz = parseTriple(o->Attribute("xyz"), joint.origin_xyz);
            joint.origin_rpy = parseTriple(o->Attribute("rpy"), joint.origin_rpy);
        }
        joint.origin_q = urdf_rpyToQuat(joint.origin_rpy);
        // Fixed joint special case: set axis to zero and limits to zero
        if (joint.type == eJointType::FIXED) {
            joint.axis = Vec3::Zero();
            joint.limits.continuous = false;
            joint.limits.minAngle = 0.0;
            joint.limits.maxAngle = 0.0;
            return;
        }
        // Axis
        if (XMLElement* a = jEl->FirstChildElement("axis")) {
            joint.axis = parseTriple(a->Attribute("xyz"), Vec3(0, 0, 1));
            if (joint.axis.norm() < 1e-6) { joint.axis = Vec3(0, 0, 1); } // default axis if zero
            else { joint.axis.normalize(); }
        } else {
            joint.axis = Vec3(0, 0, 1); // default axis if not specified
        }
        // Limits
        if (XMLElement* l = jEl->FirstChildElement("limit")) {
            l->QueryDoubleAttribute("effort",   &joint.limits.maxEffort);
            l->QueryDoubleAttribute("lower",    &joint.limits.minAngle);
            l->QueryDoubleAttribute("upper",    &joint.limits.maxAngle);
            l->QueryDoubleAttribute("velocity", &joint.limits.maxqd);
            if (!continuous) {
                if (joint.limits.minAngle > joint.limits.maxAngle) {
                    std::swap(joint.limits.minAngle, joint.limits.maxAngle);
                }
            }
        } 
        if (continuous) { joint.limits.minAngle = -PI_d; joint.limits.maxAngle = PI_d; }

        // Unlike my URDF-style JSON, URDF does not specify any control gains
        // CHANGE AS NEEDED BUT DO NOT REMOVE (I will likely update my model to have a more robust way of dealing with these soon)
        joint.dynamics.damping = 0.2;
        joint.dynamics.friction = 0.05;
        joint.wn_target = 2.5;
        joint.zeta_target = 1.0;
    }

    // Public API for loading a URDF file into a RigidBodyModel
    RigidBodyModel RigidBodyLoader::loadFromURDF(const std::string& fp) {
        RigidBodyModel rb;
        LOG_INFO("Loading rigidbody model from URDF file: %s", fp.c_str());
        XMLDocument doc;
        if (doc.LoadFile(fp.c_str()) != XML_SUCCESS) {
            LOG_ERROR("Failed to load URDF file: %s (%s)", fp.c_str(), doc.ErrorStr());
            return rb;
        }

        // Robot Element (may update for more generalised applications, though I know URDF's are usually robot-based configurations)
        XMLElement* robot = doc.FirstChildElement("robot");
        if (!robot) { LOG_ERROR("No <robot> element found in URDF file: %s", fp.c_str()); return rb; }
        rb.name = robot->Attribute("name") ? robot->Attribute("name") : "unnamed_body";
        rb.scale = robot->QueryFloatAttribute("scale", &rb.scale) == XML_SUCCESS ? rb.scale : 1.0;
        rb.kinematicsModel = eKinematicsModel::URDF;

        // URDF has NO baseframe, most of the models I use need a Z-up -> engine -90deg X rotation.
        // We will default to this idea, BUT I will NEED to make this more configurable in the future for actual usability.
        rb.baseFrameIsEngineAligned = false;
        {
            mathlib::Quat q = urdf_rpyToQuat(mathlib::Vec3(-PI_d / 2.0, 0.0, 0.0)); // Default base frame rotation to align URDF Z-up to engine Y-up
            rb.baseFrame = mathlib::Mat4::Identity(); // Initialise base frame to identity
            rb.baseFrame.block<3, 3>(0, 0) = q.toRotationMatrix(); // Set the rotation part of the base frame to the quaternion's rotation matrix
        }

        // Links
        const std::filesystem::path urdfDir = std::filesystem::path(fp).parent_path();
        const std::filesystem::path assetRoot = paths::assets();
        // Attempts to compute the relative path from the URDF directory to the assets root, and appends "meshes" to it for mesh file resolution
        std::string meshDirRel;
        {
            std::error_code ec;
            auto rel = std::filesystem::relative(urdfDir, paths::assets(), ec);
            meshDirRel = ec ? urdfDir.string() : rel.string();
            std::replace(meshDirRel.begin(), meshDirRel.end(), '\\', '/'); // Ensures forward slashes for consistency across platforms
            meshDirRel += "/meshes"; // Append "meshes" to the relative path for mesh files
        }
        // Parse links and joints from the URDF
        for (XMLElement* lEl = robot->FirstChildElement("link"); lEl; lEl = lEl->NextSiblingElement("link")) {
            RigidBodyLink link;
            urdf_parseLink(lEl, link, meshDirRel);
            rb.links.push_back(link);
            LOG_INFO("Link: %s | Mass: %.3f", link.name.c_str(), link.inertial.mass);
        }
        // Joints
        for (XMLElement* jEl = robot->FirstChildElement("joint"); jEl; jEl = jEl->NextSiblingElement("joint")) {
            RigidBodyJoint joint;
            urdf_parseJoint(jEl, joint);
            rb.joints.push_back(joint);
            LOG_INFO("Joint: %s | %s -> %s | type=%d | axis=(%.3f, %.3f, %.3f) | maxEffort=%.3f | limits=(%.3f, %.3f) | maxVelocity=%.3f",
                joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), static_cast<int>(joint.type),
                joint.axis.x(), joint.axis.y(), joint.axis.z(),
                joint.limits.maxEffort, joint.limits.minAngle, joint.limits.maxAngle, joint.limits.maxqd
            );
        }
        // Find links that are no joint's child (roots)
		std::set<std::string> child_links;
		for (const auto& j : rb.joints) { child_links.insert(j.child); }
		for (const auto& link : rb.links) {
			if (child_links.find(link.name) == child_links.end()) {
				// This link has no parent joint. If it's the ONLY link (single body), make it FREE.
				if (rb.links.size() == 1) {
					RigidBodyJoint freeJoint;
					freeJoint.name = "free_" + link.name;
					freeJoint.type = eJointType::FREE;
					freeJoint.parent = "world";
					freeJoint.child = link.name;
                    freeJoint.free_pos = Vec3(0.0, 0.5, 0.0);   // 50cm up
                    freeJoint.free_qref = Quat(1,0,0,0);
                    freeJoint.free_rot_v = Vec3::Zero();
                    freeJoint.free_vel = VecX::Zero(6);
					rb.joints.push_back(freeJoint);
					LOG_INFO("Synthesized FREE joint for single free body '%s'", link.name.c_str());
                    LOG_INFO("RigidBody (%s) origin q(joint): (%.3f, %.3f, %.3f, %.3f), origin xyz(joint): (%.3f, %.3f, %.3f)", link.name.c_str(),
                        freeJoint.origin_q.w(), freeJoint.origin_q.x(), freeJoint.origin_q.y(), freeJoint.origin_q.z(),
                        freeJoint.origin_xyz.x(), freeJoint.origin_xyz.y(), freeJoint.origin_xyz.z()
                    );
				}
			}
            LOG_INFO("RigidBody (%s) origin xyz(link): (%.3f, %.3f, %.3f)", link.name.c_str(),
                link.visual.origin_xyz.x(), link.visual.origin_xyz.y(), link.visual.origin_xyz.z()
            );
		}
        LOG_INFO("RigidBody (URDF) loaded: %d links, %d joints", static_cast<int>(rb.links.size()), static_cast<int>(rb.joints.size()));
        return rb;
    }
} // namespace systems