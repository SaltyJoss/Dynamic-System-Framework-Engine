/*
 * File: Physics/CollisionResolver.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"
#include "Physics/CollisionResolver.h"

#include <collision/OBB.h>
#include <collision/SAT.h>
#include <collision/contact.h>
#include <collision/contact_solver.h>

#include "EngineLib/LogMacros.h"

namespace physics {
    /*
     * Helper functions
     */
    // Creates an OBB from a RigidBodySystem's free body state and local AABB
    static std::optional<physlib::collision::OBB> makeOBB(const systems::RigidBodySystem& body) {
        mathlib::Vec3 pos, linVel, angVel;
        mathlib::Quat orientation;
        if (!body.freeBodyState(pos, orientation, linVel, angVel)) { return std::nullopt; }
        mathlib::Vec3 aabbMin, aabbMax;
        if (!body.freeBodyLocalAABB(aabbMin, aabbMax)) { return std::nullopt; }
        physlib::collision::AABB local{ aabbMin, aabbMax };
        const mathlib::Mat3 R = orientation.toRotationMatrix();
        return physlib::collision::OBB::fromAABB(local, R, pos);
    }

    /*
     * CollisionResolver class implementation
     */
    // Constructor & Destructor
    CollisionResolver::CollisionResolver() = default;
    CollisionResolver::~CollisionResolver() = default;

    // Resolves collisions between a set of RigidBodySystems and updates their states accordingly
    void CollisionResolver::resolveContact_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, mathlib::Vec3& norm, physlib::collision::ContactPoint& p, double e, double mu) {
        // Create BodyInfo instances for both bodies to extract relevant physical properties
        auto a = BodyInfo(A); auto b = BodyInfo(B);
        const mathlib::Mat3 R_A = a.ori.toRotationMatrix();
        const mathlib::Mat3 R_B = b.ori.toRotationMatrix();
        const mathlib::Mat3 I_A = R_A * a.I * R_A.transpose();
        const mathlib::Mat3 I_B = R_B * b.I * R_B.transpose();
        const mathlib::Mat3 I_Ainv = I_A.inverse();
        const mathlib::Mat3 I_Binv = I_B.inverse();
        // Compute world-space inertia tensors and relative velocities at the contact point
        mathlib::Vec3 vLin_A = a.linVel, w_A = a.angVel; 
        mathlib::Vec3 vLin_B = b.linVel, w_B = b.angVel;
        // Compute the contact points relative to the centers of mass of each body
        const mathlib::Vec3 r_A = p.pos - a.pos;
        const mathlib::Vec3 r_B = p.pos - b.pos;
        // Compute relative velocity at the contact point
        const mathlib::Vec3 v_Apt = vLin_A + w_A.cross(r_A);
        const mathlib::Vec3 v_Bpt = vLin_B + w_B.cross(r_B); 
        const mathlib::Vec3 v_rel = v_Bpt - v_Apt;
        // Compute the effective mass along the contact normal
        auto m_eff_norm = [&](BodyInfo& b, const mathlib::Vec3& r, const mathlib::Mat3& I_inv, const mathlib::Vec3& d) -> double {
            return (1.0/b.m) + d.dot(r.cross(I_inv * r.cross(d)));
        };
        double m_eff = m_eff_norm(a, r_A, I_Ainv, norm) + m_eff_norm(b, r_B, I_Binv, norm); // Compute the effective mass for the contact
        double j = physlib::collision::solveNormalImpulse(norm, v_rel , m_eff, e); // Compute the normal impulse magnitude
        if (!std::isfinite(j) || m_eff <= 1e-12) { return; } // Avoid division by zero or non-finite impulses
        mathlib::Vec3 J = j * norm; // Compute the impulse vector
        vLin_A -= J / a.m; w_A -= I_Ainv * r_A.cross(J); // Update linear and angular velocities of body A based on the impulse
        vLin_B += J / b.m; w_B += I_Binv * r_B.cross(J); // Update linear and angular velocities of body B based on the impulse
        // Friction resolution
        mathlib::Vec3 v_T = v_rel - (v_rel.dot(norm) * norm); // Compute the relative velocity in the tangent plane
        if (v_T.norm() > 1e-6) { // If there is significant tangential relative velocity, apply friction
            mathlib::Vec3 t = v_T.normalized(); // Tangent direction
            double m_eff_t = m_eff_norm(a, r_A, I_Ainv, t) + m_eff_norm(b, r_B, I_Binv, t); // Effective mass along the tangent
            mathlib::Vec3 J_f = physlib::collision::solveFrictionImpulse(t, v_rel, m_eff_t, j, mu); // Compute the friction impulse
            vLin_A -= J_f / a.m; w_A -= I_Ainv * r_A.cross(J_f); // Update body A with friction impulse
            vLin_B += J_f / b.m; w_B += I_Binv * r_B.cross(J_f); // Update body B with friction impulse
        }
        if (!vLin_A.allFinite() || !w_A.allFinite() || !vLin_B.allFinite() || !w_B.allFinite()) {
            LOG_ERROR_ONCE("CollisionResolver::resolveContact_fb: Non-finite velocity detected after collision resolution.");
            return;
        }
        // Update the RigidBodySystems with the new velocities
        A.setVelocity_fb(vLin_A, w_A);
        B.setVelocity_fb(vLin_B, w_B);
    }

    // Resolves positional corrections for a contact manifold between two RigidBodySystems
    void CollisionResolver::positionalCorrection_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, physlib::collision::ContactManifold& m) {
        auto a = BodyInfo(A); auto b = BodyInfo(B);
        constexpr double slop = 0.005; // Allowable penetration depth before correction
        constexpr double beta = 0.4;  // Percentage of penetration to correct per frame
        // Compute the maximum penetration depth from the contact manifold points
        double depth = 0.0;
        for (int i = 0; i < m.pointCount; ++i) { depth = std::max(depth, m.points[i].depth); } // Accumulate penetration depth
        const double penetration = std::max(depth - slop, 0.0); // Compute the penetration depth to correct
        if (penetration <= 0.0) { return; } // No correction needed if penetration is within the slop
        // Compute the inverse masses of both bodies and the total inverse mass
        const double m_Ainv = 1.0 / a.m, m_Binv = 1.0 / b.m;
        const double m_invTotal = m_Ainv + m_Binv;
        if (m_invTotal <= 1e-12) { return; } // Avoid division by zero (singularities) if both bodies are immovable
        // Compute the positional correction vector and apply it to both bodies
        const mathlib::Vec3 push = (beta * penetration / m_invTotal) * m.normal; // Compute the positional correction vector
        A.setPosition_fb(a.pos - m_Ainv * push); // Move body A away from the contact
        B.setPosition_fb(b.pos + m_Binv * push); // Move body B away from the contact
    }

    // Builds a world-space capsule from a link's local collision shape.
    physlib::collision::Capsule CollisionResolver::makeCapsule(const systems::RigidBodyLink& link, const mathlib::Mat4& world_T) {
        if (link.collision.type != systems::eCollisionShape::CAPSULE) { LOG_ERROR("Link collision shape is not a capsule, cannot create capsule"); return physlib::collision::Capsule{}; }
        auto xform = [&](const mathlib::Vec3& p) -> mathlib::Vec3 {
            mathlib::Vec4 h(p.x(), p.y(), p.z(), 1.0); // Homogeneous coordinates
            mathlib::Vec4 h_w = world_T * h; // Transform to world coordinates
            return mathlib::Vec3(h_w.x(), h_w.y(), h_w.z());
        };
        physlib::collision::Capsule c;
        c.a = xform(link.collision.localA);
        c.b = xform(link.collision.localB);
        c.radius = link.collision.radius;
        return c;
    }

    // Resolves collisions between a set of RigidBodySystems and updates their states accordingly
    void CollisionResolver::resolveCollisions(std::vector<std::unique_ptr<systems::RigidBodySystem>>& bodies, double dt) {
		for (size_t a = 0; a < bodies.size(); ++a) {
			for (size_t b = a + 1; b < bodies.size(); ++b) {
				LOG_INFO("collide pair %zu-%zu", a, b);
				if (!bodies[a]->hasFreeJoint() || !bodies[b]->hasFreeJoint()) { LOG_INFO("Not free joint -> skipping"); continue; }
				auto obbA = makeOBB(*bodies[a]);
				auto obbB = makeOBB(*bodies[b]);
				if (!obbA || !obbB) { LOG_INFO("No OBB (AABB unpopulated?) -> skipping"); continue; }
				physlib::collision::ContactManifold m;
				if (!physlib::collision::SAT_OBB(*obbA, *obbB, m)) { continue; }
				for (int k = 0; k < m.pointCount; ++k) {
					resolveContact_fb(*bodies[a], *bodies[b], m.normal, m.points[k], 0.2, 0.5);
				}
				positionalCorrection_fb(*bodies[a], *bodies[b], m);
			}
		}
	}
} // namespace physics