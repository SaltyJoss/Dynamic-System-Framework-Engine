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
     * Helper function to create an OBB from a RigidBodySystem's free body state and local AABB.
     * Returns std::nullopt if the RigidBodySystem does not have a free body or if the local AABB is not available.
     */
    static std::optional<physlib::collision::OBB> makeOBB(const std::unique_ptr<systems::RigidBodySystem>& body) {
        mathlib::Vec3 pos, linVel, angVel;
        mathlib::Quat orientation;
        if (!body->freeBodyState(pos, orientation, linVel, angVel)) { return std::nullopt; }
        mathlib::Vec3 aabbMin, aabbMax;
        if (!body->freeBodyLocalAABB(aabbMin, aabbMax)) { return std::nullopt; }
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
        /* Placeholder */
    }

    // Resolves positional corrections for a contact manifold between two RigidBodySystems
    void CollisionResolver::positionalCorrection_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, physlib::collision::ContactManifold& m) {
        /* Placeholder */
    }

    // Resolves collisions between a set of RigidBodySystems and updates their states accordingly
    void CollisionResolver::resolveCollisions(std::vector<std::unique_ptr<systems::RigidBodySystem>>& bodies, double dt) {
        // cube-cube collision resolution for now
        for (size_t a = 0; a < bodies.size(); ++a) {
            for (size_t b = a + 1; b < bodies.size(); ++b) {
                auto& A = bodies[a];
                auto& B = bodies[b];
                const double e = std::min(A->restitution_d(), B->restitution_d());
                const double mu = std::min(A->friction_d(), B->friction_d());
                auto obb_A = makeOBB(A);
                auto obb_B = makeOBB(B);
                if (!obb_A || !obb_B) { continue; } // Skip if either body does not have a valid OBB
                physlib::collision::ContactManifold m;
                if (!physlib::collision::SAT_OBB(*obb_A, *obb_B, m)) { continue; } // No collision detected
                // Collision detected, resolve contact
                for (int i = 0; i < m.pointCount; ++i) {
                    resolveContact_fb(*A, *B, m.normal, m.points[i], e, mu); // Assuming a friction coefficient of 0.5 for now
                }
                positionalCorrection_fb(*A, *B, m);
            }
        }
    }
} // namespace physics