/*
 * File: Physics/CollisionResolver.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"
#include "Physics/CollisionResolver.h"

#include <collision/OBB.h>
#include <collision/SAT.h>
#include <collision/contact.h>

#include "EngineLib/LogMacros.h"

namespace physics {
    // physlib::collision::OBB CollisionResolver::makeOBB(const std::unique_ptr<systems::RigidBodySystem>& body) {
    //     return physlib::collision::OBB::fromAABB(body->model().aabb, body->model().orientation, body->model().position);
    // }

    void CollisionResolver::resolveContact_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, mathlib::Vec3& norm, std::array<mathlib::Vec3, 4>& contactPoints, double mu) {
        /* Placeholder */
    }

    void CollisionResolver::positionalCorrection_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, physlib::collision::ContactManifold& m) {
        /* Placeholder */
    }

    void CollisionResolver::resolveCollisions(std::vector<std::unique_ptr<systems::RigidBodySystem>>& bodies, double dt) {
        // cube-cube collision resolution for now
        for (size_t a = 0; a < bodies.size(); ++a) {
            for (size_t b = a + 1; b < bodies.size(); ++b) {
                auto& A = bodies[a];
                auto& B = bodies[b];

                physlib::collision::OBB obb_A = makeOBB(A);
                physlib::collision::OBB obb_B = makeOBB(B);

                
            }
        }
    }
} // namespace physics