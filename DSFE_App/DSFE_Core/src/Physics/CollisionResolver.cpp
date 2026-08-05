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
    void CollisionResolver::resolveContact_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, mathlib::Vec3& norm, std::array<mathlib::Vec3, 4>& contactPoints, double mu) {
        /* Placeholder */
    }

    void CollisionResolver::positionalCorrection_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, physlib::collision::ContactManifold& m) {
        /* Placeholder */
    }

    void CollisionResolver::resolveCollisions(std::vector<std::unique_ptr<systems::RigidBodySystem>>& bodies, double dt) {
        /* Placeholder */
    }
} // namespace physics