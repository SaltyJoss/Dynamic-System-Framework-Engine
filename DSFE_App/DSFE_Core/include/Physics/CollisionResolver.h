/*
 * File: Physics/CollisionResolver.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once
#include "EngineCore.h"
#include <MathLib>
#include "Systems/RigidBodySystem.h"
#include <collision/contact.h>

namespace physics {
    class DSFE_API CollisionResolver {
        public:
            CollisionResolver();
            ~CollisionResolver();
            // Resolves a collision between two OBBs and returns the contact information
            void resolveCollisions(std::vector<std::unique_ptr<systems::RigidBodySystem>>& bodies, double dt);

        private:
            void resolveContact_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, mathlib::Vec3& norm, std::array<mathlib::Vec3, 4>& contactPoints, double mu);
            void positionalCorrection_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, physlib::collision::ContactManifold& m);
    };
} // namespace physics