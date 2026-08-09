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
            /*
             * Internal body info struct to store relevant physical states
             */
            struct BodyInfo {
                systems::RigidBodySystem& body;
                double m;
                mathlib::Mat3 I;
                mathlib::Vec3 pos;
                mathlib::Quat ori;
                mathlib::Vec3 linVel;
                mathlib::Vec3 angVel;
                // Constructor to initialise BodyInfo from a RigidBodySystem pointer
                struct BodyInfo(systems::RigidBodySystem& b) : body(b) {
                    m = b.mass();
                    I = b.inertia_fb();
                    pos = b.position_fb();
                    ori = b.orientation_fb();
                    linVel = b.linearVelocity_fb();
                    angVel = b.angularVelocity_fb();
                }
            };

            /*
             * Internal helper functions for collision resolution
             */
            void resolveContact_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, mathlib::Vec3& norm, physlib::collision::ContactPoint& p, double e, double mu);
            void positionalCorrection_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, physlib::collision::ContactManifold& m);
    };
} // namespace physics