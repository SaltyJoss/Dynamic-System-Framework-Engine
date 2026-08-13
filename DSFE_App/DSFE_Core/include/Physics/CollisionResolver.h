/*
 * File: Physics/CollisionResolver.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once
#include "EngineCore.h"
#include <MathLib>
#include "Systems/RigidBodySystem.h"
#include <collision/capsule.h>
#include <collision/contact.h>

namespace physics {
    class DSFE_API CollisionResolver {
        public:
            CollisionResolver();
            ~CollisionResolver();
            // Resolves a collision between two OBBs and returns the contact information
            void resolveCollisions(std::vector<std::unique_ptr<systems::RigidBodySystem>>& bodies, double dt);
            physlib::collision::Capsule makeCapsule(const systems::RigidBodyLink& link, const mathlib::Mat4& world_T);

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
                BodyInfo(systems::RigidBodySystem& b) : body(b) {
                    m = b.mass();
                    I = b.inertia_fb();
                    pos = b.position_fb();
                    ori = b.orientation_fb();
                    linVel = b.linearVelocity_fb();
                    angVel = b.angularVelocity_fb();
                }
            };

            /*
             * Internal methods for collision resolution and positional correction
             */
            // Resolves collisions between a set of RigidBodySystems and updates their states accordingly
            void resolveContact_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, mathlib::Vec3& norm, physlib::collision::ContactPoint& p, double e, double mu);
            // Resolves positional corrections for a contact manifold between two RigidBodySystems
            void positionalCorrection_fb(systems::RigidBodySystem& A, systems::RigidBodySystem& B, physlib::collision::ContactManifold& m);
            // Resolves a collision between a free body and a fixed body and updates the free body's velocity based on the contact information
            void resolveContact_fbVsFixed(systems::RigidBodySystem& fb, mathlib::Vec3& norm, physlib::collision::ContactPoint& p, double e, double mu);
            // Resolves positional corrections for a contact manifold between a free body and a fixed body
	        void positionalCorrection_fbVsFixed(systems::RigidBodySystem& fb, physlib::collision::ContactManifold& m);
            // Wrapper function to handle collisions between a free body and an arm (fixed body) and update the free body's state accordingly
            void collideFreeVsFixed(systems::RigidBodySystem& free, systems::RigidBodySystem& fixed);
    };
} // namespace physics