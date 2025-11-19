
#pragma once

#include "EngineCore.h"
#include "PhysicsSystem.h"

namespace physics {
	// public struct for physical states
	struct ENGINE_API PhysicsState {
		double theta;						// Orientation angle around a fixed axis (e.g., for 2D rotation)
		Eigen::Vector3d linearVelocity;		// Velocity of the object's center of mass
		Eigen::Vector3d angularVelocity;	// Angular velocity vector

		double gravity;				// Gravitational acceleration (in m/s^2)
		double mass;				// Mass of the object (in kg)
		Eigen::Matrix3d inertia;	// Inertia tensor for rotational dynamics
		Eigen::Vector3d forces;		// Accumulated forces acting on the object
		Eigen::Vector3d position;	// Position of the object in world space
		Eigen::Vector3d torques;	// Accumulated torques acting on the object
	};
}



/*
* NOTES:
* This struct encapsulates the physical state of an object in a physics simulation.
* It includes properties for both translational and rotational dynamics, such as:
* - theta: Orientation angle around a fixed axis (useful for 2D rotation).
* - linearVelocity: Velocity of the object's center of mass.
* - angularVelocity: Angular velocity vector for rotational motion.
* - mass: Mass of the object, essential for calculating acceleration from forces.
* - inertia: Inertia tensor, which defines how the object resists rotational acceleration.
* - forces: Accumulated forces acting on the object, used to compute linear acceleration.
* - position: Position of the object in world space.
* - torques: Accumulated torques acting on the object, used to compute angular acceleration.
* 
* Some logic notes:
* - torque = moment of inertia * angular acceleration
* - torque = d(angular momentum)/dt
* - force = mass * acceleration
* - angular momentum = moment of inertia * angular velocity
* - acceleration = d(velocity)/dt
* - angular acceleration = d(angular velocity)/dt
* - momentum = mass * velocity
* - delta velocity = J/mass
* - J = impulse applied
* - delta angular velocity = I^-1 * L
* - L = angular impulse applied
* 
* - TWO BODY COLLISION -> 
*		- relative velocity Vr = V1 - V2
*		- V1' = V1 + (J / m1) * n	--> Heavier bodies / objects change velocity less
*		- V2' = V2 - (J / m2) * n	--> Lighter bodies / objects change velocity more (Bounce more)
*		- impulse J = -(1 + e)(Vr . n) / (1/m1 + 1/m2)
* 
* - How mass affects rotation:
*		- Angular momentum is shown above as L = I * w
* 		- Torque (tau) is this instance is tau = I * alpha
*		- Where alpha is angular acceleration alpha = I^-1 * tau
* 		- Moment of inertia (I) depends on mass distribution relative to the axis of rotation.
* 		- Higher mass increases moment of inertia, making it harder to change rotational speed.
*		- Lower mass decreases moment of inertia, making it easier to change rotational speed.
*/