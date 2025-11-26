#pragma once


//=============================================
//            File: PhysicsState.h
//=============================================
// Struct encapsulating the physical state of an object in a physics simulation.
// 
// Summary:
// ============================================
// 
// Structs:
// --------------------------------------------
// PhysicsState
//    -> Represents the physical state of an object, including position, velocity, mass, inertia, and forces.
// --------------------------------------------
// 
// Internal State Variables:
// --------------------------------------------
// Eigen::Vector3d theta
//      -> Orientation angle around a fixed axis (e.g., for 2D rotation).
// Eigen::Vector3d linearVelocity
//      -> Velocity of the object's center of mass.
// Eigen::Vector3d angularVelocity
//		-> Angular velocity vector for rotational motion.
// double gravity
// 		-> Gravitational acceleration (in m/s^2).
// double mass
//      -> Mass of the object (in kg).
// Eigen::Matrix3d inertia
//      -> Inertia tensor for rotational dynamics.
// Eigen::Vector3d forces
//      -> Accumulated forces acting on the object.
// Eigen::Vector3d position
//      -> Position of the object in world space.
// Eigen::Vector3d torques
//      -> Accumulated torques acting on the object
// --------------------------------------------
// 
// ============================================

#include "EngineCore.h"
#include "PhysicsSystem.h"

namespace physics {
	// public struct for physical states
	struct ENGINE_API PhysicsState {
		Eigen::Vector3d theta;
		Eigen::Vector3d linearVelocity;
		Eigen::Vector3d angularVelocity;

		double gravity;
		double mass;
		Eigen::Matrix3d inertia;
		Eigen::Vector3d forces;
		Eigen::Vector3d position;
		Eigen::Vector3d torques;
	};
}

/*
* NOTES:
* ============
* (19-11-2025)
* ============
* This struct encapsulates the physical state of an object in a physics simulation.
* It includes properties for both translational and rotational dynamics (see base definitions above).
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
* 
* ============
* (26-11-2025)
* ============
* Realising that I need to link each robotic part to its parent, to properly simulate jointed motion.
* May need to add parent-child relationships in the Object class, or manage it in the PhysicsSystem.
* Already have Transform struct for position/rotation/scale, could extend that.
* I have to think about how forces/torques propagate through the hierarchy, should research "articulated body dynamics".
* 
* Some logic notes surrounding this:
* - Each part needs to know its parent to compute relative transforms.
* - Forces on child parts affect parent parts (e.g., torque from a swinging arm affects the torso).
* - Joint constraints need to be enforced between connected parts (e.g., hinge joints, ball-and-socket joints).
* - May need to compute global transforms from local transforms recursively up the hierarchy.
* - Could use a scene graph structure to manage the hierarchy of objects.
* - Will need to update the PhysicsSystem to handle joint constraints and hierarchical force propagation.
* - This will allow for more realistic simulation of robots and other articulated mechanisms.
* - Need to consider performance implications of hierarchical updates in the physics loop.
* - In current physical state (zero-g), gravity is not applied, meaning the links float freely unless constrained by joints or external forces.
*		-> This also means that any applied forces or torques will directly influence the motion of the links without gravitational effects, potentially leading to more pronounced movements???
*		-> The exclusion of gravity purposely allows for testing pure joint dynamics without gravitational interference, but once gravity is introduced, it will add a downward force on each link proportional to its mass, affecting stability and control.
* - PID controllers will be essential for managing the positions and orientations of each robotic link, especially in a zero-gravity environment where traditional weight-based stabilization is absent (GOOD POINT FOR MY PROJECTS OUTCOMES -> REFER TO IN WIP).
* - Each link's motion is determined by its own state and the constraints imposed by its joints to parent links.
* 
* Design considerations for sorting this link hierarchy:
* - Use a tree structure where each node represents a link, and edges represent joints (DONE in control panel for UI).
* - Perform a depth-first traversal to update transforms from parent to child -> ENSURES parents are processed before children (IMPORTANT).
* - Store parent indices in each link for quick access during updates.
* - Consider using a topological sort if there are complex dependencies between links -> ENSURES no cycles and correct processing order.
* - Optimize updates by only recalculating transforms for links that have changed.
* - Ensure that joint constraints are applied after updating transforms to maintain physical accuracy.
* - Test with simple robotic arms first to validate the hierarchy and joint dynamics before scaling up to more complex structures.
* - Consider performance implications of hierarchical updates in the physics loop (DOCUMENT IT JOSS!!!!)
* - Look into existing physics engines for articulated body dynamics for inspiration and best practices.
* - Use existing 3rd Party ideas on hierarchical kinematics as reference -> e.g. ROS, Gazebo, Bullet Physics, MuJoCo, O.D.E., etc.
* - Find other resources on robotic kinematics and dynamics to inform implementation.
* 
* Code ideas (rough syntax):
* - Each link has a struct like:
*	->	struct RobotLink {
*			std::string name;
*			std::string meshFile;
*			elements::Object* attachedObject = nullptr;
*		};
* 
* - But maybe should instead have:
* 	->	struct RobotLink {
*			std::string name;
*			glm::mat4 localTransform; // Transform relative to parent
*			std::string meshFile;
*			elements::Object* attachedObject = nullptr;
* 	};
* 
* - No need for global transform here, that is stored in ModelGroup.h so that rendering can access it directly.
* 
* - During the physics update:
*	->	void updateLinkTransforms(std::vector<Link>& links) {
*			for (const auto& link : links) {
*				if (link.parentIndex >= 0) {
*					link.globalTransform = links[link.parentIndex].globalTransform * link.localTransform;
*				} else {
*					link.globalTransform = link.localTransform; // Root link
*				}
*			}
*		}
* 
* - This makes sure that each link's global transform is computed based on its parent's global transform.
* - Right now I am doing this in SceneView, but it may be better suited for PhysicsSystem later on.
* - The correct place may depend on whether the transforms are needed for rendering or physics calculations first.
* - I may need to refactor sceneview later to separate rendering from physics updates, and other seperations honestly
*		-> Keep code modular and maintainable as complexity increases.
*		-> Look at using design patterns like ECS (Entity-Component-System) for better organization.
*
* - Ideally sceneview should contain only rendering logic, while physicssystem handles all physics updates including hierarchical transforms.
* - World camera is a specific case that may need special handling in sceneview, but other object transforms should be managed by physics.
* - I will move the hierarchical transform update logic to PhysicsSystem in the next refactor (not the current one this commit is part of, thats control panel and code comments).
* - This separation will make it easier to manage and extend the codebase as new features are added.
* 
* Also need to ensure that the user can move the ENTIRE robot if needed, not just individual links.
* - This could be done by applying a global transform to the root link.
* - Need to consider how this interacts with individual link transforms and joint constraints.
* - May need to implement a method to set the base transform of the robot in PhysicsSystem.
* - This will allow for easy repositioning of the entire robot in the scene, while still respecting the joint hierarchy.
* - Could add a method like:
* 	->	void setRobotBaseTransform(const glm::mat4& baseTransform) {
* 			inks[0].globalTransform = baseTransform; // Assuming links[0] is the root link
* 	}
* 
* - This method would be called before updating link transforms to ensure the entire robot is positioned correctly.
* - (FOR THE Z1-ONLY ATM)	-> Need to make sure that the user themself cannot rotate the base along every axis, instead on along the y-axis only (UPRIGHT ROBOT).
*							-> In fact the following Z1 heirarchy should be enforced:
* 								- Base (static)
* 								- Link01 / base link (rotates around y-axis only)
* 								- Link02 (rotates around z-axis only) -> its y-axis depends on Link01 rotation
* 								- Link03 (rotates around z-axis only) -> its y-axis depends on Link01->Link02 rotation
* 								- Link04 (rotates around z-axis only) -> its y-axis depends on Link01->Link02->Link03 rotation
*								- Link05 (rotates around z-axis only) -> its y-axis depends on Link01->Link02->Link03->Link04 rotation
*								- Link06 (rotates around z-axis only) -> its y-axis depends on Link01->Link02->Link03->Link04->Link05 rotation
* 								- End Effector (no rotation, just position) ~ not implemented yet
* - Not too sure how exactly to enforce this yet, may need to research joint constraints more.
* - Could potentially hardcode these constraints in the joint update logic for now, as this is a Z1-specific feature for testing.
* - Will need to generalize later for other robot types.
* - Could use enum types for joint types (e.g., revolute, prismatic) to handle different constraints.
* - Could edit JSON robot file to include joint type information for more flexibility
*		-> Already have "links" with "name" and "mesh"
*		-> Already have "joints" with "name", "parent", "child", "axis", "offset", "quat"
*		-> Could add "type": "revolute" or "prismatic" to each joint definition
*		-> For connection each links trasnform to its parent link, could add "transform": { "position": [...], "rotation": [...] } to each link definition
*			-> This would define the local transform relative to the parent link
*			-> I need to find a way for each rotation of the parent to directly affect the child link's local rotation axis, best way to do this is to use quaternions probably, but need to research more on that
* - Need to document these design decisions and constraints clearly in the code for future reference.
*/