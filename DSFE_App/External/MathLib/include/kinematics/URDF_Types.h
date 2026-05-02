#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

using namespace mathlib;

namespace kinematics {
	// URDF Joint Types
	enum class JointType_URDF {
		REVOLUTE,
		CONTINUOUS,
		PRISMATIC,
		FIXED,
		FLOATING,
		PLANAR
	};
	// URDF Link Collision Shape Types
	enum class CollisionShapeType_URDF {
		BOX,
		CYLINDER,
		SPHERE,
		MESH
	};
	// URDF Joint Structure
	struct JointURDF {
		Vec3 origin_xyz;		// joint origin translation
		Mat3 origin_R;			// joint origin rotation
		Vec3 axis;				// joint axis
		JointType_URDF type;    // Revolute / Prismatic / ...	
		bool axixInJointFrame;  // true for "axis_frame":"joint"
	};

}