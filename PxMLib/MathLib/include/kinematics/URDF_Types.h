#pragma once

#include <core/MathLib.h>

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
	// URDF Link Collision Shape Types (May remove now, but Ill leave until full refactor is done)
	enum class CollisionShapeType_URDF {
		BOX,
		CYLINDER,
		SPHERE,
		MESH
	};
	// URDF Joint Structure
	template<typename Scalar>
	struct JointURDF {
		Vec3_T<Scalar> origin_xyz; // joint origin translation
		Mat3_T<Scalar> origin_R;   // joint origin rotation
		Vec3_T<Scalar> axis;	   // joint axis
		JointType_URDF type;   // Revolute / Prismatic / ...
		bool axixInJointFrame; // true for "axis_frame":"joint"
	};

}