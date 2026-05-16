// PxM/MathLib Types_tpl.h
#pragma once

#include "MathLibAPI.h"
#include <Eigen/Dense>

namespace mathlib {
	// Template version of 3D vector
	template<typename Scalar>
	using Vec3_T = Eigen::Matrix<Scalar, 3, 1>;

	// Template version of 3x3 matrix
	template<typename Scalar>
	using Mat3_T = Eigen::Matrix<Scalar, 3, 3>;

	// Template version of 6D vector and 6x6 matrix for spatial algebra
	template<typename Scalar>
	using Vec6_T = Eigen::Matrix<Scalar, 6, 1>;

	// Template version of 6x6 matrix for spatial algebra
	template<typename Scalar>
	using Mat6_T = Eigen::Matrix<Scalar, 6, 6>;

	// Template version of dynamic-size vector and matrix
	template<typename Scalar>
	using VecX_T = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

	// Template version of dynamic-size matrix
	template<typename Scalar>
	using MatX_T = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
}