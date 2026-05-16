// PxM/MathLib Types_tpl.h
#pragma once

#include <Eigen/Dense>

template<typename Scalar>
using Vec3_T = Eigen::Matrix<Scalar, 3, 1>;

template<typename Scalar>
using Mat3_T = Eigen::Matrix<Scalar, 3, 3>;

template<typename Scalar>
using Vec6_T = Eigen::Matrix<Scalar, 6, 1>;

template<typename Scalar>
using Mat6_T = Eigen::Matrix<Scalar, 6, 6>;

template<typename Scalar>
using VecX_T = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

template<typename Scalar>
using MatX_T = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;