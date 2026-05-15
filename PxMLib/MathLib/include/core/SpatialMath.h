// PxMLib SpatialMath.h
#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

namespace mathlib {

	using SpatialMat = Eigen::Matrix<double, 6, 6>;

	struct SpatialVec {
		Eigen::Matrix<double, 6, 1> v;

		SpatialVec() {
			v.setZero();
		}

		SpatialVec(const Vec3& angular, const Vec3& linear) {
			v.segment<3>(0) = angular;
			v.segment<3>(3) = linear;
		}

		Vec3 angular() const { return v.segment<3>(0); }
		Vec3 linear() const { return v.segment<3>(3); }

		SpatialVec operator+(const SpatialVec& rhs) const {
			SpatialVec out;
			out.v = this->v + rhs.v;
			return out;
		}

		SpatialVec operator-(const SpatialVec& rhs) const {
			SpatialVec out;
			out.v = this->v - rhs.v;
			return out;
		}

		SpatialVec operator*(double rhs) const {
			SpatialVec out;
			out.v = this->v * rhs;
			return out;
		}

		SpatialVec& operator+=(const SpatialVec& rhs) {
			this->v += rhs.v;
			return *this;
		}

		SpatialVec operator*=(double rhs) {
			this->v *= rhs;
			return *this;
		}

		double dot(const SpatialVec& sv) const {
			return this->v.dot(sv.v);
		}
	};

	inline Mat3 skewSymmetric(const Vec3& v) {
		Mat3 S;
		S <<
			0, -v.z(), v.y(),
			v.z(), 0, -v.x(),
			-v.y(), v.x(), 0;

		return S;
	}

	inline SpatialMat motionCrossMatrix(const SpatialVec& sv) {
		SpatialMat X = SpatialMat::Zero();

		Vec3 w = sv.angular();
		Vec3 v = sv.linear();

		X.block<3, 3>(0, 0) = skewSymmetric(w);
		X.block<3, 3>(3, 0) = skewSymmetric(v);
		X.block<3, 3>(3, 3) = skewSymmetric(w);

		return X;
	}

	inline SpatialMat forceCrossMatrix(const SpatialVec& sv) {
		return (- motionCrossMatrix(sv).transpose()).eval();
	}

	inline SpatialMat spatialTransform(const Mat3& R, const Vec3& r) {
		SpatialMat X = SpatialMat::Zero();

		X.block<3, 3>(0, 0) = R;
		X.block<3, 3>(3, 0) = -R * skewSymmetric(r);
		X.block<3, 3>(3, 3) = R;

		return X;
	}

	inline SpatialMat spatialInertia(const double mass, const Vec3& com, const Mat3& I_com) {
		SpatialMat I = SpatialMat::Zero();
		
		Mat3 c_skew = skewSymmetric(com);

		I.block<3, 3>(0, 0) = I_com + mass * c_skew.transpose() * c_skew;
		I.block<3, 3>(0, 3) = mass * c_skew.transpose();
		I.block<3, 3>(3, 0) = mass * c_skew;
		I.block<3, 3>(3, 3) = mass * Mat3::Identity();

		return I;
	}

	// Operator overloads for spatial vector and matrix operations

	// SpatialMat * SpatialVec
	inline SpatialVec operator*(
		const SpatialMat& lhs,
		const SpatialVec& rhs
		) {
		SpatialVec out;
		out.v = lhs * rhs.v;
		return out;
	}

	// SpatialVec1 * SpatialVec2 (outer product)
	inline SpatialMat outer(
		const SpatialVec& rhs,
		const SpatialVec& lhs
	) {
		return rhs.v * lhs.v.transpose();
	}
	// SpatialVec^2 (outer product)
	inline SpatialMat outer(const SpatialVec& sv) {
		return sv.v * sv.v.transpose();
	}

	// SpatialVec1 . SpatialVec2
	inline double dot(
		const SpatialVec& lhs,
		const SpatialVec& rhs
	) {
		return lhs.v.dot(rhs.v);
	}
	// SpatialVec .^2 (element-wise square)
	inline double dot(const SpatialVec& sv) {
		return sv.v.dot(sv.v);
	}

	// SpatialVec x SpatialVec (motion cross product)
	inline SpatialVec crossMotion(
		const SpatialVec& lhs,
		const SpatialVec& rhs
	) {
		SpatialVec out;
		out.v = motionCrossMatrix(lhs) * rhs.v;
		return out;
	}

	// SpatialVec x SpatialVec (force cross product)
	inline SpatialVec crossForce(
		const SpatialVec& lhs,
		const SpatialVec& rhs
	) {
		SpatialVec out;
		out.v = forceCrossMatrix(lhs) * rhs.v;
		return out;
	}
} // namespace mathlib