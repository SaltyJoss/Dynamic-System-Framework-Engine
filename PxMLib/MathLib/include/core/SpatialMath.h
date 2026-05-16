// PxMLib SpatialMath.h
#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "core/Types_tpl.h"

namespace mathlib {
	using Vec3 = Vec3_T<double>;
	using Mat3 = Mat3_T<double>;

	// Template version of spatial vector
	template<typename Scalar>
	struct SpatialVec_T {
		using ScalarT = Scalar;
		using Vec3S = Vec3_T<Scalar>;
		using Vec6S = Vec6_T<Scalar>;

		using AngularBlock = decltype(std::declval<Vec6S&>().template segment<3>(0));
		using ConstAngularBlock = decltype(std::declval<const Vec6S&>().template segment<3>(0));

		using LinearBlock = decltype(std::declval<Vec6S&>().template segment<3>(3));
		using ConstLinearBlock = decltype(std::declval<const Vec6S&>().template segment<3>(3));

		Vec6S v;
		
		SpatialVec_T() { v.setZero(); }

		SpatialVec_T(const Vec3S& angular, const Vec3S& linear) {
			v.template segment<3>(0) = angular;
			v.template segment<3>(3) = linear;
		}

		AngularBlock angular() { return v.template segment<3>(0); }
		ConstAngularBlock angular() const { return v.template segment<3>(0); }

		LinearBlock linear() { return v.template segment<3>(3); }
		ConstLinearBlock linear() const { return v.template segment<3>(3); }

		SpatialVec_T operator+(const SpatialVec_T& rhs) const {
			SpatialVec_T out;
			out.v = this->v + rhs.v;
			return out;
		}

		SpatialVec_T operator-(const SpatialVec_T& rhs) const {
			SpatialVec_T out;
			out.v = this->v - rhs.v;
			return out;
		}

		SpatialVec_T operator*(Scalar rhs) const {
			SpatialVec_T out;
			out.v = this->v * rhs;
			return out;
		}

		SpatialVec_T& operator+=(const SpatialVec_T& rhs) {
			this->v += rhs.v;
			return *this;
		}

		SpatialVec_T& operator*=(Scalar rhs) {
			this->v *= rhs;
			return *this;
		}

		Scalar dot(const SpatialVec_T& sv) const {
			return this->v.dot(sv.v);
		}
	};
	using SpatialVec = SpatialVec_T<double>;

	// Template version of spatial matrix
	template<typename Scalar>
	using SpatialMat_T = Mat6_T<Scalar>;
	using SpatialMat = SpatialMat_T<double>;

	template<typename Scalar>
	inline Mat3_T<Scalar> skewSymmetric(const Vec3_T<Scalar>& v) {
		Mat3_T<Scalar> S;
		S <<
			Scalar(0), -v.z(), v.y(),
			v.z(), Scalar(0), -v.x(),
			-v.y(), v.x(), Scalar(0);

		return S;
	}

	template<typename Scalar>
	inline SpatialMat_T<Scalar> motionCrossMatrix(const SpatialVec_T<Scalar>& sv) {
		SpatialMat_T<Scalar> X = SpatialMat_T<Scalar>::Zero();

		Vec3_T<Scalar> w = sv.angular();
		Vec3_T<Scalar> v = sv.linear();

		X.template block<3, 3>(0, 0) = skewSymmetric(w);
		X.template block<3, 3>(3, 0) = skewSymmetric(v);
		X.template block<3, 3>(3, 3) = skewSymmetric(w);

		return X;
	}

	template<typename Scalar>
	inline SpatialMat_T<Scalar> forceCrossMatrix(const SpatialVec_T<Scalar>& sv) {
		return (- motionCrossMatrix(sv).transpose()).eval();
	}

	template<typename Scalar>
	inline SpatialMat_T<Scalar> spatialTransform(
		const Mat3_T<Scalar>& R,
		const Vec3_T<Scalar>& r
	) {
		SpatialMat_T<Scalar> X = SpatialMat_T<Scalar>::Zero();

		X.template block<3, 3>(0, 0) = R;
		X.template block<3, 3>(3, 0) = -R * skewSymmetric(r);
		X.template block<3, 3>(3, 3) = R;

		return X;
	}

	template<typename Scalar>
	inline SpatialMat_T<Scalar> spatialInertia(
		const Scalar mass,
		const Vec3_T<Scalar>& com,
		const Mat3_T<Scalar>& I_com
	) {
		SpatialMat_T<Scalar> I = SpatialMat_T<Scalar>::Zero();
		
		Mat3_T<Scalar> c_skew = skewSymmetric(com);

		I.template block<3, 3>(0, 0) = I_com + mass * c_skew.transpose() * c_skew;
		I.template block<3, 3>(0, 3) = mass * c_skew.transpose();
		I.template block<3, 3>(3, 0) = mass * c_skew;
		I.template block<3, 3>(3, 3) = mass * Mat3_T<Scalar>::Identity();

		return I;
	}

	// Operator overloads for spatial vector and matrix operations

	// SpatialMat * SpatialVec
	template<typename Scalar>
	inline SpatialVec_T<Scalar> operator*(
		const SpatialMat_T<Scalar>& lhs,
		const SpatialVec_T<Scalar>& rhs
		) {
		SpatialVec_T<Scalar> out;
		out.v = lhs * rhs.v;
		return out;
	}

	template<typename Derived, typename Scalar>
	inline SpatialVec_T<Scalar> operator*(
		const Eigen::MatrixBase<Derived>& lhs,
		const SpatialVec_T<Scalar>& rhs
	) {
		SpatialVec_T<Scalar> out;
		out.v = lhs * rhs.v;
		return out;
	}

	// SpatialVec1 * SpatialVec2 (outer product)
	template<typename Scalar>
	inline SpatialMat_T<Scalar> outer(
		const SpatialVec_T<Scalar>& rhs,
		const SpatialVec_T<Scalar>& lhs
	) {
		return rhs.v * lhs.v.transpose();
	}

	// SpatialVec^2 (outer product)
	template<typename Scalar>
	inline SpatialMat_T<Scalar> outer(const SpatialVec_T<Scalar>& sv) {
		return sv.v * sv.v.transpose();
	}

	// SpatialVec1 . SpatialVec2
	template<typename Scalar>
	inline Scalar dot(
		const SpatialVec_T<Scalar>& lhs,
		const SpatialVec_T<Scalar>& rhs
	) {
		return lhs.v.dot(rhs.v);
	}

	// SpatialVec .^2 (element-wise square)
	template<typename Scalar>
	inline Scalar dot(const SpatialVec_T<Scalar>& sv) {
		return sv.v.dot(sv.v);
	}

	// SpatialVec x SpatialVec (motion cross product)
	template<typename Scalar>
	inline SpatialVec_T<Scalar> crossMotion(
		const SpatialVec_T<Scalar>& lhs,
		const SpatialVec_T<Scalar>& rhs
	) {
		SpatialVec_T<Scalar> out;
		out.v = motionCrossMatrix(lhs) * rhs.v;
		return out;
	}

	// SpatialVec x SpatialVec (force cross product)
	template<typename Scalar>
	inline SpatialVec_T<Scalar> crossForce(
		const SpatialVec_T<Scalar>& lhs,
		const SpatialVec_T<Scalar>& rhs
	) {
		SpatialVec_T<Scalar> out;
		out.v = forceCrossMatrix(lhs) * rhs.v;
		return out;
	}
} // namespace mathlib