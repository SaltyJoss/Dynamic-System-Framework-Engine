// PxM/MathLib SpatialVec.inl
#pragma once

namespace mathlib {
	template<typename Scalar>
	template<typename ScalarT>
	SpatialVec_T<ScalarT> SpatialVec_T<Scalar>::cast() const {
		SpatialVec_T<ScalarT> out;
		out.v = this->v.template cast<ScalarT>();
		return out;
	}
} // namespace mathlib