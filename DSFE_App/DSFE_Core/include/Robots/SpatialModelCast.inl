// DSFE_Core SpatialModelCast.inl
#pragma once

namespace robots {
	template<typename Scalar>
	template<typename ScalarT>
	SpatialModel<ScalarT> SpatialModel<Scalar>::cast() const {
		SpatialModel<ScalarT> out;
		out.joints.resize(joints.size());
		for (size_t i = 0; i < joints.size(); ++i) {
			const auto& j = joints[i];
			auto& out_j = out.joints[i];
			out_j.parent = j.parent;
			out_j.type = j.type;
			out_j.Xtree = j.Xtree.template cast<ScalarT>();
			out_j.inertia = j.inertia.template cast<ScalarT>();
			out_j.S = j.S.template cast<ScalarT>();
			out_j.name = j.name;
		}
		return out;
	}
}