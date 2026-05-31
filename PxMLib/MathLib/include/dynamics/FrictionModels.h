// PxM/MathLib FrictionModels.h

#include <core/MathLib.h>

namespace dynamics {
	template<typename Scalar>
	inline Scalar computeKarnoppFriction(const Scalar qd, const Scalar tau_active, Scalar c, Scalar b) {
		Scalar Dv = Scalar(1e-4); // Small vel deadband
		if (mathlib::abs(qd) > Dv) { return c * mathlib::sgn(qd) + b * qd; } // Viscous friction
		else {
			Scalar max_static_friction = c * Scalar(1.2); // Max static friction (20 percent higher than dynamic friction)
			return mathlib::clamp(tau_active, -max_static_friction, max_static_friction); // Static friction with saturation
		}
	}
} // namespace dynamics