// DSFE_GUI Element.h
#pragma once
#include "Platform/Logger.h"

namespace scene {
	class Element {
	public:
		virtual ~Element() = default;

		virtual void update() {}
	};
} // namespace scene