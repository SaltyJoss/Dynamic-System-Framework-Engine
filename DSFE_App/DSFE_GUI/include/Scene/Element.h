// DSFE_GUI Element.h
#pragma once

#include "Rendering/ShaderUtil.h"
#include "Platform/Logger.h"

namespace scene {
	class Element {
	public:
		virtual ~Element() = default;

		virtual void update(shaders::Shader*) {}
	};
} // namespace scene