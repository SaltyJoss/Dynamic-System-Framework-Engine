#pragma once
// File:   Element.h
// GitHub: SaltyJoss
#include "EngineCore.h"

#include "Rendering/ShaderUtil.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace scene {
	class ENGINE_API Element {
	public:
		virtual ~Element() = default;

		virtual void update(shaders::Shader*) {}
	};
} // namespace scene