#pragma once
#pragma warning(disable : 4100)

// =============================================
//             File: Element.h
// =============================================
// Base class for scene scene that can be updated with shaders.
//
// Summary:
// =============================================
// 
// public:
// --------------------------------------------
// virtual ~Element()
//      -> Virtual destructor for the Element class.
// virtual void update(shaders::Shader* shader)
//      -> Virtual method to update the element with the given shader.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================


#include "EngineCore.h"

#include "Rendering/ShaderUtil.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace scene {
	class ENGINE_API Element {
	public:
		virtual ~Element() = default;

		virtual void update(shaders::Shader* shader) {}
	};
}