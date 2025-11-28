#pragma once

// =============================================
//            File: VertexHolder.h
// =============================================
// Class representing a vertex with position, normal, and texture coordinates.
//
// Summary:
// =============================================
// public:
// --------------------------------------------
// VertexHolder()
//      -> Default constructor initializing position, normal, and texture coordinates to zero.
// VertexHolder(const std::vector<std::string> tokens)
//      -> Constructor that initializes the vertex from a vector of string tokens.
// VertexHolder(const glm::vec3& pos, const glm::vec3& normal, const glm::vec2& texCoord = glm::vec2(0.0f))
//      -> Constructor that initializes the vertex with given position, normal, and optional texture coordinates.
// ~VertexHolder()
//      -> Default destructor.
// --------------------------------------------
//
// Built upon code from:
// ============================================
//	 GitHub: jayanam/jgl_demos/JGL_MeshLoader
// ============================================
// 
// ============================================
//              GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace scene {
	class ENGINE_API VertexHolder {
	public:
		VertexHolder() : _pos(), _normal(), _texCoord() {}

		VertexHolder(const std::vector<std::string> tokens) {}

		VertexHolder(const glm::vec3& pos, 
					 const glm::vec3& normal,
					 const glm::vec2& texCoord = glm::vec2(0.0f))
			: _pos(pos), _normal(normal), _texCoord(texCoord) {}

		~VertexHolder() = default;

		glm::vec3 _pos;
		glm::vec3 _normal;
		glm::vec2 _texCoord;
	};
}
