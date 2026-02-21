#pragma once
// File:   VertexHolder.h
// GitHub: SaltyJoss
// -----
// Initially templated off a tutorial:
// GitHub: jayanam/jgl_demos/JGL_MeshLoader
#include "EngineCore.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace assets {
	class ENGINE_API VertexHolder {
	public:
		VertexHolder() : _pos(), _normal(), _texCoord() {}

		VertexHolder(const std::vector<std::string>& /*tokens*/) {}

		VertexHolder(const glm::vec3& pos, 
					 const glm::vec3& normal,
					 const glm::vec2& texCoord = glm::vec2(0.0f))
			: _pos(pos), _normal(normal), _texCoord(texCoord) {}

		~VertexHolder() = default;

		glm::vec3 _pos = glm::vec3(0.0f);
		glm::vec3 _normal = glm::vec3(0.0f);
		glm::vec2 _texCoord = glm::vec2(0.0f);
	};
} // namespace scene
