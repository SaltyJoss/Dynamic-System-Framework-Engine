// DSFE_GUI VertexHolder.h
#pragma once
// -----
// Initially templated off a tutorial:
// GitHub: jayanam/jgl_demos/JGL_MeshLoader
#pragma once

#include <glm/glm.hpp>
#include <string>

namespace assets {
	class VertexHolder {
	public:
		VertexHolder() : _pos(), _normal(), _texCoord() {}

		VertexHolder(const std::vector<std::string>& /*tokens*/) {}

		VertexHolder(const glm::vec3& pos, 
					 const glm::vec3& normal,
					 const glm::vec2& texCoord)
			: _pos(pos), _normal(normal), _texCoord(texCoord) {}

		~VertexHolder() = default;

		glm::vec3 _pos{ 0.0f };
		glm::vec3 _normal{ 0.0f };
		glm::vec2 _texCoord{ 0.0f };
	};
} // namespace scene
