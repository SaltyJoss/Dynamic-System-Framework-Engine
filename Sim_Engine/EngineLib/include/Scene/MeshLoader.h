#pragma once

#include "EngineCore.h"

#include <assimp/scene.h>
#include <vector>
#include <memory>
#include <string>

namespace ai {
	class Node;
	class Scene;
	class Mesh;
}

namespace elements {
	class Mesh;
}

namespace gui {
	class ENGINE_API MeshLoader {
	public:
		std::vector<std::shared_ptr<elements::Mesh>> load(const std::string& filepath);

	private:
		std::vector<std::shared_ptr<elements::Mesh>> _imported;
		std::shared_ptr<elements::Mesh> processMesh(aiMesh* mesh);

		void processNode(aiNode* node, const aiScene* scene, std::vector<std::shared_ptr<elements::Mesh>>& out);
	};
}