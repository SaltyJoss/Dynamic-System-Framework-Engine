// DSFE_GUI MeshPresentationBuilder.h
#pragma once


#include <vector>
#include <memory>

namespace scene {
	class Mesh;
	class Object;
}

namespace presentation {
	struct MeshRenderBinding {
		std::vector<std::unique_ptr<scene::Object>> owndObjs;
		std::vector<scene::Object*> visObjs;
	};

	class MeshPresentationBuilder {
	public:
		MeshRenderBinding build(const std::vector<std::shared_ptr<scene::Mesh>>& meshes);
	};
} // namespace presentation