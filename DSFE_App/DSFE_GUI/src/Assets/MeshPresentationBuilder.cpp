// DSFE_GUI MeshPresentationBuilder.cpp
#include "Assets/MeshPresentationBuilder.h"
#include "Scene/Object.h"

namespace presentation {
	MeshRenderBinding MeshPresentationBuilder::build(const std::vector<std::shared_ptr<scene::Mesh>>& meshes) {
		MeshRenderBinding binding;
		for (const auto& mesh : meshes) {
			auto obj = std::make_unique<scene::Object>(mesh);
			obj->category = scene::ObjectCategory::General;
			binding.visObjs.push_back(obj.get());
			binding.owndObjs.push_back(std::move(obj));
		}
		return binding;
	}
} // namespace presentation