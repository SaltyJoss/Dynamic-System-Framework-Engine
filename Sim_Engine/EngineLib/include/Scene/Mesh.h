#pragma once

//=============================================
//				File: Mesh.h
//=============================================
// Class responsible for loading 3D mesh files using the Assimp library.
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
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "Rendering/RenderBase.h"
#include "Scene/VertexHolder.h"
#include "Scene/Element.h"

#include "Scene/Face.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace render {
	class VertexIndexBuffer;
}

namespace scene {
	class ENGINE_API Mesh {
	public:
		Mesh() = default;
		
		//load
		bool load(const std::string& filepath);

		// CPU buffers
		std::vector<VertexHolder> _vertices;
		std::vector<unsigned int> _indices;

		void addVertex(const VertexHolder& vertex) { _vertices.push_back(vertex); }
		void addVertexIndex(unsigned int vertexIndx) { _indices.push_back(vertexIndx); }

		// GPU
		void init();
		void createBuffers();
		void deleteBuffers();
		void bind();
		void unbind();
		void render();
		void clean();

		std::string getName() const { return _name; }

		// Update
		void update(shaders::Shader* shader) {	// will use for specifying objects colour and texture
			shader->setVec3(_colour, "albedo");
			shader->setFlt1(_metallic, "metallic");
			shader->setFlt1(1.0f, "ao");
		}

		glm::mat4 localTransform = glm::mat4(1.0f);

		glm::vec3 _colour = { 0.0f, 0.0f, 1.0f };
		float _metallic = 0.1f; // When rotating could be useful for seeing rotations with respect to a fixed light source.
		bool _isStatic = false; // true for floor or immovable meshes

	private:
		std::unique_ptr<render::VertexIndexBuffer> _rndrBffrMngr;

		std::string  _name = "";
	};
}