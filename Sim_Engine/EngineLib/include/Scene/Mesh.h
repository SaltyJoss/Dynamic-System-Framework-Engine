#pragma once
#pragma warning(disable : 4251)

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

#include "Rendering/RenderBase.h"
#include "Scene/VertexHolder.h"
#include "Scene/Element.h"

#include "Scene/Face.h"	
#include "Platform/Logger.h"

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
		std::string setName(const std::string& name) { return _name = name.c_str() + id; }

		float getMetallic() const { return _metallic; }
		void setMetallic(float m) { _metallic = m; }

		glm::vec3 getAlbedo() const { return _albedo; }
		void setAlbedo(const glm::vec3& a) { _albedo = a; }
		
		// Update
		const void update(shaders::Shader* shader) const {	// will use for specifying objects colour and texture
			shader->setVec3(_albedo, "albedo");
			shader->setFlt1(_metallic, "metallic");
			shader->setFlt1(1.0f, "ao");
		}

		glm::mat4 localTransform = glm::mat4(1.0f);

	private:
		std::unique_ptr<render::VertexIndexBuffer> _rndrBffrMngr;

		int id = 0;
		std::string  _name = "obj" + id;

		glm::vec3 _albedo = glm::vec3(7.0f, 0.0f, 0.2f);
		float _metallic = 0.1f;
		float _roughness = 0.5f;

	};
}