#pragma once

//=============================================
//				File: Mesh.h
//=============================================
// Class responsible for loading 3D mesh files using the Assimp library.
// 
// Summary:
// ============================================
// 
// public:
// --------------------------------------------
// Mesh()
//      -> Default constructor for the Mesh class.
// bool load(const std::string& filepath)
//      -> Loads a mesh from the specified file path.
// void addVertex(const VertexHolder& vertex)
//      -> Adds a vertex to the mesh's vertex list.
// void addVertexIndex(unsigned int vertexIndx)
//      -> Adds a vertex index to the mesh's index list.
// void init()
//      -> Initializes the mesh for rendering.
// void createBuffers()
//      -> Creates the necessary GPU buffers for the mesh.
// void deleteBuffers()
//      -> Deletes the GPU buffers associated with the mesh.
// void bind()
//      -> Binds the mesh's GPU buffers for rendering.
// void unbind()
//      -> Unbinds the mesh's GPU buffers.
// void render()
//      -> Renders the mesh.
// void clean()
// 		-> Cleans up the mesh resources.
// void update(shaders::Shader* shader)
//		-> Updates the shader with the mesh's material properties.
// glm::mat4 localTransform
//      -> Local transformation matrix for the mesh.
// glm::vec3 _colour
//      -> Colour of the mesh.s
// float _metallic
//		-> Metallic property of the mesh material.
// bool _isStatic
// 		-> Indicates if the mesh is static (immovable).
// --------------------------------------------
// 
// private:
// --------------------------------------------
// std::unique_ptr<render::VertexIndexBuffer> _rndrBffrMngr
// 		-> Unique pointer to the mesh's vertex and index buffer manager.
// --------------------------------------------
// 
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

namespace elements {
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

		// Update
		void update(shaders::Shader* shader) {	// will use for specifying objects colour and texture
			shader->setVec3(_colour, "albedo");
			shader->setFlt1(_metallic, "metallic");
			shader->setFlt1(1.0f, "ao");
		}

		glm::mat4 localTransform = glm::mat4(1.0f);

		glm::vec3 _colour = { 0.0f, 0.0f, 1.0f };
		float _metallic = 0.1; // When rotating could be useful for seeing rotations with respect to a fixed light source.
		bool _isStatic = false; // true for floor or immovable meshes

	private:
		std::unique_ptr<render::VertexIndexBuffer> _rndrBffrMngr;
	};
}