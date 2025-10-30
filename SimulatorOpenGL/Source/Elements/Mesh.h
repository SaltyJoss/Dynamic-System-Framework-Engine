#pragma once

#include "ch.h"

#include "Render/RenderBase.h"
#include "VertexHolder.h"
#include "Element.h"

namespace elements {
	class Mesh : public Element {
		Mesh() = default;
		
		virtual ~Mesh();

		bool load(const std::string& path);
		void addVertex(const VertexHolder& vertex) { _vertices.push_back(vertex); }
		void addVertexIndex(unsigned int vertexIndx) { _vertexIndices.push_back(vertexIndx); }
		
		void init();
		void createBuffers();
		void deleteBuffers();
		void render();
		void bind();
		void unbind();

		std::vector<unsigned int> getVertexIndicies() { return _vertexIndices; }

		void update(shaders::Shader* shader) override {	// will used for specifying objects colour and texture

		}

		glm::vec3 _colour = { 0.0f, 0.0f, 1.0f }; // red
		float _metallic = 0.1; // When rotating could be useful for seeing rotations with respect to a fixed light source.

	private:
		std::unique_ptr<render::VertexIndexBuffer> _rndrBffrMngr;
		
		std::vector<VertexHolder> _vertices;
		std::vector<unsigned int> _vertexIndices;
	};
}