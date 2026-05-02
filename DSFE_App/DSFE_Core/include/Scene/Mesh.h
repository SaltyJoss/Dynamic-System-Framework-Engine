#pragma once
// File:   Mesh.h
// GitHub: SaltyJoss
// -----
// Initially templated off a tutorial:
// GitHub: jayanam/jgl_demos/JGL_MeshLoader
#include "EngineCore.h"

#include "Rendering/RenderBase.h"
#include "Assets/VertexHolder.h"
#include "Scene/Element.h"

#include "Scene/Face.h"	
#include "Platform/Logger.h"

// Forward Declarations for VertexIndexBuffer.h
namespace render { class DSFE_API VertexIndexBuffer; }

namespace scene {
	class DSFE_API Mesh {
	public:
		// Constructors & Destructor
		Mesh() = default;
		
		//load
		bool load(const std::string& filepath);

		// CPU buffers
		std::vector<assets::VertexHolder> _vertices;
		std::vector<unsigned int> _indices;

		// Utility methods for building mesh geometry
		void addVertex(const assets::VertexHolder& vertex) { _vertices.push_back(vertex); }
		void addVertexIndex(unsigned int vertexIndx) { _indices.push_back(vertexIndx); }

		// GPU buffer management
		void init();
		void createBuffers();
		void deleteBuffers();
		void bind();
		void unbind();
		void render();
		void clean();

		// Local Transform for this Mesh
		glm::mat4 localTransform = glm::mat4(1.0f);

		// Getters & Setters for Mesh Name
		std::string getName() const { return _name; }
		std::string setName(const std::string& name) { return _name = name.c_str() + id; }

		// Material Properties
		float getMetallic() const { return _metallic; }
		void setMetallic(float m) { _metallic = m; }
		float getRoughness() const { return _roughness; }
		void setRoughness(float r) { _roughness = r; }
		glm::vec3 getAlbedo() const { return _albedo; }
		void setAlbedo(const glm::vec3& a) { _albedo = a; }

		// Shader Update
		const void update(shaders::Shader* shader) const {
			shader->setVec3(_albedo, "albedo");
			shader->setFlt1(_metallic, "metallic");
			shader->setFlt1(_roughness, "roughness");
			shader->setFlt1(1.0f, "ao");
		}

		// Utility to append another mesh's geometry to this one, applying the other mesh's local transform to its vertices in the process
		void appendGeometry(const Mesh& other) {
			const uint32_t indexOffset = (uint32_t)_vertices.size();
			const glm::mat4 T = other.localTransform;

			// Transform and append vertices
			for (const auto& v : other._vertices) {
				assets::VertexHolder out = v;

				// Apply the local transform to the vertex position and normal
				glm::vec4 p = T * glm::vec4(v._pos, 1.0f);
				out._pos = glm::vec3(p);

				// Only transform the normal if it's non-zero
				if (glm::length(v._normal) > 0.0f) {
					glm::vec4 n = T * glm::vec4(v._normal, 0.0f);
					out._normal = glm::normalize(glm::vec3(n));
				}
				_vertices.push_back(out);
			}

			// Append indices with offset
			for (uint32_t idx : other._indices) {
				_indices.push_back(idx + indexOffset);
			}
		}

		// Rebuilds the GPU buffers from the current CPU vertex/index data
		void rebuildGPU() {
			deleteBuffers();
			createBuffers();
		}

		glm::mat4 applyLocalTransform() {
			for (auto& v : _vertices) {
				glm::vec4 p = localTransform * glm::vec4(v._pos, 1.0f);
				v._pos = glm::vec3(p);
				if (glm::length(v._normal) > 0.0f) {
					glm::vec4 n = localTransform * glm::vec4(v._normal, 0.0f);
					v._normal = glm::normalize(glm::vec3(n));
				}
			}
			return localTransform;
		}

		bool hasLocalTransform() const { return localTransform != glm::mat4(1.0f); }

	private:
		std::unique_ptr<render::VertexIndexBuffer> _rndrBffrMngr;

		int id = 0;
		std::string  _name = "obj" + id;

		// Default material properties
		glm::vec3 _albedo = glm::vec3(0.4, 0.4, 0.4);
		float _metallic = 0.1f;
		float _roughness = 0.5f;

	};
} // namespace scene