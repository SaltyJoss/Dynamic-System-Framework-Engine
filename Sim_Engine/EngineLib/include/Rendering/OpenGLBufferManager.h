#pragma once
// File:   OpenGLBufferManager.h
// GitHub: SaltyJoss
#include "EngineCore.h"

#include "RenderBase.h"
#include "Platform/Logger.h"

namespace render {
	// This class manages the creation, binding, and deletion of OpenGL vertex and index buffers (VAO, VBO, EBO) for rendering meshes.
	class ENGINE_API OpenGLVertexIndexBuffer : public VertexIndexBuffer {
		public:
			OpenGLVertexIndexBuffer() : VertexIndexBuffer() {}

			void createBuffers(const std::vector<scene::VertexHolder>& vertices, const std::vector<unsigned int>& indices) override;
			void deleteBuffers() override;
			void bind() override;
			void unbind() override;
			void draw(int indxCount) override;
	};

	// This class manages the creation, binding, and deletion of OpenGL framebuffer objects (FBOs) for off-screen rendering, including support for multisampling (MSAA).
	class OpenGLFrameBuffer : public FrameBuffer {
		public:
			void createBuffers(int32_t width, int32_t height, int samples = 1) override;
			void deleteBuffers() override;
			void bind() override;
			void unbind() override;
			void endSetup();
			uint32_t getTexture() override;
	};
} // namespace render