#pragma once

// =============================================
//            File: OpenGLBufferManager.h
// =============================================
// Classes for managing OpenGL vertex/index buffers and framebuffers.
//
// Summary:
// =============================================
//
// classes:
// --------------------------------------------
// class OpenGLVertexIndexBuffer : public VertexIndexBuffer
//		-> OpenGLVertexIndexBuffer class manages OpenGL Vertex Array Object (VAO), Vertex Buffer Object (VBO), and Element Buffer Object (EBO).
// class OpenGLFrameBuffer : public FrameBuffer
//		-> OpenGLFrameBuffer class manages an OpenGL framebuffer object (FBO) for off-screen rendering.
// --------------------------------------------
// ============================================
// 
// OpenGLVertexIndexBuffer : public VertexIndexBuffer
// =============================================
// 
// public:
// --------------------------------------------
// void createBuffers(const std::vector<scene::VertexHolder>& vertices, const std::vector<unsigned int>& indices)
//      -> Creates OpenGL buffers for the given vertices and indices.
// void deleteBuffers()
//      -> Deletes the OpenGL buffers.
// void bind()
//      -> Binds the OpenGL buffers for rendering.
// void unbind()
//      -> Unbinds the OpenGL buffers.
// void draw(int indxCount)
//      -> Draws the elements using the bound buffers.
// --------------------------------------------
// ============================================
// 
// OpenGLFrameBuffer : public FrameBuffer
// =============================================
// 
// public:
// --------------------------------------------
// void createBuffers(int32_t width, int32_t height)
//      -> Creates an OpenGL framebuffer with the specified width and height.
// void deleteBuffers()
//      -> Deletes the OpenGL framebuffer and associated resources.
// void bind()
//      -> Binds the OpenGL framebuffer for rendering.
// void unbind()
//      -> Unbinds the OpenGL framebuffer.
// uint32_t getTexture()
//      -> Returns the texture ID associated with the framebuffer.
// --------------------------------------------
//
// Built upon code from:
// ============================================
//	 GitHub: jayanam/jgl_demos/JGL_MeshLoader
// ============================================
// 
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"

#include "RenderBase.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace render {
	class ENGINE_API OpenGLVertexIndexBuffer : public VertexIndexBuffer {
		public:
			OpenGLVertexIndexBuffer() : VertexIndexBuffer() {}

			void createBuffers(const std::vector<scene::VertexHolder>& vertices, const std::vector<unsigned int>& indices) override;
			void deleteBuffers() override;
			void bind() override;
			void unbind() override;
			void draw(int indxCount) override;
	};

	class OpenGLFrameBuffer : public FrameBuffer {
		public:
			void createBuffers(int32_t width, int32_t height, int samples = 1) override;
			void deleteBuffers() override;
			void bind() override;
			void unbind() override;
			uint32_t getTexture() override;
	};
}