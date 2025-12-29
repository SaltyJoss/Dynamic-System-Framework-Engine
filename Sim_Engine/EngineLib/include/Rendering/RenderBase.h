#pragma once

// =============================================
//            File: RenderBase.h
// =============================================
// Base classes for rendering components such as vertex/index buffers and render context.
//
// Summary:
// =============================================
//	
// classes:
// --------------------------------------------
// VertexIndexBuffer
//      -> Abstract base class for managing vertex and index buffers.
// FrameBuffer
//      -> Abstract base class for managing framebuffers.
// RenderContext
//      -> Abstract base class for managing the rendering context.
// --------------------------------------------
//
// ============================================
// 
// VertexIndexBuffer:
// ============================================
// 
// public:
// --------------------------------------------
// virtual ~VertexIndexBuffer()
//      -> Virtual destructor for the VertexIndexBuffer class.
// VertexIndexBuffer() : _VAO{ 0 }, _VBO{ 0 }, _EBO{ 0 } {}
//      -> Constructor initializing VAO, VBO, and EBO to zero.
// virtual void createBuffers(const std::vector<scene::VertexHolder>& verticies, const std::vector<unsigned int>& indices) = 0
//      -> Pure virtual method to create vertex and index buffers.
// virtual void deleteBuffers() = 0
//      -> Pure virtual method to delete vertex and index buffers.
// virtual void bind() = 0
//      -> Pure virtual method to bind the buffers.
// virtual void unbind() = 0
//      -> Pure virtual method to unbind the buffers.
// virtual void draw(int indxCount) = 0
//      -> Pure virtual method to draw the buffers.
// --------------------------------------------
//
// protected:
// --------------------------------------------
// GLuint _VAO
//      -> OpenGL Vertex Array Object ID.
// GLuint _VBO
//      -> OpenGL Vertex Buffer Object ID.
// GLuint _EBO
//      -> OpenGL Element Buffer Object ID.
// --------------------------------------------
// ============================================
//
// FrameBuffer:
// ============================================
// 
// public:
// --------------------------------------------
// virtual ~FrameBuffer()
//      -> Virtual destructor for the FrameBuffer class.
// FrameBuffer() : _FBO{ 0 }, _depthID{ 0 } {}
//      -> Constructor initializing FBO and depthID to zero.
// virtual void createBuffers(int32_t width, int32_t height) = 0
//      -> Pure virtual method to create framebuffer buffers.
// virtual void deleteBuffers() = 0
//      -> Pure virtual method to delete framebuffer buffers.
// virtual void bind() = 0
//      -> Pure virtual method to bind the framebuffer.
// virtual void unbind() = 0
//      -> Pure virtual method to unbind the framebuffer.
// virtual uint32_t getTexture() = 0
//      -> Pure virtual method to get the texture ID of the framebuffer.
// --------------------------------------------
//
// protected:
// --------------------------------------------
// uint32_t _FBO = 0
//      -> Framebuffer Object ID.
// uint32_t _texID = 0
//      -> Texture ID associated with the framebuffer.
// uint32_t _depthID = 0
//      -> Depth buffer ID.
// int32_t _width = 0
//      -> Width of the framebuffer.
// int32_t _height = 0
//      -> Height of the framebuffer.
// --------------------------------------------
// ============================================
// 
// RenderContext:
// ============================================
// 
// public:
// --------------------------------------------
// virtual ~RenderContext()
// 		-> Virtual destructor for the RenderContext class.
// RenderContext() : _window(nullptr) {}
//      -> Constructor initializing the window pointer to nullptr.
// virtual bool init(window::IWindow* win)
//      -> Virtual method to initialize the render context with a window.
// virtual void preRender() = 0
//      -> Pure virtual method for pre-rendering operations.
// virtual void postRender() = 0
//      -> Pure virtual method for post-rendering operations.
// virtual void end() = 0
//      -> Pure virtual method to end the rendering context.
// --------------------------------------------
// 
// protected:
// --------------------------------------------
// window::IWindow* _window
//      -> Pointer to the associated window.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================


#include "EngineCore.h"

#include "Platform/Window.h"
#include "Scene/VertexHolder.h"

#include <cstdint>
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

using GLuint = std::uint32_t;

namespace render {
	class ENGINE_API VertexIndexBuffer {
	public:
		virtual ~VertexIndexBuffer() = default;

		// Replaces and Centralises old VAO, VBO, EBO classes -> See OpenGLBufferManager
		VertexIndexBuffer() : _VAO{ 0 }, _VBO{ 0 }, _EBO{ 0 } {}

		virtual void createBuffers(const std::vector<scene::VertexHolder>& verticies, const std::vector<unsigned int>& indices) = 0;
		virtual void deleteBuffers() = 0;
		virtual void bind() = 0;
		virtual void unbind() = 0;
		virtual void draw(int indxCount) = 0;

	protected:
		GLuint _VAO;
		GLuint _VBO;
		GLuint _EBO;
	};

	class FrameBuffer {
	public:
		virtual ~FrameBuffer() = default;

		// This was NOT working in previous version, so revised code with docs and research -> See OpenGLBufferManager
		FrameBuffer() : _FBO{ 0 }, _depthID{ 0 } {}

		virtual void createBuffers(int32_t width, int32_t height, int samples) = 0;
		virtual void deleteBuffers() = 0;
		virtual void bind() = 0;
		virtual void unbind() = 0;
		virtual uint32_t getTexture() = 0;

	protected:
		uint32_t _FBO = 0;
		uint32_t _texID = 0;
		uint32_t _depthID = 0;

		int32_t _width = 0;
		int32_t _height = 0;

		uint32_t _msaaFBO = 0;
		uint32_t _msaaColor = 0;     // GL_TEXTURE_2D_MULTISAMPLE
		uint32_t _msaaDepthRBO = 0;  // renderbuffer
		int _samples = 1;
	};

	class RenderContext {
	public:
		virtual ~RenderContext() = default;

		// Centeralised way to gain context on the renders' process -> see OpenGLContext
		RenderContext() : _window(nullptr) {}

		virtual bool init(window::IWindow* win) {
			_window = win;
			return true;
		}

		virtual void preRender() = 0;
		virtual void postRender() = 0;
		virtual void end() = 0;

	protected:
		window::IWindow* _window;
	};
}