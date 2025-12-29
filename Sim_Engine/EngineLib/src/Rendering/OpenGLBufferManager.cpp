
#include "pch.h"

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Rendering/OpenGLBufferManager.h"
#include "EngineLib/LogMacros.h"

namespace render {
	/*
	* --------------------------------------------
	*      OPENGL VERTEX-INDEX-BUFFER METHODS
	* --------------------------------------------
	* 
	* Summary:
	* --------------------------------------------
	* createBuffers(const std::vector<scene::VertexHolder>& vertices, const std::vector<unsigned int>& indices) -> Creates the VAO, VBO, and EBO buffers and uploads the vertex and index data to the GPU.
	* deleteBuffers() -> Deletes the VAO, VBO, and EBO buffers.
	* bind() -> Binds the VAO for rendering.
	* unbind() -> Unbinds the VAO.
	* draw(int indxCount) -> Draws the scene using the bound VAO and the specified index count.
	* --------------------------------------------
	*/
	void render::OpenGLVertexIndexBuffer::createBuffers(const std::vector<scene::VertexHolder>& vertices, const std::vector<unsigned int>& indices) {
		LOG_INFO("Called createBuffers() with %zu vertices and %zu indices", vertices.size(), indices.size());

		glGenVertexArrays(1, &_VAO);
		glGenBuffers(1, &_EBO);
		glGenBuffers(1, &_VBO);

		if (!_VAO || !_EBO || !_VBO) { LOG_ERROR("Failed to generate VAO/VBO/EBO"); return; }

		glBindVertexArray(_VAO);

		glBindBuffer(GL_ARRAY_BUFFER, _VBO);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(scene::VertexHolder), vertices.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(scene::VertexHolder), (void*)offsetof(scene::VertexHolder, _pos));

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(scene::VertexHolder), (void*)offsetof(scene::VertexHolder, _normal));

		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(scene::VertexHolder), (void*)offsetof(scene::VertexHolder, _texCoord));

		glBindVertexArray(0);

		LOG_INFO("OpenGLVertexIndexBuffer buffers created successfully");
	}

	void render::OpenGLVertexIndexBuffer::deleteBuffers() {
		LOG_INFO("Deleting OpenGLVertexIndexBuffer buffers");

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glDeleteBuffers(1, &_EBO);
		glDeleteBuffers(1, &_VBO);
		glDeleteVertexArrays(1, &_VAO);

		LOG_INFO("Buffers deleted");
	}

	void render::OpenGLVertexIndexBuffer::bind() { glBindVertexArray(_VAO); }

	void render::OpenGLVertexIndexBuffer::unbind() { glBindVertexArray(0); }

	void render::OpenGLVertexIndexBuffer::draw(int indxCount) {
		bind();
		glDrawElements(GL_TRIANGLES, indxCount, GL_UNSIGNED_INT, nullptr);
		unbind();
	}


	/*
	* --------------------------------------------
	*      OPENGL FRAME-BUFFER METHODS
	* --------------------------------------------
	* 
	* Summary:
	* -------------------------------------------
	* createBuffers(int32_t width, int32_t height) -> Creates the framebuffer and its associated color and depth textures with the specified width and height.
	* deleteBuffers() -> Deletes the framebuffer and its associated textures.
	* bind() -> Binds the framebuffer for rendering and sets the viewport to its dimensions.
	* unbind() -> Unbinds the framebuffer, reverting to the default framebuffer.
	* --------------------------------------------
	*/
	void render::OpenGLFrameBuffer::createBuffers(int32_t width, int32_t height, int samples) {
		LOG_INFO("Creating framebuffer buffers with size %dx%d (samples=%d)", width, height, samples);
		_width = width;
		_height = height;

		_samples = (samples < 1) ? 1 : samples;

		if (_FBO || _msaaFBO) {
			LOG_WARN("Framebuffer already exists. Deleting old buffers.");
			deleteBuffers();
		}

		glGenFramebuffers(1, &_FBO);
		glBindFramebuffer(GL_FRAMEBUFFER, _FBO);
		glCreateTextures(GL_TEXTURE_2D, 1, &_texID);
		glBindTexture(GL_TEXTURE_2D, _texID);

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, _width, _height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, _texID, 0);

		glCreateTextures(GL_TEXTURE_2D, 1, &_depthID);
		glBindTexture(GL_TEXTURE_2D, _depthID);
		glTexStorage2D(GL_TEXTURE_2D, 1, GL_DEPTH24_STENCIL8, _width, _height);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, _depthID, 0);
		GLenum buffers[4] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(1, buffers);

		// ----------------------------
		// MSAA FBO (optional) - what you render into when samples > 1
		// ----------------------------
		if (_samples > 1)
		{
			glGenFramebuffers(1, &_msaaFBO);
			glBindFramebuffer(GL_FRAMEBUFFER, _msaaFBO);

			glCreateTextures(GL_TEXTURE_2D_MULTISAMPLE, 1, &_msaaColor);
			glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, _msaaColor);
			glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, _samples, GL_RGBA8, _width, _height, GL_TRUE);
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, _msaaColor, 0);

			glGenRenderbuffers(1, &_msaaDepthRBO);
			glBindRenderbuffer(GL_RENDERBUFFER, _msaaDepthRBO);
			glRenderbufferStorageMultisample(GL_RENDERBUFFER, _samples, GL_DEPTH24_STENCIL8, _width, _height);
			glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, _msaaDepthRBO);

			GLenum drawBuf = GL_COLOR_ATTACHMENT0;
			glDrawBuffers(1, &drawBuf);
		}

		unbind();

		LOG_INFO_ONCE("frame tex=%u (samples=%d)", _texID, _samples);
		LOG_INFO_ONCE("msaaFBO=%u resolveFBO=%u", _msaaFBO, _FBO);

		D_INFO_ONCE("frame tex=%u (samples=%d)", _texID, _samples);
		D_INFO_ONCE("msaaFBO=%u resolveFBO=%u", _msaaFBO, _FBO);

		LOG_INFO("Framebuffer buffers created successfully");
	}

	void render::OpenGLFrameBuffer::deleteBuffers() {
		if (_FBO) {
			LOG_INFO("Deleting framebuffer buffers");
			if (_msaaFBO) glDeleteFramebuffers(1, &_msaaFBO);
			if (_msaaColor) glDeleteTextures(1, &_msaaColor);
			if (_msaaDepthRBO) glDeleteRenderbuffers(1, &_msaaDepthRBO);

			if (_FBO) glDeleteFramebuffers(1, &_FBO);
			if (_texID) glDeleteTextures(1, &_texID);
			if (_depthID) glDeleteTextures(1, &_depthID);

			_msaaFBO = 0;
			_msaaColor = 0;
			_msaaDepthRBO = 0;

			_FBO = 0;
			_texID = 0;
			_depthID = 0;

			LOG_INFO("Framebuffer buffers deleted");
		}
		else { LOG_WARN("Attempted to delete framebuffer buffers but none exist"); }
	}

	void render::OpenGLFrameBuffer::bind() {
		if (!_FBO) { LOG_WARN_ONCE("Attempted to bind framebuffer but FBO is 0"); return; }

		const bool useMSAA = (_samples > 1 && _msaaFBO != 0);
		uint32_t target = useMSAA ? _msaaFBO : _FBO;

		glBindFramebuffer(GL_FRAMEBUFFER, target);
		glViewport(0, 0, _width, _height);

		// CRITICAL: restore draw/read buffers & colour writes
		glDrawBuffer(GL_COLOR_ATTACHMENT0);
		glReadBuffer(GL_COLOR_ATTACHMENT0);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDisable(GL_SCISSOR_TEST);

		LOG_INFO_ONCE("FB bind target=%u (msaaFBO=%u resolveFBO=%u samples=%d)", target, _msaaFBO, _FBO, _samples);
	}

	void render::OpenGLFrameBuffer::unbind() {

		const bool useMSAA = (_samples > 1) && (_msaaFBO != 0);

		if (useMSAA && _FBO) {
			glBindFramebuffer(GL_READ_FRAMEBUFFER, _msaaFBO);
			glReadBuffer(GL_COLOR_ATTACHMENT0);

			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _FBO);
			glDrawBuffer(GL_COLOR_ATTACHMENT0);

			glBlitFramebuffer(
				0, 0, _width, _height,
				0, 0, _width, _height,
				GL_COLOR_BUFFER_BIT,
				GL_NEAREST
			);

			LOG_INFO_ONCE("Resolved MSAA -> resolve FBO");

		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		glDrawBuffer(GL_BACK);
		glReadBuffer(GL_BACK);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDisable(GL_SCISSOR_TEST);
	}
	uint32_t render::OpenGLFrameBuffer::getTexture() { return _texID; }
}


