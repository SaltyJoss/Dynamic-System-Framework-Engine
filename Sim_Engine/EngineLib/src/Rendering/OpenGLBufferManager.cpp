#include "pch.h"
// File:   OpenGLBufferManager.cpp
// GitHub: SaltyJoss
#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Rendering/OpenGLBufferManager.h"

#include "EngineLib/LogMacros.h"

namespace render {
	// Creates the VAO, VBO, and EBO for this buffer using the provided vertex and index data
	void OpenGLVertexIndexBuffer::createBuffers(const std::vector<assets::VertexHolder>& vertices, const std::vector<unsigned int>& indices) {
		//LOG_INFO("Called createBuffers() with %zu vertices and %zu indices", vertices.size(), indices.size());

		glGenVertexArrays(1, &_VAO);
		glGenBuffers(1, &_EBO);
		glGenBuffers(1, &_VBO);

		if (!_VAO || !_EBO || !_VBO) { LOG_ERROR("Failed to generate VAO/VBO/EBO"); return; }

		glBindVertexArray(_VAO);

		glBindBuffer(GL_ARRAY_BUFFER, _VBO);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(assets::VertexHolder), vertices.data(), GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(assets::VertexHolder), (void*)offsetof(assets::VertexHolder, _pos));

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(assets::VertexHolder), (void*)offsetof(assets::VertexHolder, _normal));

		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(assets::VertexHolder), (void*)offsetof(assets::VertexHolder, _texCoord));

		glBindVertexArray(0);

		//LOG_INFO("OpenGLVertexIndexBuffer buffers created successfully");
	}

	// Deletes the VAO, VBO, and EBO associated with this buffer
	void OpenGLVertexIndexBuffer::deleteBuffers() {
		//LOG_INFO("Deleting OpenGLVertexIndexBuffer buffers");

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glDeleteBuffers(1, &_EBO);
		glDeleteBuffers(1, &_VBO);
		glDeleteVertexArrays(1, &_VAO);

		//LOG_INFO("Buffers deleted");
	}

	// Binds the VAO for this buffer, making it active for rendering
	void OpenGLVertexIndexBuffer::bind() { glBindVertexArray(_VAO); }
	// Unbinds the VAO, resetting to the default state
	void OpenGLVertexIndexBuffer::unbind() { glBindVertexArray(0); }

	// Issues a draw call using the currently bound VAO and EBO, rendering the specified number of indices as triangles
	void OpenGLVertexIndexBuffer::draw(int indxCount) {
		bind();
		glDrawElements(GL_TRIANGLES, indxCount, GL_UNSIGNED_INT, nullptr);
		unbind();
	}

	// Creates the framebuffer and its associated color and depth attachments, with optional MSAA support
	void OpenGLFrameBuffer::createBuffers(int32_t width, int32_t height, int samples) {
		//LOG_INFO("Creating framebuffer buffers with size %dx%d (samples=%d)", width, height, samples);
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

		// Allocate full mip chain (immutable storage)
		int levels = 1 + (int)std::floor(std::log2((double)std::max(_width, _height)));
		glTexStorage2D(GL_TEXTURE_2D, levels, GL_RGBA8, _width, _height);

		// Use mipmaps for minification (downsampling)
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		// Attach level 0 to the resolve FBO
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
		
		// Check FBO completeness
		if (_samples > 1) {
			glGenFramebuffers(1, &_msaaFBO);
			glBindFramebuffer(GL_FRAMEBUFFER, _msaaFBO);

			glCreateTextures(GL_TEXTURE_2D_MULTISAMPLE, 1, &_msaaColour);
			glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, _msaaColour);
			glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, _samples, GL_RGBA8, _width, _height, GL_TRUE);
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, _msaaColour, 0);

			glGenRenderbuffers(1, &_msaaDepthRBO);
			glBindRenderbuffer(GL_RENDERBUFFER, _msaaDepthRBO);
			glRenderbufferStorageMultisample(GL_RENDERBUFFER, _samples, GL_DEPTH24_STENCIL8, _width, _height);
			glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, _msaaDepthRBO);

			GLenum drawBuf = GL_COLOR_ATTACHMENT0;
			glDrawBuffers(1, &drawBuf);
		}

		auto checkFBO = [&](const char* name) {
			GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
			if (status != GL_FRAMEBUFFER_COMPLETE) {
				LOG_ERROR("%s incomplete: 0x%X", name, (unsigned)status);
				return false;
			}
			return true;
		};

		// --- Check RESOLVE FBO completeness ---
		glBindFramebuffer(GL_FRAMEBUFFER, _FBO);
		if (!checkFBO("Resolve FBO")) {
			LOG_ERROR("Resolve FBO failed; disabling framebuffer.");
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			deleteBuffers();
			return;
		}

		// --- Check MSAA FBO completeness ---
		if (_samples > 1 && _msaaFBO != 0) {
			glBindFramebuffer(GL_FRAMEBUFFER, _msaaFBO);
			if (!checkFBO("MSAA FBO")) {
				LOG_ERROR("MSAA FBO failed; falling back to non-MSAA.");

				// destroy msaa only (keep resolve FBO alive)
				glDeleteFramebuffers(1, &_msaaFBO); _msaaFBO = 0;
				glDeleteTextures(1, &_msaaColour);  _msaaColour = 0;
				glDeleteRenderbuffers(1, &_msaaDepthRBO); _msaaDepthRBO = 0;

				_samples = 1; // force no msaa path
			}
		}

		endSetup();

		//LOG_INFO_ONCE("frame tex=%u (samples=%d)", _texID, _samples);
		//LOG_INFO_ONCE("msaaFBO=%u resolveFBO=%u", _msaaFBO, _FBO);
		//LOG_INFO("Framebuffer buffers created successfully");
	}

	// Deletes the framebuffer and its associated attachments
	void OpenGLFrameBuffer::deleteBuffers() {
		if (_FBO) {
			LOG_INFO("Deleting framebuffer buffers");
			if (_msaaFBO) glDeleteFramebuffers(1, &_msaaFBO);
			if (_msaaColour) glDeleteTextures(1, &_msaaColour);
			if (_msaaDepthRBO) glDeleteRenderbuffers(1, &_msaaDepthRBO);

			if (_FBO) glDeleteFramebuffers(1, &_FBO);
			if (_texID) glDeleteTextures(1, &_texID);
			if (_depthID) glDeleteTextures(1, &_depthID);

			_msaaFBO = 0;
			_msaaColour = 0;
			_msaaDepthRBO = 0;

			_FBO = 0;
			_texID = 0;
			_depthID = 0;

			//LOG_INFO("Framebuffer buffers deleted");
		}
		else { LOG_WARN("Attempted to delete framebuffer buffers but none exist"); }
	}

	// Binds the framebuffer for rendering, using the MSAA FBO if samples > 1, otherwise the resolve FBO
	void OpenGLFrameBuffer::bind() {
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

		//LOG_INFO_ONCE("FB bind target=%u (msaaFBO=%u resolveFBO=%u samples=%d)", target, _msaaFBO, _FBO, _samples);
	}

	// Unbinds the framebuffer, resolving MSAA if necessary, and restores default backbuffer state
	void OpenGLFrameBuffer::unbind() {
		const bool useMSAA = (_samples > 1) && (_msaaFBO != 0);

		if (useMSAA && _FBO)
		{
			// Resolve MSAA -> resolve FBO (texture-backed)
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
		}

		// CRITICAL: reset ALL framebuffer targets, not just GL_FRAMEBUFFER
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		// Restore default backbuffer state
		glDrawBuffer(GL_BACK);
		glReadBuffer(GL_BACK);

		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDisable(GL_SCISSOR_TEST);
	}

	// Called after creating or resizing buffers to ensure the default framebuffer state is correct
	void OpenGLFrameBuffer::endSetup() {
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		glDrawBuffer(GL_BACK);
		glReadBuffer(GL_BACK);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDisable(GL_SCISSOR_TEST);
	}

	// Returns the texture ID of the framebuffer's color attachment (the resolved texture if MSAA is used)
	uint32_t OpenGLFrameBuffer::getTexture() { return _texID; }
}


