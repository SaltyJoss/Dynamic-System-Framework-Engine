#include "OpenGLBufferManager.h"

namespace render {
	// --- OpenGLVertexIndexBuffer ---
	void render::OpenGLVertexIndexBuffer::createBuffers(const std::vector<elements::VertexHolder>& verticies, const std::vector<unsigned int>& indices)
	{
	}

	void render::OpenGLVertexIndexBuffer::deleteBuffers()
	{
	}

	void render::OpenGLVertexIndexBuffer::bind()
	{
	}

	void render::OpenGLVertexIndexBuffer::unbind()
	{
	}

	void render::OpenGLVertexIndexBuffer::draw(int indxCount)
	{
	}

	// --- OpenGLFrameBuffer ---
	void render::OpenGLFrameBuffer::createBuffers(int32_t width, int32_t height)
	{
	}

	void render::OpenGLFrameBuffer::deleteBuffers()
	{
	}

	void render::OpenGLFrameBuffer::bind()
	{
	}

	void render::OpenGLFrameBuffer::unbind()
	{
	}

	uint32_t render::OpenGLFrameBuffer::getTexture()
	{
		return 0;
	}
}


