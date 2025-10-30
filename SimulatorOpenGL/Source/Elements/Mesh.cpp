#include "ch.h"
#include "Mesh.h"

#include "Render/OpenGLBufferManager.h"

namespace elements {
	void Mesh::init() {
		_rndrBffrMngr = std::make_unique<render::OpenGLVertexIndexBuffer>();
		createBuffers();
	}

	elements::Mesh::~Mesh() { deleteBuffers(); }

	bool Mesh::load(const std::string& path)
	{
		return false;
	}

	void Mesh::createBuffers() { _rndrBffrMngr->createBuffers(_vertices, _vertexIndices); }
	void Mesh::deleteBuffers() { _rndrBffrMngr->deleteBuffers(); }
	void Mesh::bind() { _rndrBffrMngr->deleteBuffers(); }
	void Mesh::unbind() { _rndrBffrMngr->unbind(); }
	void Mesh::render() { _rndrBffrMngr->draw((int) _vertexIndices.size()); }

}