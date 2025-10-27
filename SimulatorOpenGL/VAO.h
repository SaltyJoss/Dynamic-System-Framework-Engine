#ifndef VAO_CLASS_H
#define VAO_CLASS_H

#include <glad/glad.h>
#include "VBO.h"

class VAO
{
public:
	GLuint ID;
	VAO();	// Constructor that generates a Vertex Array Object

	void LinkAttrib(VBO& VBO, GLuint layout, GLuint numComponents, GLenum type, GLsizeiptr stride, void* offset);
	void Bind();			// Binds the VAO
	void Unbind();			// Unbinds the VAO
	void Delete();			// Deletes the VAO
};

#endif
