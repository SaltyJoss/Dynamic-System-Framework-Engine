#ifndef VAO_CLASS_H
#define VAO_CLASS_H

#include <glad/glad.h>
#include "VBO.h"

class VAO
{
public:
	GLuint ID;
	VAO();	// Constructor that generates a Vertex Array Object

	void LinkVBO(VBO& VBO, GLuint layout); // Links a VBO to the VAO
	void Bind();			// Binds the VAO
	void Unbind();			// Unbinds the VAO
	void Delete();			// Deletes the VAO
};

#endif
