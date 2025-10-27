#include "VAO.h"

// Constructor that generates a Vertex Array Object
VAO::VAO()
{
	glGenVertexArrays(1, &ID);	// Generate 1 vertex array
}

// Links a VBO to the VAO
void VAO::LinkAttrib(VBO& VBO, GLuint layout, GLuint numComponents, GLenum type, GLsizeiptr stride, void* offset)
{
	VBO.Bind();	// Bind the VBO

	glVertexAttribPointer(layout, numComponents, type, GL_FALSE, stride, offset); // Link VBO attributes such as position
	glEnableVertexAttribArray(layout); // Enable the VBO attribute to be used by the VAO

	VBO.Unbind();			// Unbind the VBO
}

// Binds the VAO
void VAO::Bind()
{
	glBindVertexArray(ID);
}

// Unbinds the VAO
void VAO::Unbind()
{
	glBindVertexArray(0);
}

// Deletes the VAO
void VAO::Delete()
{
	glDeleteVertexArrays(1, &ID);
}
