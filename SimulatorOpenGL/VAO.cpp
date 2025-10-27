#include "VAO.h"

// Constructor that generates a Vertex Array Object
VAO::VAO()
{
	glGenVertexArrays(1, &ID);	// Generate 1 vertex array
}

// Links a VBO to the VAO
void VAO::LinkVBO(VBO& VBO, GLuint layout)
{
	VBO.Bind();	// Bind the VBO

	glVertexAttribPointer(layout, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); // Link VBO attributes such as position
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
