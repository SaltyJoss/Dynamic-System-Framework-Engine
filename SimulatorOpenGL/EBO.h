#ifndef EBO_CLASS_H
#define EBO_CLASS_H

#include <glad/glad.h>

class EBO
{
public:
	GLuint ID;
	EBO(GLuint* indices, GLsizeiptr size);	// Constructor that generates an Element Buffer Object and links it to indices

	void Bind();	// Binds the EBO
	void Unbind();	// Unbinds the EBO
	void Delete();	// Deletes the EBO
};

#endif
