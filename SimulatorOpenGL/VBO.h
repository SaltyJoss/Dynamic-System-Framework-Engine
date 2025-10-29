#ifndef VBO_CLASS_H
#define VBO_CLASS_H

#include <glad/glad.h>

class VBO {
public:
    GLuint ID;
    VBO() : ID(0) {} // default constructor
    void Init(const GLfloat* vertices, GLsizeiptr size);
    void Bind();	// Binds the VBO
    void Unbind();	// Unbinds the VBO
    void Delete();	// Deletes the VBO
};

#endif