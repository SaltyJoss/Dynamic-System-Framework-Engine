#ifndef EBO_CLASS_H
#define EBO_CLASS_H

#include <glad/glad.h>

class EBO {
public:
    GLuint ID;
    EBO() : ID(0) {} // default constructor
    void Init(const GLuint* indices, GLsizeiptr size);
    void Bind();	// Binds the VBO
    void Unbind();	// Unbinds the VBO
    void Delete();	// Deletes the VBO
};

#endif
