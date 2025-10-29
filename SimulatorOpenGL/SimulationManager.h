#ifndef SIMULATION_MANAGER_H
#define SIMULATION_MANAGER_H

#include "CoreIncludes.h"
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"
#include "shaderClass.h"

class SimulationManager
{
private:
    class Buffers {
    public:
        static const GLfloat vertices[];
        static const GLuint indices[];
    };

    std::unique_ptr<Shader> shaderProgram;

    VAO _VAO;
    VBO _VBO;
    EBO _EBO;

	GLuint uniID;

    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 projection;

public:
    bool Init();            // create shaders, buffers, etc.
    void Update(float dt);  // update logic or physics
    void Render();          // issue OpenGL draw calls
    void Terminate();       // release GPU resources
};

#endif
