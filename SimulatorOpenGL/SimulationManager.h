#ifndef SIMULATION_MANAGER_H
#define SIMULATOR_MANAGER_H

#include "CoreIncludes.h"

class SimulationManager
{
public:
    bool Init();            // create shaders, buffers, etc.
    void Update(float dt);  // update logic or physics
    void Render();          // issue OpenGL draw calls
    void Terminate();       // release GPU resources

private:
    unsigned int shaderProgram = 0;     // Can use previous logic
    unsigned int VAO = 0, VBO = 0;      // Can use previous logic

    // optional: camera, time, scene data
    glm::mat4 view;
    glm::mat4 projection;
    glm::mat4 model;
};

#endif
