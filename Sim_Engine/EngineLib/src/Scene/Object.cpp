#include "pch.h"
#include "Scene/Object.h"

glm::mat4 scene::Transform::toMatrix() const
{
    glm::mat4 model(1.0f);
    model = glm::translate(model, position);
    model *= glm::toMat4(rotQ); // quaternions use!
    model = glm::scale(model, scale);
    return model;
}
