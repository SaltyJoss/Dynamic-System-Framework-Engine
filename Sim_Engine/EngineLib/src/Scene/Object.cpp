#include "pch.h"
#include "Scene/Object.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

glm::mat4 scene::Transform::toMatrix() const
{
    glm::mat4 model(1.0f);


    // translation
    model = glm::translate(model, position);

    // rotation (yawPitchRoll expects yaw, pitch, roll)
    model *= glm::yawPitchRoll(
        rotation.y,
        rotation.x,
        rotation.z
    );

    // scale
    model = glm::scale(model, scale);

    return model;
}
