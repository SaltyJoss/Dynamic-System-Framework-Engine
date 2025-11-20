#include "pch.h"
#include "Object.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>

glm::mat4 elements::Transform::toMatrix() const
{
    glm::mat4 model(1.0f);

    // scale
    model = glm::scale(model, scale);

    // rotation (yawPitchRoll expects yaw, pitch, roll)
    model *= glm::yawPitchRoll(
        rotation.y,
        rotation.x,
        rotation.z
    );

    // translation
    model = glm::translate(model, position);

    return model;
}
