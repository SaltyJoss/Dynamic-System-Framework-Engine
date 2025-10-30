#pragma once

#include "ch.h"

#include "Element.h"
#include "Shader/ShaderUtil.h"

namespace elements {
    class Light : public Element
    {
    public:

        Light() {
            mColor = glm::vec3(1.0f, 1.0f, 1.0f);
            mPosition = { 1.5f, 3.5f, 3.0f };
            mStrength = 100.0f;
        }

        ~Light() {}

        void update(shaders::Shader* shader) override {
            shader->setVec3(mPosition, "lightPosition");
            shader->setVec3(mColor * mStrength, "lightColour");
        }

        glm::vec3 mPosition;
        glm::vec3 mColor;
        float mStrength;
    };
}