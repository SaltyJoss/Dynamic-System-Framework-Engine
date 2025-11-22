#pragma once
#include "EngineCore.h"

#include "Scene/Element.h"
#include "Rendering/ShaderUtil.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace elements {
    class ENGINE_API Light : public Element
    {
    public:

        Light() {
            _direction = glm::vec3(-1.0f, -1.0f, -0.3f);
            _position = glm::vec3{ 1.5f, 3.5f, 3.0f };
            _colour = glm::vec3(1.0f, 1.0f, 1.0f);
            _strength = 50.0f;
			_intensity = 50.0f;
            _size = 25.0f;
        }

        struct DirectionalLight {
            glm::vec3 direction{ -1.0, -0.3f, 0.2f };
            glm::vec3 colour{ 1.0f, 0.98f, 0.95f };
            float intensity = 1.0f;
        };

        ~Light() {}

		void setDirection(const glm::vec3& dir) { _direction = glm::normalize(dir); }

        glm::vec3 getPosition() const { return _position; }
		glm::vec3 getDirection() const { return _direction; }
        glm::vec3 getColour() const { return _colour; }
		float getIntensity() const { return _intensity; }    

        void update(shaders::Shader* shader) override {
            if (_isDirectional) {
                glm::vec3 dir = glm::normalize(_direction);
                shader->setVec3(dir, "lightDirection");
                shader->setVec3(_colour, "lightColour");
                shader->setFlt1(_intensity, "lightIntensity");
            }
            else {
                shader->setVec3(_position, "lightPosition");
                shader->setVec3(_colour, "lightColour");
                shader->setFlt1(_intensity, "lightIntensity");
                shader->setFlt1(_size, "lightSize");
            }
        }

        bool _isDirectional = true;

        glm::vec3 _direction;
        glm::vec3 _colour;
		glm::vec3 _position;
		float _strength;
        float _intensity;
        float _size;
    };
}