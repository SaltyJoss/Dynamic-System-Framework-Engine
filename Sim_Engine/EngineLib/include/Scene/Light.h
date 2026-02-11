#pragma once
// File:   Light.h
// GitHub: SaltyJoss
#include "EngineCore.h"

#include "Scene/Element.h"
#include "Rendering/ShaderUtil.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace scene {
	class ENGINE_API Light : public Element {
    public:

        Light() {
            _direction = glm::vec3(5.0f, -25.0f, 1.5f);
            _position = glm::vec3(-4.0f, 20.0f, 12.0f);
            _colour = glm::vec3(1.0f, 0.98f, 0.98f);
			_intensity = 2.5f;
            _size = 25.0f;
        }
        ~Light() {}

		// Default light configurations
        struct DirectionalLight {
            glm::vec3 direction{ 5.0f, -25.0f, 1.5f };
            glm::vec3 colour{ 1.0f, 0.98f, 0.95f };
            float intensity = 2.5f;
        };

		// Point light configuration
        struct PointLight {
            glm::vec3 position{ 0.0f, 10.0f, 0.0f };
            glm::vec3 colour{ 1.0f, 0.98f, 0.95f };
            float intensity = 25.0f;
            float size = 15.0f;
        };

		void setDirection(const glm::vec3& dir) { _direction = glm::normalize(dir); }
		void setPosition(const glm::vec3& pos) { _position = pos; }
        void setIntensity(const float intsy) { _intensity = intsy; }
		void setSize(const float size) { _size = size; }

        glm::vec3 getPosition() const { return _position; }
		glm::vec3 getDirection() const { return _direction; }
        float getIntensity() const { return _intensity; }
        glm::vec3 getColour() const { return _colour; }

        void update(shaders::Shader* shader) override {
            if (_isDirectional) {
                glm::vec3 dir = glm::normalize(_direction);
                shader->setVec3(dir, "lightDirection");
                shader->setFlt1(_intensity, "lightIntensity");
                shader->setVec3(_colour, "lightColour");
                shader->setFlt1(_size, "lightSize");
            }
            else {
                shader->setVec3(_position, "lightPosition");
                shader->setFlt1(_intensity, "lightIntensity");
                shader->setVec3(_colour, "lightColour");
                shader->setFlt1(_size, "lightSize");
            }
        }

        bool _isDirectional = true;

        glm::vec3 _direction;
        glm::vec3 _colour;
		glm::vec3 _position;
        float _intensity;
        float _size;
    };
}