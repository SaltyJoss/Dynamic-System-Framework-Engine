// DSFE_GUI Light.h
#pragma once

#include "Scene/Element.h"
#include "Platform/Logger.h"

namespace scene {
	class Light : public Element {
    public:

		Light() {
			_direction = glm::vec3(1.0f, -1.5f, 0.8f);
			_position = glm::vec3(-4.0f, 20.0f, 12.0f);
			_colour = glm::vec3(1.0f, 0.99f, 0.96f);
			_intensity = 1.8f;
			_size = 25.0f;
		}
        ~Light() {}

		// Default light configurations
        struct DirectionalLight {
            glm::vec3 direction{ 1.0f, -1.5f, 0.8f };
            glm::vec3 colour{ 1.0f, 0.99f, 0.96f };
            float intensity = 1.8f;
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

        void update() override {
        }

        bool _isDirectional = true;

        glm::vec3 _direction;
        glm::vec3 _colour;
		glm::vec3 _position;
        float _intensity;
        float _size;
    };
}