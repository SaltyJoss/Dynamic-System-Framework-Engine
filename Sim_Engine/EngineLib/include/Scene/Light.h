#pragma once

// =============================================
//            File: Light.h
// =============================================
// Class representing a light source in the 3D scene.
//
// Summary:
// =============================================
//
// public:
// --------------------------------------------
// Light()
//      -> Constructor that initializes the light with default properties.
// ~Light()
//      -> Destructor for the Light class.
// void setDirection(const glm::vec3& dir)
//      -> Sets the direction of the light.
// void setIntensity(const float intsy)
//      -> Sets the intensity of the light.
// glm::vec3 getPosition() const
//      -> Returns the position of the light.
// glm::vec3 getDirection() const
//      -> Returns the direction of the light.
// glm::vec3 getColour() const
//      -> Returns the colour of the light.
// float getIntensity() const
//      -> Returns the intensity of the light.
// void update(shaders::Shader* shader) override
//      -> Updates the shader with the light's properties.
// bool _isDirectional
//      -> Indicates whether the light is directional.
// glm::vec3 _direction
//      -> Direction vector of the light.
// glm::vec3 _colour
//      -> Colour of the light.
// glm::vec3 _position
//      -> Position of the light.
// float _strength
//      -> Strength of the light. (Mainly used for point lights)
// float _intensity
//      -> Intensity of the light. (Mainly used for directional lights)
// float _size
//      -> Size of the light (for point lights).
// --------------------------------------------
//
// ============================================
//              GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"

#include "Scene/Element.h"
#include "Rendering/ShaderUtil.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace scene {
	class ENGINE_API Light : public Element
    {
    public:

        Light() {
            _direction = glm::vec3(-1.0f, -1.0f, -0.3f);
            _position = glm::vec3(-4.0f, 20.0f, 12.0f);
            _colour = glm::vec3(0.3f, 0.2f, 0.8f);
            _strength = 25.0f;
			_intensity = 10.0f;
            _size = 25.0f;
        }
        ~Light() {}

		// Default light configurations
        struct DirectionalLight {
            glm::vec3 direction{ -1.0, -0.3f, 0.2f };
            glm::vec3 colour{ 1.0f, 0.98f, 0.95f };
            float intensity = 10.0f;
        };

		// Point light configuration
        struct PointLight {
            glm::vec3 position{ 0.0f, 10.0f, 0.0f };
            glm::vec3 colour{ 1.0f, 0.98f, 0.95f };
            float intensity = 25.0f;
            float size = 15.0f;
        };

		void setDirection(const glm::vec3& dir) { _direction = glm::normalize(dir); }
        void setIntensity(const float intsy) { _intensity = intsy; }

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