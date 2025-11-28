#pragma once

// =============================================
//            File: SkyboxRenderer.h
// =============================================
// Class for rendering a skybox in a 3D scene.
//
// Summary:
// =============================================
// 
// public:
// --------------------------------------------
// SkyboxRenderer()
//      -> Constructor for the SkyboxRenderer class.
// ~SkyboxRenderer()
//      -> Destructor for the SkyboxRenderer class.
// void render(const glm::mat4& projection, const glm::mat4& view)
//      -> Renders the skybox using the provided projection and view matrices.
// void setEnvironmentTexture(GLuint texture)
//      -> Sets the environment cubemap texture for the skybox.
// --------------------------------------------
//
// private:
// --------------------------------------------
// GLuint _VAO, _VBO
//      -> OpenGL Vertex Array Object and Vertex Buffer Object for the skybox geometry.
// GLuint _envCubemap
//      -> OpenGL texture ID for the environment cubemap.
// shaders::Shader shader
//      -> Shader program used for rendering the skybox.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include <glad/glad.h>
#include <glm/glm.hpp>
#include "Rendering/ShaderUtil.h"

namespace render {
    class SkyboxRenderer {
    public:
        SkyboxRenderer();
        ~SkyboxRenderer();

        void render(const glm::mat4& projection, const glm::mat4& view);
        void setEnvironmentTexture(GLuint texture);

    private:
        GLuint _VAO, _VBO;
		GLuint _envCubemap;
        shaders::Shader shader;
    };
}


