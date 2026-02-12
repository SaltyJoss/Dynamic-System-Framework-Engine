#include "pch.h"
// File:   SkyboxRenderer.cpp
// GitHub: SaltyJoss
#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>

#include "Rendering/SkyboxRenderer.h"
#include "Rendering/CubeVertices.h"

namespace render{
	// Static cube vertices for skybox rendering (36 vertices for 12 triangles)
	SkyboxRenderer::SkyboxRenderer() : _VAO(0), _VBO(0), _envCubemap(0) {
        shader.load(
            "Engine/assets/shaders/skybox_render.vert.glsl",
            "Engine/assets/shaders/skybox_render.frag.glsl"
        );

        shader.use();
        shader.setInt1(0, "environmentMap");

        glGenVertexArrays(1, &_VAO);
        glGenBuffers(1, &_VBO);

        glBindVertexArray(_VAO);
        glBindBuffer(GL_ARRAY_BUFFER, _VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(CUBE_VERTICES), CUBE_VERTICES, GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

        glBindVertexArray(0);
    }

	// Clean up OpenGL resources
    SkyboxRenderer::~SkyboxRenderer() {
        glDeleteVertexArrays(1, &_VAO);
        glDeleteBuffers(1, &_VBO);
    }

	// Render the skybox using the provided projection and view matrices
    void SkyboxRenderer::render(const glm::mat4 & projection, const glm::mat4 & view) {
        glDepthMask(GL_FALSE);
        glDepthFunc(GL_LEQUAL);

        shader.use();
        shader.setMat4(glm::mat4(glm::mat3(view)), "view");
        shader.setMat4(projection, "projection");

		// Bind the environment cubemap texture to texture unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, _envCubemap);

		// Render the cube representing the skybox
        glBindVertexArray(_VAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);

		// Restore depth state
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
    }

	// Set the environment cubemap texture to be used for skybox rendering
    void SkyboxRenderer::setEnvironmentTexture(GLuint tex) { _envCubemap = tex; }
}