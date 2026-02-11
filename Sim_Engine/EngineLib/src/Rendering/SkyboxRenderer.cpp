#include "pch.h"
#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>

#include "Rendering/SkyboxRenderer.h"
#include "Rendering/CubeVertices.h"

namespace render{
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

    SkyboxRenderer::~SkyboxRenderer()
    {
        glDeleteVertexArrays(1, &_VAO);
        glDeleteBuffers(1, &_VBO);
    }

    void SkyboxRenderer::render(const glm::mat4 & projection,
        const glm::mat4 & view)
    {
        glDepthMask(GL_FALSE);
        glDepthFunc(GL_LEQUAL);

        shader.use();
        shader.setMat4(glm::mat4(glm::mat3(view)), "view");
        shader.setMat4(projection, "projection");

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, _envCubemap);

        glBindVertexArray(_VAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);

        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
    }

    void SkyboxRenderer::setEnvironmentTexture(GLuint tex)
    {
        _envCubemap = tex;
    }
}