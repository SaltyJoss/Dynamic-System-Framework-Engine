// DSFE_GUI SkyboxRenderer.h
#pragma once

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
} // namespace render


