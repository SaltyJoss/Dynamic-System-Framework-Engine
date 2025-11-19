#include "pch.h"
#include "Rendering/Texture.h"

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <stb/stb_image.h>
#include <iostream>

#include "EngineLib/LogMacros.h"

namespace render {
	Texture::~Texture() {
		if (ID != 0) {
			glDeleteTextures(1, &ID);
			LOG_INFO("Texture ID %d deleted in destructor", ID);
		}
	}

	void Texture::bind(GLuint unit) const {
		glActiveTexture(GL_TEXTURE0 + unit);
		glBindTexture(GL_TEXTURE_2D, ID);
	}

	GLuint Texture::load(const std::string& path, bool sRGB) {
		int width, height, channels;

		stbi_set_flip_vertically_on_load(true);
		unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 0);

		if (!data) {
			LOG_ERROR("Failed to load texture from %s", path.c_str());
			return 0;
		}

		GLuint texID;
		glGenTextures(1, &texID);
		glBindTexture(GL_TEXTURE_2D, texID);

		GLenum internalFormat;
		GLenum dataFormat;

		if (channels == 1) {
			internalFormat = GL_RED;
			dataFormat = GL_RED;
		} else if (channels == 3) {
			internalFormat = sRGB ? GL_SRGB : GL_RGB;
			dataFormat = GL_RGB;
		} else if (channels == 4) {
			internalFormat = sRGB ? GL_SRGB_ALPHA : GL_RGBA;
			dataFormat = GL_RGBA;
		} else {
			LOG_ERROR("Unsupported number of channels (%d) in texture %s", channels, path.c_str());
			stbi_image_free(data);
			return 0;
		}

		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, dataFormat, GL_UNSIGNED_BYTE, data);

		glGenerateMipmap(GL_TEXTURE_2D);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);
		LOG_INFO("Texture loaded from %s (ID: %d, Size: %dx%d, Channels: %d)", path.c_str(), texID, width, height, channels);
		
		return texID;
	}
}