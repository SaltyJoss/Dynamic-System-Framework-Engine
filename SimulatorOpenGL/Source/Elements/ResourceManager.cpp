#define STB_IMAGE_IMPLEMENTATION
#include "ResourceManager.h"

std::unordered_map<std::string, GLuint> ResourceManager::textures;

void ResourceManager::Init() {}

void ResourceManager::LoadTexture(const std::string& name, const std::string& path) {
    int w, h, channels;
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &channels, 4);
    if (!data) return;

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    stbi_image_free(data);

    textures[name] = tex;
}

ImTextureID ResourceManager::GetTexture(const std::string& name) {
    return (ImTextureID)(intptr_t)textures[name];
}

void ResourceManager::CleanUp() {
    for (auto it = textures.begin(); it != textures.end(); ++it)
        glDeleteTextures(1, &it->second);
    textures.clear();
}