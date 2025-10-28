#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include "CoreIncludes.h"
#include <stb_image.h>

class ResourceManager {
public:
    static void Init();
    static ImTextureID GetTexture(const std::string& name);
    static void LoadTexture(const std::string& name, const std::string& path);
    static void CleanUp();

private:
    static std::unordered_map<std::string, GLuint> textures;
};

#endif