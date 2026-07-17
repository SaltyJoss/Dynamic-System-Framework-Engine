#include "Renderer/VulkanLoader.h"
#define VOLK_IMPLEMENTATION
#include <volk.h>

namespace renderer {
    bool initialise_vulkan_loader() { return volkInitialize() == VK_SUCCESS; }
    void shutdown_vulkan_loader() { volkFinalize(); }
} // namespace renderer