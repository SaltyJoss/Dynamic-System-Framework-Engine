#pragma once

#include <volk.h>

namespace renderer {
    bool initialise_vulkan_loader();
    void shutdown_vulkan_loader();
} // namespace renderer