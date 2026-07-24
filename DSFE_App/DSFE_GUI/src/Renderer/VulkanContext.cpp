#include "Renderer/VulkanContext.h"

#ifdef _WIN32
    #include <windows.h>
#endif

#include "EngineLib/LogMacros.h"


namespace renderer {
    // Vulkan debug callback function for validation layer messages
    static VKAPI_ATTR VkBool32 VKAPI_CALL debug_callback(
        VkDebugUtilsMessageSeverityFlagBitsEXT severity,
        VkDebugUtilsMessageTypeFlagsEXT,
        const VkDebugUtilsMessengerCallbackDataEXT* data,
        void*) {
        if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
            LOG_ERROR("[vulkan] %s", data->pMessage);
        } else if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
            LOG_WARN("[vulkan] %s", data->pMessage);
        }
        return VK_FALSE;
    }

    // Create a Vulkan instance with the specified application name and version, and enable required extensions and validation layers.
    bool VulkanContext::create_instance() {
        if (volkInitialize() != VK_SUCCESS) {
            LOG_ERROR("volkInitialize failed — is vulkan-1.dll present?");
            return false;
        }

        VkApplicationInfo app_info {
            .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
            .pApplicationName = "DSFE_GUI",
            .applicationVersion = VK_MAKE_VERSION(1, 0, 0),
            .pEngineName = "DSFE_Engine",
            .engineVersion = VK_MAKE_VERSION(1, 0, 0),
            .apiVersion = VK_VERSION
        };

        // Request required Vulkan extensions for surface creation and validation layers
        std::vector<const char*> extensions { VK_KHR_SURFACE_EXTENSION_NAME };

        // Platform-specific surface extensions for Vulkan instance creation
    #ifdef _WIN32
        extensions.push_back(VK_KHR_WIN32_SURFACE_EXTENSION_NAME);
    #elif defined(__linux__)
        extensions.push_back(VK_KHR_XCB_SURFACE_EXTENSION_NAME);
    #endif
        
        // Enable debug utils extension for validation layer support
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        // Enable validation layer for error handling
        std::vector<const char *> requested_layers { "VK_LAYER_KHRONOS_validation" };

        // Set up debug messenger create info for Vulkan validation layers
        VkDebugUtilsMessengerCreateInfoEXT debug_info {
            .sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT,
            .messageSeverity =
                VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
                VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
            .messageType =
                VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | 
                VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT,
            .pfnUserCallback = debug_callback
        };

        // Create the Vulkan instance with the specified application info, requested layers, and extensions (it takes 1000 lines to draw a triangle [;)
        VkInstanceCreateInfo create_info {
            .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
            .pNext = &debug_info,
            .pApplicationInfo = &app_info,
            .enabledLayerCount = static_cast<uint32_t>(requested_layers.size()),
            .ppEnabledLayerNames = requested_layers.data(),
            .enabledExtensionCount = static_cast<uint32_t>(extensions.size()),
            .ppEnabledExtensionNames = extensions.data()
        };

        VkResult result = vkCreateInstance(&create_info, nullptr, &_instance);

        if (result != VK_SUCCESS) {
            LOG_ERROR("Failed to create Vulkan instance: %d", result);
            return false;
        }

        volkLoadInstance(_instance); // Load Vulkan instance functions using volk
        return true;
        
    }

    // Create a Vulkan surface for rendering using the provided native window handle (SDL window in this case).
    bool VulkanContext::create_surface(const NativeWindow& win) {
    #ifdef _WIN32
        if (!win.handle) {
            LOG_ERROR("Native window handle is null; cannot create Vulkan surface.");
            return false;
        }

        VkWin32SurfaceCreateInfoKHR surface_info {
            .sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR,
            .hinstance = GetModuleHandle(nullptr),
            .hwnd = static_cast<HWND>(win.handle)
        };

        VkResult result = vkCreateWin32SurfaceKHR(_instance, &surface_info, nullptr, &_surface);
        if (result != VK_SUCCESS) {
            LOG_ERROR("Failed to create Vulkan Win32 surface: %d", result);
            return false;
        }
    #elif defined(__linux__)
        if (!win.connection) {
            LOG_ERROR("XCB connection handle is null; cannot create Vulkan surface (is Qt on xcb?).");
            return false;
        }
        VkXcbSurfaceCreateInfoKHR surface_info {
            .sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR,
            .connection = static_cast<xcb_connection_t*>(win.connection),
            .window = static_cast<xcb_window_t>(reinterpret_cast<uintptr_t>(win.handle))
        };
        VkResult result = vkCreateXcbSurfaceKHR(_instance, &surface_info, nullptr, &_surface);
        if (result != VK_SUCCESS) {
            LOG_ERROR("Failed to create Vulkan XCB surface: %d", result);
            return false;
        }
    #endif

        return true;
    }

    VkPhysicalDevice VulkanContext::find_physical_device() {
        uint32_t device_count = 0;
        vkEnumeratePhysicalDevices(_instance, &device_count, nullptr);
        if (device_count == 0) {
            LOG_ERROR("Failed to find any Vulkan physical devices.");
            return VK_NULL_HANDLE;
        }

        std::vector<VkPhysicalDevice> devices(device_count);
        vkEnumeratePhysicalDevices(_instance, &device_count, devices.data());

        for (VkPhysicalDevice device : devices) {
            VkPhysicalDeviceProperties props{};
            vkGetPhysicalDeviceProperties(device, &props);
            LOG_INFO("Found Vulkan device: %s", props.deviceName);
        }

        return devices.front();
    }

    // Find a graphics queue family that supports both graphics and present operations for the given physical device and surface.
    bool VulkanContext::find_graphics_queue() {
        uint32_t count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(_phys_device, &count, nullptr);
        std::vector<VkQueueFamilyProperties> families(count);
        vkGetPhysicalDeviceQueueFamilyProperties(_phys_device, &count, families.data());

        for (uint32_t i = 0; i < count; ++i) {
            if (!(families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) { continue; }
            VkBool32 present = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(_phys_device, i, _surface, &present);
            if (present) { _gfx_queue_fam_idx = i; return true; }
        }
        LOG_ERROR("No graphics queue family with present support");
        return false;
    }

    // Create a Vulkan logical device from the specified physical device, enabling required features and extensions.`
    bool VulkanContext::create_device(VkPhysicalDevice phys) {
        const float priority = 1.0f;
        VkDeviceQueueCreateInfo queue_info{
            .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
            .queueFamilyIndex = _gfx_queue_fam_idx,
            .queueCount = 1,
            .pQueuePriorities = &priority
        };
        VkPhysicalDeviceVulkan13Features f13{
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES,
            .shaderDemoteToHelperInvocation = VK_TRUE,
            .synchronization2 = VK_TRUE,
            .dynamicRendering = VK_TRUE
        };
        VkPhysicalDeviceVulkan12Features f12{
            .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES,
            .pNext = &f13,
            .timelineSemaphore = VK_TRUE
        };

        const char* device_extensions[]{ VK_KHR_SWAPCHAIN_EXTENSION_NAME };

        VkDeviceCreateInfo info{
            .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
            .pNext = &f12,
            .queueCreateInfoCount = 1,
            .pQueueCreateInfos = &queue_info,
            .enabledExtensionCount = 1,
            .ppEnabledExtensionNames = device_extensions
        };

        VkResult result = vkCreateDevice(phys, &info, nullptr, &_device);
        if (result != VK_SUCCESS) { LOG_ERROR("vkCreateDevice failed: %d", result); return false; }

        volkLoadDevice(_device);   // not optional — device-level pointers come from here
        vkGetDeviceQueue(_device, _gfx_queue_fam_idx, 0, &_gfx_queue);
        return true;
    }

    bool VulkanContext::initialise_vma() {
        // VMA needs pointers explicitly under VK_NO_PROTOTYPES; volk has them already.
        VmaVulkanFunctions fns{};
        
        fns.vkGetInstanceProcAddr = vkGetInstanceProcAddr;
        fns.vkGetDeviceProcAddr   = vkGetDeviceProcAddr;

        VmaAllocatorCreateInfo info{
            .physicalDevice = _phys_device,
            .device = _device,
            .pVulkanFunctions = &fns,
            .instance = _instance,
            .vulkanApiVersion = VK_VERSION
        };
        VkResult result = vmaCreateAllocator(&info, &_allocator);
        if (result != VK_SUCCESS) {
            LOG_ERROR("vmaCreateAllocator failed: %d", result);
            return false;
        }
        LOG_INFO("initialise_vma: allocator=%p", (void*)_allocator);
        return true;
    }

    // Initialise the Vulkan context by creating the Vulkan instance, surface, physical device, logical device, and VMA allocator.
    bool VulkanContext::init(const NativeWindow& win) {
        if (!create_instance()) { return false; }
        if (!create_surface(win)) { return false; }
        _phys_device = find_physical_device();
        if (_phys_device == VK_NULL_HANDLE) { return false; }
        if (!find_graphics_queue()) { return false; }
        if (!create_device(_phys_device)) { return false; }
        if (!initialise_vma()) { return false; }
        return true;
    }
    // Shutdown the Vulkan context by destroying the VMA allocator, logical device, surface, and instance.
    void VulkanContext::shutdown() {
        if (_allocator) { vmaDestroyAllocator(_allocator); _allocator = nullptr; }
        if (_device) { vkDestroyDevice(_device, nullptr); _device = VK_NULL_HANDLE; }
        if (_surface) { vkDestroySurfaceKHR(_instance, _surface, nullptr); _surface = VK_NULL_HANDLE; }
        if (_instance) { vkDestroyInstance(_instance, nullptr); _instance = VK_NULL_HANDLE; }
    }


} // namespace renderer