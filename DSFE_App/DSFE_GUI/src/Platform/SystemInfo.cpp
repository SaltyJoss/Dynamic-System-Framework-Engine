#include "Platform/SystemInfo.h"
#include "EngineLib/LogMacros.h"

#include <volk.h>

#include <QSysInfo>
#include <QStorageInfo>
#include <QCoreApplication>

#include <thread>
#include <vector>

#ifdef _WIN32
    #include <intrin.h>
    #include <windows.h>
#else
    #include <fstream>
    #include <sys/sysinfo.h>
#endif

namespace gui {
    static QString cpuBrand() {
#ifdef _WIN32
        int cpuInfo[4] = { -1 };
        char brand[0x40] = { 0 };
        __cpuid(cpuInfo, 0x80000000);
        const unsigned max_ext = static_cast<unsigned>(cpuInfo[0]);
        if (max_ext >= 0x80000002) {
            __cpuid(reinterpret_cast<int*>(cpuInfo), 0x80000002);
            __cpuid(reinterpret_cast<int*>(cpuInfo), 0x80000003);
            __cpuid(reinterpret_cast<int*>(cpuInfo), 0x80000004);
            return QString::fromLatin1(brand).trimmed();
        }
        return QString("Unknown CPU");
#else
        std::ifstream cpuinfo("/proc/cpuinfo");
        std::string line;
        while (std::getline(cpuinfo, line)) {
            if (line.find("model name") != std::string::npos) {
                auto pos = line.find(':');
                if (pos != std::string::npos) {
                    return QString::fromStdString(line.substr(pos + 2)).trimmed();
                }
            }
        }
        return QString("Unknown CPU");
#endif
    }

    static QString ramSize() {
#ifdef _WIN32
        MEMORYSTATUSEX s{};
        s.dwLength = sizeof(s);
        if (GlobalMemoryStatusEx(&s)) {
            return QString("%1 GB").arg(s.ullTotalPhys / (1024 * 1024 * 1024));
        }
        return QString("Unknown");
#else
        struct sysinfo si{};
        if (sysinfo(&si) == 0) {
            const double gb = (double)si.totalram * si.mem_unit / (1024.0 * 1024.0 * 1024.0);
            return QString("%1 GB").arg(gb, 0, 'f', 2);
        }
        return QString("Unknown");
#endif
    }

    static QString storageSize() {
        QStorageInfo s(QCoreApplication::applicationDirPath());
        if (!s.isValid() || !s.isReady()) { return QString("Unknown"); }
        qint64 free_GB = s.bytesAvailable() / (1024 * 1024 * 1024);
        qint64 total_GB = s.bytesTotal() / (1024 * 1024 * 1024);
        return QString("%1 / %2").arg(free_GB / (1024 * 1024 * 1024)).arg(total_GB / (1024 * 1024 * 1024));
    }

    static void queryGpus(SystemInfo& si) {
        // Minimal throwaway instance just to enumerate devices. volk must be initialised
        static bool volkReady = false;
        if (!volkReady) {
            if (volkInitialize() != VK_SUCCESS) { si.gpus << "Vulkan unavailable"; return; }
            volkReady = true;
        }
        VkApplicationInfo app{ .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO, .apiVersion = VK_API_VERSION_1_3 };
        VkInstanceCreateInfo ci{ .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO, .pApplicationInfo = &app };
        VkInstance instance = VK_NULL_HANDLE;
        if (vkCreateInstance(&ci, nullptr, &instance) != VK_SUCCESS) { si.gpus << "Vulkan unavailable"; return; }
        volkLoadInstance(instance);

        uint32_t count = 0;
        vkEnumeratePhysicalDevices(instance, &count, nullptr);
        std::vector<VkPhysicalDevice> devices(count);
        vkEnumeratePhysicalDevices(instance, &count, devices.data());

        uint32_t apiVer = 0;
        for (VkPhysicalDevice d : devices) {
            VkPhysicalDeviceProperties props{};
            vkGetPhysicalDeviceProperties(d, &props);
            if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_CPU) { continue; }
            si.gpus << QString::fromLatin1(props.deviceName);
            if (props.apiVersion > apiVer) { apiVer = props.apiVersion; }
        }
        if (si.gpus.isEmpty()) { si.gpus << "No hardware GPU"; }
        si.vulkanVersion = QString("%1.%2.%3").arg(VK_API_VERSION_MAJOR(apiVer)).arg(VK_API_VERSION_MINOR(apiVer)).arg(VK_API_VERSION_PATCH(apiVer));
        vkDestroyInstance(instance, nullptr);
    }

    SystemInfo SystemInfo::query() {
        SystemInfo si;
        si.os = QSysInfo::prettyProductName();
        si.processor = cpuBrand();
        si.ram = ramSize();
        si.storage = storageSize();
        queryGpus(si);
        return si;
    }
} // namespace gui