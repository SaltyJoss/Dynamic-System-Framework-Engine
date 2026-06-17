// DSFE_Core Paths.cpp
#include "pch.h"
#include "Platform/Paths.h"
// Windows-specific includes for known folder paths
#ifdef _WIN32
	#define NOMINMAX
	#include <windows.h>
	#ifdef _MSC_VER
		#pragma comment(lib, "Shell32.lib")
	#endif
	#include <shlobj.h>      // SHGetKnownFolderPath
	#include <combaseapi.h>  // CoTaskMemFree
#endif
// Linux-specific includes for known folder paths
#ifdef __linux__
	#include <unistd.h>
	#include <limits.h>
#endif

namespace {
	std::filesystem::path g_root, g_assets, g_configs, g_logs, g_runs;

	// Get the directory of the currently executing module (executable)
	inline std::filesystem::path getExecDir() {
	#ifdef _WIN32
		wchar_t buf[MAX_PATH]{};
		GetModuleFileNameW(NULL, buf, MAX_PATH);
		return std::filesystem::path(buf).parent_path();
	#elif defined(__linux__)
		char buf[PATH_MAX]{};
		ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
		if (len != -1) {
			buf[len] = '\0';
			return std::filesystem::path(buf).parent_path();
		}
		return std::filesystem::current_path();
	#else
		return std::filesystem::current_path(); // Fallback for other platforms
	#endif
	}


	// Get the LocalAppData directory for the current user
	inline std::filesystem::path localAppDataDir() {
	#ifdef _WIN32
		PWSTR raw = nullptr;
		if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, nullptr, &raw))) {
			std::filesystem::path p(raw);
			CoTaskMemFree(raw);
			return p;
		}
		return {};
	#else
		// Linux equivalent: use $XDG_DATA_HOME or fallback to ~/.local/share
		if (const char* xdg = std::getenv("XDG_DATA_HOME")) {
			return std::filesystem::path(xdg);
		}
		if (const char* home = std::getenv("HOME")) {
			return std::filesystem::path(home) / ".local" / "share";
		}
		return std::filesystem::current_path(); // Fallback to current path if all else fails
	#endif
	}
}

namespace paths {
	void init() {
		// Get executable directory
		g_root	  = getExecDir();

		// Set standard subdirectories
		g_assets  = g_root/ "assets";
		g_configs = g_root/ "configs";

		// Use LocalAppData for logs and runs
		auto localAppData = localAppDataDir() / "DSFE";
		g_logs	  = localAppData / "logs";
		g_runs	  = localAppData / "runs";

		// Ensure directories exist
		std::filesystem::create_directories(g_logs);
		std::filesystem::create_directories(g_runs);
	}

	// Accessors
	const std::filesystem::path& root()    { return g_root; }		// Executable directory
	const std::filesystem::path& assets()  { return g_assets; }		// Assets directory (relative to executable)
	const std::filesystem::path& configs() { return g_configs; }	// Configs directory (relative to executable)
	const std::filesystem::path& logs()    { return g_logs; }		// Logs directory (in LocalAppData)
	const std::filesystem::path& runs()    { return g_runs; }		// Runs directory (in LocalAppData)
}