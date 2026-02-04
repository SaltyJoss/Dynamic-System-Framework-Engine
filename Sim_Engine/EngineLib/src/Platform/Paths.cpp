#include "pch.h"
#include "Platform/Paths.h"
#ifdef _WIN32
	#include <windows.h>
		#ifdef _MSC_VER
		#pragma comment(lib, "Shell32.lib")
	#endif
#endif
#include <shlobj.h>      // SHGetKnownFolderPath
#include <combaseapi.h>  // CoTaskMemFree

namespace {
	std::filesystem::path g_root, g_assets, g_configs, g_logs, g_runs;

	inline std::filesystem::path getExecDir() {
		wchar_t buf[MAX_PATH]{};
		GetModuleFileNameW(NULL, buf, MAX_PATH);
		return std::filesystem::path(buf).parent_path();
	}

	inline std::filesystem::path localAppDataDir() {
		PWSTR raw = nullptr;
		if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, nullptr, &raw))) {
			std::filesystem::path p(raw);
			CoTaskMemFree(raw);
			return p;
		}
		return {};
	}
}

namespace paths {
	void init() {
		// Get executable directory
		g_root	  = getExecDir();
		// Set standard subdirectories
		g_assets  = g_root / "assets";
		g_configs = g_root / "configs";
		// Use LocalAppData for logs and runs
		auto localAppData = localAppDataDir() / "DSFE";
		g_logs	  = localAppData / "logs";
		g_runs	  = localAppData / "runs";
		// Ensure directories exist
		std::filesystem::create_directories(g_logs);
		std::filesystem::create_directories(g_runs);
	}

	// Accessors
	const std::filesystem::path& root()    { return g_root; }
	const std::filesystem::path& assets()  { return g_assets; }
	const std::filesystem::path& configs() { return g_configs; }
	const std::filesystem::path& logs()    { return g_logs; }
	const std::filesystem::path& runs()    { return g_runs; }
}