#include "pch.h"
// File:   Paths.cpp
// GitHub: SaltyJoss
#include "Platform/Paths.h"
#ifdef _WIN32
	#include <windows.h>
		#ifdef _MSC_VER
		#pragma comment(lib, "Shell32.lib")
	#endif
	#include <shlobj.h>      // SHGetKnownFolderPath
	#include <combaseapi.h>  // CoTaskMemFree
#endif

namespace {
	std::filesystem::path g_root, g_assets, g_configs, g_logs, g_runs;

	// Get the directory of the currently executing module (executable)
	inline std::filesystem::path getExecDir() {
		wchar_t buf[MAX_PATH]{};
		GetModuleFileNameW(NULL, buf, MAX_PATH);
		return std::filesystem::path(buf).parent_path();
	}

	// Get the LocalAppData directory for the current user
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