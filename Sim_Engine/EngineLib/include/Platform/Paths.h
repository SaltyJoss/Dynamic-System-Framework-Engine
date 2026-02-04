#pragma once

#include <windows.h>
#include <string>
#include <filesystem>
#pragma comment(lib, "Shell32.lib")

namespace paths {
	void init();

	const std::filesystem::path& root();
	const std::filesystem::path& assets();
	const std::filesystem::path& configs();
	const std::filesystem::path& logs();
	const std::filesystem::path& runs();
}