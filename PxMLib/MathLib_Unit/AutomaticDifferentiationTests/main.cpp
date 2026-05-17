// PxMLib/MathLib_Unit DualNumberTests.cpp
#include "TestHarness.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

int main() {
#ifdef _WIN32
	// Enable ANSI escape codes on Windows 10+
	HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	DWORD mode = 0;
	if (GetConsoleMode(hOut, &mode)) {
		SetConsoleMode(hOut, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
	}
#endif

	std::cout << "========================================\n";
	std::cout << "  Automatic Differentation Unit Tests\n";
	std::cout << "========================================\n";

	int failures = test::runAll();

	std::cout << "\nPress Enter to exit...";
	std::cin.get();

	return failures;
}
