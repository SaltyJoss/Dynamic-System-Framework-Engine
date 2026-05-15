#pragma once
// File:   TestHarness.h
// GitHub: SaltyJoss
// Pretty basic console test runner for integration tests
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace test {
	// ANSI colour codes for console output
    inline constexpr const char* GREEN  = "\033[32m";
    inline constexpr const char* RED    = "\033[31m";
    inline constexpr const char* YELLOW = "\033[33m";
    inline constexpr const char* CYAN   = "\033[36m";
    inline constexpr const char* RESET  = "\033[0m";

	// assertTrue checks a condition and throws a runtime_error with the given message if it fails.
    inline void assertTrue(bool condition, const char* msg) {
        if (!condition) { throw std::runtime_error(msg); }
    }

	// TestCase represents a single test with its group, name, and function to execute.
    struct TestCase {
        std::string group;
        std::string name;
        std::function<void()> fn;
    };

	// registry() returns a reference to the static vector of registered test cases.
    inline std::vector<TestCase>& registry() {
        static std::vector<TestCase> tests;
        return tests;
    }

	// AutoRegister is a helper that registers a test case at static initialization time.
    struct AutoRegister {
        AutoRegister(const char* group, const char* name, std::function<void()> fn) {
            registry().push_back({ group, name, std::move(fn) });
        }
    };

	// Runs all registered tests, printing results and timing. Returns number of failed tests.
    inline int runAll() {
        int passed = 0, failed = 0;
        std::string lastGroup;
		// Iterate through all registered test cases, grouped by their group name.
        for (const auto& tc : registry()) {
            if (tc.group != lastGroup) {
                std::cout << "\n" << CYAN << "=== " << tc.group << " ===" << RESET << "\n";
                lastGroup = tc.group;
            }
			// Time the test execution and catch any exceptions to report failures.
            auto t0 = std::chrono::high_resolution_clock::now();
            try {
                tc.fn();
                auto t1 = std::chrono::high_resolution_clock::now();
                double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                std::cout << "  " << GREEN << "PASS" << RESET
                << "  " << tc.name
                << "  (" << static_cast<int>(ms) << " ms)\n";
                ++passed;
            }
			// If an exception is thrown, catch it and report as a failure with the error message.
            catch (const std::exception& e) {
                auto t1 = std::chrono::high_resolution_clock::now();
                double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                std::cout << "  " << RED << "FAIL" << RESET
                << "  " << tc.name
                << "  (" << static_cast<int>(ms) << " ms)"
                << "  -> " << e.what() << "\n";
                ++failed;
            }
        }

		// Print a summary of the test results, showing total, passed, and failed counts with color coding.
        std::cout << "\n" << std::string(50, '-') << "\n";
        std::cout << "Total: " << (passed + failed)
        << "   " << GREEN << "Passed: " << passed << RESET
        << "   " << (failed ? RED : GREEN) << "Failed: " << failed << RESET << "\n";

        return failed;
    }
}

// TEST(group, name) registers a free function as a test case.
#define TEST(groupName, testName)                                            \
    static void testName##_impl();                                           \
    namespace { static test::AutoRegister _reg_##testName(                   \
        groupName, #testName, testName##_impl); }                            \
    static void testName##_impl()

#define ASSERT_TRUE(cond, msg)  test::assertTrue((cond), (msg))