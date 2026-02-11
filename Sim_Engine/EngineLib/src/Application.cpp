#include "pch.h"
// File:   Application.cpp
// GitHub: SaltyJoss
#include "Application.h"
#include "Platform/WindowManager.h"
#include "Platform/Paths.h"
#include "Scene/Camera.h"
#include <filesystem>
#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

namespace fs = std::filesystem;
// Initialize the static instance pointer to nullptr
Application* Application::sInstance = nullptr;

// Constructor: Initializes paths, sets up data manager, and creates the main application window
Application::Application(const std::string& appName) {
	paths::init();
	data::DataManager::instance().setParentFolder(paths::runs().string());

	LOG_INFO("Root path: %s", paths::root().string().c_str());
	LOG_INFO("Assets path: %s", paths::assets().string().c_str());
	LOG_INFO("Configs path: %s", paths::configs().string().c_str());
	LOG_INFO("Logs path: %s", paths::logs().string().c_str());
	LOG_INFO("Runs path: %s", paths::runs().string().c_str());

	_window = std::make_unique<window::GLWindow>();
	_window->init(1920, 1080, appName);
}

// Destructor: Defaulted since we're using smart pointers for resource management
Application::~Application() = default;

// Main application loop: Continues running until the window signals to close, updating and rendering each frame
void Application::run() {
    while (_window->isRunning() && !_window->shouldClose()) {

		// Poll for and process events (keyboard, mouse, window events, etc.)
		// This should be called before any input handling to ensure we have the latest events
        _window->update();                             // Scene updates, movement
        _window->render();                             // Draw everything
    }
}