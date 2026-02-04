#include "pch.h"
#include "Application.h"
#include "Platform/WindowManager.h"
#include "Platform/Paths.h"
#include "Scene/Camera.h"
#include <filesystem>
#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

namespace fs = std::filesystem;
Application* Application::sInstance = nullptr;

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

Application::~Application() = default;

void Application::run() {
    while (_window->isRunning() && !_window->shouldClose()) {

        // --- Frame steps ---
        _window->update();                             // Scene updates, movement
        _window->render();                             // Draw everything
    }
}