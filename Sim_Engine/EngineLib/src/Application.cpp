#include "pch.h"
#include "Application.h"

#include "Platform/WindowManager.h"
#include "Scene/Camera.h"
#include "Platform/Window.h"
#include <filesystem>
#include "EngineLib/LogMacros.h"

namespace fs = std::filesystem;
Application* Application::sInstance = nullptr;

static void ensureWorkingDir() {
    namespace fs = std::filesystem;
    auto root = fs::path(__FILE__).parent_path().parent_path().parent_path();
    fs::current_path(root);
    LOG_INFO("Working directory set to: %s", fs::current_path().string().c_str());
}

Application::Application(const std::string& appName) {
    ensureWorkingDir();
    LOG_INFO("Working directory set to: %s", fs::current_path().string().c_str());

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