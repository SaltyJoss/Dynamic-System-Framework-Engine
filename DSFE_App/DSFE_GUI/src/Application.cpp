// DSFE_GUI Application.cpp
#include "Application.h"

#include "Platform/WindowManager.h"
#include "Scene/Camera.h"

#ifdef __gl_h_
#undef __gl_h_
#endif
#include <GLFW/glfw3.h>

#include <filesystem>
#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"

#include <QCoreApplication>
#include <qapplication.h>
#include <qmainwindow.h>

namespace fs = std::filesystem;
// Initialise the static instance pointer to nullptr
Application* Application::sInstance = nullptr;

static QApplication* gQtApp = nullptr;
static QMainWindow* gQtWindow = nullptr;

// Constructor: Initialises paths, sets up data manager, and creates the main application window
Application::Application(const std::string& appName) {
	paths::init();

	LOG_INFO("Root path: %s", paths::root().string().c_str());
	LOG_INFO("Assets path: %s", paths::assets().string().c_str());
	LOG_INFO("Configs path: %s", paths::configs().string().c_str());
	LOG_INFO("Logs path: %s", paths::logs().string().c_str());
	LOG_INFO("Runs path: %s", paths::runs().string().c_str());

	int winW = 1920, winH = 1080;

	if (glfwInit()) {
		const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		if (mode) {
			winH = (int)(mode->height * 0.8f);
			winW = (winH * 16) / 9;
			// Clamp width to 90% of monitor width in case ultra-wide
			if (winW > (int)(mode->width * 0.9f)) {
				winW = (int)(mode->width * 0.9f);
				winH = (winW * 9) / 16;
			}
			LOG_INFO("Monitor: %dx%d -> Window: %dx%d (16:9)", mode->width, mode->height, winW, winH);
		}
		glfwTerminate(); // OpenGLContext::init will call glfwInit again
	}

	if (!gQtApp) {
		QCoreApplication::addLibraryPath("C:/Qt/6.11.1/msvc2022_64/plugins");
		int argc = 0;
		gQtApp = new QApplication(argc, nullptr);
		gQtWindow = new QMainWindow();
		gQtWindow->setWindowTitle(QString::fromStdString(appName));
		gQtWindow->resize(800, 600);
		gQtWindow->show();
		LOG_INFO("Qt application and main window created with title '%s'", appName.c_str());
	}

	_window = std::make_unique<window::GLWindow>();
	_window->init(winW, winH, appName);
}

Application::~Application() = default;

// Main application loop: Continues running until the window signals to close, updating and rendering each frame
void Application::run() {
    while (_window->isRunning() && !_window->shouldClose()) {
		QCoreApplication::processEvents();

        _window->update();
        _window->render();
    }
}