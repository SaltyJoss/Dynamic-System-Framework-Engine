
// =============================================
//            File: Application.h
// =============================================
// Main Application class for initializing and running the engine.
// Summary:
// =============================================
//
// public:
// --------------------------------------------
// Application(const std::string& name)
//      -> Constructor that initializes the application with a given name.
// ~Application()
//      -> Destructor that cleans up resources.
// static Application& Instance()
//      -> Returns a reference to the singleton instance of the application.
// void run()
//      -> Runs the main application loop.
// --------------------------------------------
//
// private:
// --------------------------------------------
// static Application* sInstance
//      -> Static pointer to the singleton instance of the application.
// std::unique_ptr<window::GLWindow> _window
//      -> Unique pointer to the OpenGL window.
// std::unique_ptr<scene::Camera> _camera
//      -> Unique pointer to the main camera.
// --------------------------------------------
//
// ============================================
//              GitHub: SaltyJoss
// ============================================


#include "EngineCore.h"

#include <memory>
#include <string>

// forward declarations
namespace window { class GLWindow; }
namespace scene { class Camera; }

class ENGINE_API Application {
public:
	Application(const std::string& name);
	~Application();

	static Application& Instance() { return *sInstance; }
	void run();

private:
	static Application* sInstance;

	std::unique_ptr<window::GLWindow> _window;
	std::unique_ptr<scene::Camera> _camera;
};