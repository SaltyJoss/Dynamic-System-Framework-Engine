# pragma once
// File:   Application.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include <memory>
#include <string>

// forward declarations
namespace window { class ENGINE_API GLWindow; }
namespace scene { class ENGINE_API Camera; }

class ENGINE_API Application {
public:
	Application(const std::string& name);
	~Application();

	static Application& Instance() { return *sInstance; }
	void run();

private:
	static Application* sInstance;

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251) // Suppress C4251 for private members
#endif
	std::unique_ptr<window::GLWindow> _window;
	std::unique_ptr<scene::Camera> _camera;
#ifdef _MSC_VER
#pragma warning(pop)
#endif
};