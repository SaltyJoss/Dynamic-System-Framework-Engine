// DSFE_Engine Application.h
# pragma once

#include "GUIExports.h"

#include <memory>
#include <string>
#include <vector>
#include "Platform/Logger.h"

// forward declarations
//namespace window { class GLWindow; }
class QApplication;
namespace gui { class SimManager; }
namespace window { class DSFE_MainWindow; }

class DSFE_GUI_API Application {
public:
	Application(const std::string& name);
	~Application();

	static Application& Instance() { return *sInstance; }
	int run();

private:
	static Application* sInstance;

	std::string _name;

	int _qtArgc = 0;
	std::vector<std::string> _qtArgStorage;
	std::vector<char*> _qtArgv;

	std::unique_ptr<QApplication> _qtApp;
	std::unique_ptr<gui::SimManager> _sim;
	std::unique_ptr<window::DSFE_MainWindow> _mainW;

//#ifdef _MSC_VER
//#pragma warning(push)
//#pragma warning(disable: 4251) // Suppress C4251 for private members
//#endif
//	std::unique_ptr<window::GLWindow> _window;
//	std::unique_ptr<scene::Camera> _camera;
//#ifdef _MSC_VER
//#pragma warning(pop)
//#endif
};