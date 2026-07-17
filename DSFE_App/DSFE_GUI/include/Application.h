// DSFE_Engine Application.h
#pragma once

#include "GUIExports.h"

#include <memory>
#include <string>
#include <vector>
#include "Platform/Logger.h"

// forward declarations
//namespace window { class GLWindow; }
class QApplication;
namespace gui { class SimulationManager; }
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
	std::unique_ptr<gui::SimulationManager> _sim;
	std::unique_ptr<window::DSFE_MainWindow> _mainW;
};