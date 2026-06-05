// DSFE_GUI Application.cpp
#include "Application.h"

#include "MainWindow/DSFE_MainWindow.h"
#include "Scene/SimulationManager.h"
#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"

#include <QApplication>
#include <QCoreApplication>
#include <QGuiApplication>
#include <QScreen>
#include <QSurfaceFormat>

namespace fs = std::filesystem;
// Initialise the static instance pointer to nullptr
Application* Application::sInstance = nullptr;

// Constructor: Initialises paths, sets up data manager, and creates the main application window
Application::Application(const std::string& appName) : _name(appName) {
	paths::init();
	LOG_INFO("Root path: %s", paths::root().string().c_str());
	LOG_INFO("Assets path: %s", paths::assets().string().c_str());
	LOG_INFO("Configs path: %s", paths::configs().string().c_str());
	LOG_INFO("Logs path: %s", paths::logs().string().c_str());
	LOG_INFO("Runs path: %s", paths::runs().string().c_str());

	qputenv("QSG_RHI_BACKEND", "opengl");
#if defined(Q_OS_WIN)
	QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
#endif
	QCoreApplication::addLibraryPath("C:/Qt/6.11.1/msvc2022_64/plugins");

	_qtArgStorage.clear();
	_qtArgStorage.emplace_back(_name.empty() ? "DSFE" : _name);
	_qtArgc = static_cast<int>(_qtArgStorage.size());
	_qtArgv.clear();
	_qtArgv.reserve(_qtArgStorage.size());
	for (std::string& arg : _qtArgStorage) { _qtArgv.push_back(arg.data()); }

	QSurfaceFormat format;
	format.setProfile(QSurfaceFormat::CoreProfile);
	format.setVersion(4, 5);
	format.setDepthBufferSize(24);
	format.setStencilBufferSize(8);
	QSurfaceFormat::setDefaultFormat(format);

	_qtApp = std::make_unique<QApplication>(_qtArgc, _qtArgv.data());
	_sim = std::make_unique<gui::SimManager>();

	int winW = 1920, winH = 1080;
	if (QScreen* screen = QGuiApplication::primaryScreen()) {
		const QRect g = screen->geometry();
		winH = int(g.height() * 0.8f); // maintain 16:9 aspect ratio and fit within 90% of screen height
		winW = (winH * 16) / 9; // maintain 16:9 aspect ratio
	}

	_mainW = std::make_unique<window::DSFE_MainWindow>(_sim.get());
	_mainW->resize(winW, winH);
	_mainW->show();
}

Application::~Application() = default;

int Application::run() {
	return _qtApp ? _qtApp->exec() : 0;
}