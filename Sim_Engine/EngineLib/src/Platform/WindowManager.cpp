#include "pch.h"
// File:   WindowManager.cpp
// GitHub: SaltyJoss
#ifdef __gl_h_
#undef __gl_h_
#endif
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Platform/Window.h"

#include "Rendering/GUIContext.h"
#include "Rendering/OpenGLContext.h"
#include "Rendering/OpenGLBufferManager.h"
#include "Rendering/ShaderUtil.h"

#include "Scene/Camera.h"
#include "Scene/Light.h"
#include "Scene/Input.h"
#include "Scene/Mesh.h"

#include "Scene/SimulationManager.h"
#include "ui/DebugPanel.h"
#include "ui/ControlPanel.h"
#include "ui/CommandScriptEditor.h"

#include "Platform/WindowManager.h"

#include "EngineLib/LogMacros.h"

bool controlPanelOpen = false;

namespace window {
	// Constructor
    GLWindow::GLWindow() { _header = new std::string(); }
    // Destructor
    GLWindow::~GLWindow() { 
        _renderCntx->end();
        _GUICntx->end();
        
        if (_window) { glfwDestroyWindow(_window); }

        glfwTerminate();
        delete _header;

        LOG_INFO("GLWindow destroyed, rendering and GUI contexts ended");
    }

    void GLWindow::render() {
        _renderCntx->preRender();
        _GUICntx->preRender();

        if (_sim)           _sim->render();
        if (_controlPanel)  _controlPanel->render(_sim.get());
		if (_debugPanel)    _debugPanel->render();
		if (_cmdEditor)     _cmdEditor->render();


		// Menu Callback
        _GUICntx->setMenuCallback([this]() {

            ImGuiStyle& style = ImGui::GetStyle();
            // Vertical spacing between menu items
            style.ItemSpacing = ImVec2(10.0f, 6.0f);
            // Padding inside menus
			style.WindowPadding = ImVec2(12.0f, 12.0f); // (left/right, top/bottom)
            // Padding inside each menu item
            style.FramePadding = ImVec2(8.0f, 4.0f);

            _controlPanel->drawMenus(_sim.get());
			_cmdEditor->drawMenus();
        });

        _GUICntx->postRender();
        _renderCntx->postRender();
    }

	// Initialisation: Sets up the OpenGL context, GUI context, and simulation manager
    bool GLWindow::init(int width, int height, const std::string& header) {
        _width = width;
        _height = height;
        *_header = header;

        // Context layers
        _renderCntx = std::make_unique<render::OpenGLContext>();
        _renderCntx->init(this);

		_window = _renderCntx->getGLFWWindow();

        _GUICntx = std::make_unique<render::GUIContext>();
        _GUICntx->init(this);

        // UI + scene
        _sim = std::make_unique<gui::SimManager>();
		_sim->initGL();
        _controlPanel = std::make_unique<gui::ControlPanel>(_sim.get());
        _debugPanel = std::make_unique<gui::DebugPanel>();
		_cmdEditor = std::make_unique<gui::CommandScriptEditor>(_sim.get());

        _controlPanel->setMeshLoadCallback([this](std::string path) {
                _sim->loadMesh(path);
                LOG_INFO("Mesh loaded: %s", path.c_str());
            }
        );

        _isRunning = true;
        return true;
    }

	// Window resize callback: Enforces 16:9 aspect ratio and updates the OpenGL viewport
    void GLWindow::onResize(int width, int height) {
        if (width <= 0 || height <= 0) return;

        _width = width;
        _height = height;

        // Update the GL viewport for the default framebuffer
        glViewport(0, 0, width, height);
    }

	// Miscellaneous
    bool GLWindow::shouldClose() const { return glfwWindowShouldClose(_window); }
    void GLWindow::pollEvents() { glfwPollEvents(); }
    void GLWindow::swapBuffers() { glfwSwapBuffers(_window); }
    void* window::GLWindow::getNativeWin() { return static_cast<void*>(_window); }
    void window::GLWindow::setNativeWin(void* window) { _window = static_cast<GLFWwindow*>(window);}
    int window::GLWindow::getWidth() const { return _width; }
    int window::GLWindow::getHeight() const { return _height; }
    const std::string& window::GLWindow::getHeader() const { return *_header; }

	// Update loop: Handles input and updates the simulation state
    void GLWindow::update() {
		pollEvents();

        static double lastFrame = glfwGetTime();
        double currentFrame = glfwGetTime();
        float dt = static_cast<float>(currentFrame - lastFrame);
        lastFrame = currentFrame;

        // minimal smoothing (optional)
        static float smoothedDt = 0.016f;
        smoothedDt = glm::mix(smoothedDt, dt, 0.2f);

        if (_sim) {
            _sim->handleContinuousMovement(_window, smoothedDt);
            //_sim->getCamera()->applyGravity(smoothedDt, _sim->getPlaneHeight());
        }
    }

	// Input handling
    void window::GLWindow::setMouseCaptured(bool captured) {
        _mouseCaptured = captured;
        GLFWwindow* w = _window;
		if (!w) { return; }

		// When mouse is captured, disable the cursor and enable raw mouse motion for high-precision input
        if (_mouseCaptured) {
			// When mouse is captured, disable the cursor and enable raw mouse motion for high-precision input
            glfwSetInputMode(w, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            if (glfwRawMouseMotionSupported()) {
                glfwSetInputMode(w, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
            }
            ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouse;
            // tell SimManager to reset its first-mouse state
            if (_sim) { _sim->resetMouseDelta(); }
        }
        else {
			// When mouse is released, show the cursor and disable raw mouse motion
            glfwSetInputMode(w, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            if (glfwRawMouseMotionSupported()) {
                glfwSetInputMode(w, GLFW_RAW_MOUSE_MOTION, GLFW_FALSE);
            }
            ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
        }
    }

	// Toggle mouse capture on Escape key press, and also toggle the control panel visibility
    void window::GLWindow::onKey(int key, int /*scancode*/, int action, int /*mods*/) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            setMouseCaptured(!_mouseCaptured);
            controlPanelOpen = !controlPanelOpen;
            ImGui::SetWindowFocus(nullptr);
            return;
        }
    }

	// Forward scroll events to the SimManager for zooming or other scroll-based interactions
    void window::GLWindow::onScroll(double delta) {
        if (_sim) { _sim->onMouseWheel(delta); }
    }

	// Forward window resize events to the SimManager to adjust the internal rendering resolution and aspect ratio
    void window::GLWindow::onCursorPos(double xpos, double ypos) {
		// LOG_INFO("Mouse moved to: X=%.2f, Y=%.2f", xpos, ypos);
		if (_sim) { _sim->handleMouseLook(_window, xpos, ypos); }
	}

	// Window states
    bool GLWindow::isRunning() const { return _isRunning; }
    void GLWindow::onClose() { _isRunning = false; }
}
