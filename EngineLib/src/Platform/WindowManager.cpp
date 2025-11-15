
#include "pch.h"
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

#include "Scene/SceneView.h"
#include "Scene/DebugPanel.h"
#include "Scene/ControlPanel.h"

#include "Scene/Camera.h"
#include "Scene/Light.h"
#include "Scene/Input.h"
#include "Scene/Mesh.h"

#include "Platform/WindowManager.h"

#include "EngineLib/LogMacros.h"

bool controlPanelOpen = false;

namespace window {
/*
 * --------------------------------------------
 *              GLWINDOW METHODS
 * --------------------------------------------
 * 
 * Summary:
 * --------------------------------------------
 * GLWindow() -> Constructor that initializes the GLWindow object.
 * ~GLWindow() -> Destructor that cleans up resources and ends rendering and GUI contexts.
 * render() -> Renders the scene view, control panel, and debug panel.
 * init(int width, int height, const std::string& header) -> Initializes the GLWindow with the specified width, height, and header.
 * onResize(int width, int height) -> Handles window resize events and updates the scene view accordingly.
 * shouldClose() const -> Checks if the window should close.
 * pollEvents() -> Polls for window events.
 * swapBuffers() -> Swaps the front and back buffers of the window.
 * getNativeWin() -> Returns the native GLFW window pointer.
 * setNativeWin(void* window) -> Sets the native GLFW window pointer.
 * getWidth() const -> Returns the width of the window.
 * getHeight() const -> Returns the height of the window.
 * getHeader() const -> Returns the header string of the window.
 * --------------------------------------------
 */
    GLWindow::GLWindow() {
        _header = new std::string();
    }

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

        if (_sceneView)     _sceneView->render();
        if (!controlPanelOpen)  _controlPanel->render(_sceneView.get());

        _GUICntx->postRender();
        _renderCntx->postRender();
    }

    bool GLWindow::init(int width, int height, const std::string& header) {
        _width = width;
        _height = height;
        *_header = header;

        LOG_INFO("Window resized: Width=%d, Height=%d", getWidth(), getHeight());

        // Context layers
        _renderCntx = std::make_unique<render::OpenGLContext>();
        _renderCntx->init(this);

		_window = _renderCntx->getGLFWWindow();

        _GUICntx = std::make_unique<render::GUIContext>();
        _GUICntx->init(this);

        // UI + scene
        _sceneView = std::make_unique<gui::SceneView>();
        _sceneView->resize(_width, _height);
        _controlPanel = std::make_unique<gui::ControlPanel>(_sceneView.get());
        _debugPanel = std::make_unique<gui::DebugPanel>();

        _controlPanel->setMeshLoadCallback([this](std::string path)
            {
                _sceneView->loadMesh(path);
                LOG_INFO("Mesh loaded: %s", path.c_str());
            }
        );

        _isRunning = true;
        return true;
    }

    void GLWindow::onResize(int width, int height) {
        _width = width;
        _height = height;

        _sceneView->resize(_width, _height);
        LOG_INFO("Window resized: Width=%d, Height=%d", width, height);
        render();
    }

	// Miscellaneous
    bool GLWindow::shouldClose() const { return glfwWindowShouldClose(_window); }
    void GLWindow::pollEvents() { glfwPollEvents(); }
    void GLWindow::swapBuffers() { glfwSwapBuffers(_window); }
    void* window::GLWindow::getNativeWin() { return _window; }
    void window::GLWindow::setNativeWin(void* window) { _window = static_cast<GLFWwindow*>(window);}
    int window::GLWindow::getWidth() const { return _width; }
    int window::GLWindow::getHeight() const { return _height; }
    const std::string& window::GLWindow::getHeader() const { return *_header; }

/*
 * --------------------------------------------
 *				USER INTERACTIONS
 * --------------------------------------------
 * 
 * Summary:
 * --------------------------------------------
 * update() -> Updates the window state, handles input, and applies camera movement and gravity.
 * setMouseCaptured(bool captured) -> Sets whether the mouse is captured (disabled) or not.
 * onKey(int key, int scancode, int action, int mods) -> Handles key press events.
 * onScroll(double delta) -> Handles mouse scroll events.
 * onCursorPos(double xpos, double ypos) -> Handles mouse cursor position events.
 * --------------------------------------------
 */

    void GLWindow::update() {
		pollEvents();

        static double lastFrame = glfwGetTime();
        double currentFrame = glfwGetTime();
        float dt = static_cast<float>(currentFrame - lastFrame);
        lastFrame = currentFrame;

        // minimal smoothing (optional)
        static float smoothedDt = 0.016f;
        smoothedDt = glm::mix(smoothedDt, dt, 0.2f);

        if (_sceneView) {
            _sceneView->handleContinuousMovement(_window, smoothedDt);
            _sceneView->getCamera()->applyGravity(smoothedDt, _sceneView->getPlaneHeight());
        }
    }

    void window::GLWindow::setMouseCaptured(bool captured) {
        _mouseCaptured = captured;

        GLFWwindow* w = _window;
        if (!w) return;

        if (_mouseCaptured) {
            glfwSetInputMode(w, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            if (glfwRawMouseMotionSupported())
                glfwSetInputMode(w, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
            // tell SceneView to reset its first-mouse state
            if (_sceneView) _sceneView->resetMouseDelta();
        }
        else {
            glfwSetInputMode(w, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            if (glfwRawMouseMotionSupported())
                glfwSetInputMode(w, GLFW_RAW_MOUSE_MOTION, GLFW_FALSE);
        }
    }

    void window::GLWindow::onKey(int key, int scancode, int action, int mods) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            setMouseCaptured(!_mouseCaptured);
			// close control panel when key is pressed again
            controlPanelOpen = !controlPanelOpen;
            return;
        }

        if (action == GLFW_PRESS && key == GLFW_KEY_SPACE) {
            _sceneView->getCamera()->jump();
        }
    }

    void window::GLWindow::onScroll(double delta) {
        if (_sceneView) { _sceneView->onMouseWheel(delta); }
    }

    void window::GLWindow::onCursorPos(double xpos, double ypos) {
		// LOG_INFO("Mouse moved to: X=%.2f, Y=%.2f", xpos, ypos);
		if (_sceneView) { _sceneView->handleMouseLook(_window, xpos, ypos); }
	}

/*
 * --------------------------------------------
 *				WINDOW STATES
 * --------------------------------------------
 * 
 * Summary:
 * --------------------------------------------
 * isRunning() const -> Checks if the window is currently running.
 * onClose() -> Handles window close events by setting the running state to false.
 * --------------------------------------------
 */

    bool GLWindow::isRunning() const { return _isRunning; }
    void GLWindow::onClose() { _isRunning = false; }
}
