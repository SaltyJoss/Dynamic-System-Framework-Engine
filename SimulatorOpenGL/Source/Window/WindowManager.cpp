#include "ch.h"

#include "WindowManager.h"
#include "Elements/Input.h"

namespace window {
    GLWindow::~GLWindow() { _renderCntx->end(); _GUICntx->end(); }

    bool GLWindow::init(int width, int height, const std::string& header) {
        fprintf(stderr, "[INSIDE GLWindow::Init()] Before -> Width: %d, Height: %d, Header: %s\n", _width, _height, _header.c_str());

        _width = width;
        _height = height;
        _header = header;

        _renderCntx->init(this);
        _GUICntx->init(this);

        _winSize = ImGui::GetIO().DisplaySize;
        _padding = ImGui::GetStyle().WindowPadding;

        _sceneView = std::make_unique<SceneView>();
        _controlPanel = std::make_unique<ControlPanel>();
        _debugPanel = std::make_unique<DebugPanel>();

        _controlPanel->setMeshLoadCallback([this](std::string path) { _sceneView->loadMesh(path); });

        return _isRunning;
    }

    void GLWindow::render() {
        _GUICntx->preRender();
        _renderCntx->preRender();

        _sceneView->render();
        _controlPanel->render(_sceneView.get());
        _debugPanel->render();

        _GUICntx->postRender();
        _renderCntx->postRender();

        inputHandler();
    }

    void GLWindow::onResize(int width, int height) {
        _width = width;
        _height = height;

        _sceneView->resize(_width, _height);
        render();
    }

    void GLWindow::inputHandler() {
        if (glfwGetKey(_window, GLFW_KEY_W) == GLFW_PRESS) { _sceneView->onMouseWheel(-0.4f); }
        if (glfwGetKey(_window, GLFW_KEY_S) == GLFW_PRESS) { _sceneView->onMouseWheel(0.4f); }
        if (glfwGetKey(_window, GLFW_KEY_F) == GLFW_PRESS) { _sceneView->resetView(); }

        double x, y;
        glfwGetCursorPos(_window, &x, &y);

        _sceneView->onMouseMove(x, y, Input::GetPressedButton(_window));
    }

    void GLWindow::onScroll(double delta) { _sceneView->onMouseWheel(delta); }
    void GLWindow::onKey(int key, int scancode, int action, int mods) { if (action == GLFW_PRESS) {} }
    void GLWindow::onClose() { _isRunning = false; }
}

/*
OLD LOGIC -- DELETE SOON, KEEP WHILE STILL TUNING NEW LOGIC

*void WindowManager::Minimise() { glfwSetWindowShouldClose(_window, true); }

void WindowManager::Close(){ glfwIconifyWindow(_window); }

void WindowManager::ToggleMaximise() {
    isMaximised = !isMaximised;

    if (isMaximised) {
        glfwGetWindowPos(_window, &lastX, &lastY);
        glfwGetWindowSize(_window, &lastW, &lastH);

        int wx, wy, ww, wh;
        glfwGetWindowPos(_window, &wx, &wy);
        glfwGetWindowSize(_window, &ww, &wh);
        int cx = wx + ww / 2;
        int cy = wy + wh / 2;

        int count;
        GLFWmonitor** monitors = glfwGetMonitors(&count);
        GLFWmonitor* targetMonitor = monitors[0];
        for (int i = 0; i < count; i++) {
            int mx, my;
            glfwGetMonitorPos(monitors[i], &mx, &my);
            const GLFWvidmode* mode = glfwGetVideoMode(monitors[i]);
            if (cx >= mx && cx <= mx + mode->width &&
                cy >= my && cy <= my + mode->height) {
                targetMonitor = monitors[i];
                break;
            }
        }

        int mx, my;
        glfwGetMonitorPos(targetMonitor, &mx, &my);
        const GLFWvidmode* mode = glfwGetVideoMode(targetMonitor);

        glfwSetWindowMonitor(_window, nullptr, mx, my, mode->width, mode->height, 0);
    }
    else {
        RestoreWindowed();
    }
}

void WindowManager::RestoreWindowed() {
    glfwSetWindowMonitor(_window, nullptr, lastX, lastY, lastW, lastH, 0);
}*/