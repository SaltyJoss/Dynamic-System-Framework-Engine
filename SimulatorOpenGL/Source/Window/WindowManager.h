#ifndef WINDOW_MANAGER_H
#define WINDOW_MANAGER_H

#include "ch.h"

#include "Window/Window.h"

#include "Render/GUIContext.h"
#include "Render/OpenGLContext.h"
#include "Render/OpenGLBufferManager.h"

#include "UI/SceneView.h"
#include "UI/ControlPanel.h"

#include "Elements/Camera.h"
#include "Shader/ShaderUtil.h"

using namespace gui;
using namespace render;


/*class WindowManager {
private:
    GLFWwindow* _window;
    int lastX, lastY, lastW, lastH;
    bool isMaximised = false;

public:

    void SetupWindow(GLFWwindow* window) { _window = window; }
    ~WindowManager() = default;

    GLFWwindow* GetWindow() const { return _window; }

    void Render();

};*/

namespace window {
    class GLWindow : public IWindow {
    public:
        GLWindow() {

        }

        ~GLWindow();

        bool init(int w, int h, const std::string& header);
        void render();
        void inputHandler();
        void* getNativeWin() override { return _window; }
        void setNativeWin(void* window) { _window = (GLFWwindow*)window; }

    private:
        GLFWwindow* _window;

        std::unique_ptr<GUIContext> _GUICntx;
    };
}

#endif