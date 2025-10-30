#ifndef WINDOW_MANAGER_H
#define WINDOW_MANAGER_H

#include <GLFW/glfw3.h>
#include "ch.h"

class WindowManager {
private:
    GLFWwindow* _window;
    int lastX, lastY, lastW, lastH;
    bool isMaximised = false;

public:

    void SetupWindow(GLFWwindow* window) { _window = window; }
    ~WindowManager() = default;

    GLFWwindow* GetWindow() const { return _window; }

    void Render();

};

#endif