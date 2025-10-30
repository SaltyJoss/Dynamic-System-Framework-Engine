#include "WindowManager.h"



/*void WindowManager::Minimise() { glfwSetWindowShouldClose(_window, true); }

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